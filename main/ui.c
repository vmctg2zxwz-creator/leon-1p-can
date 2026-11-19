#include "ui.h"
#include <stdio.h>
#include "lvgl.h"
#include "can.h"

/*
 * Απλή οθόνη: τρεις μεγάλοι αριθμοί, ο ένας κάτω από τον άλλο.
 * ΣΤΡΟΦΕΣ / ΝΕΡΟ / ΕΙΣΑΓΩΓΗ. "--" όταν δεν έρχονται δεδομένα.
 */

#define BIG_SCALE 768   /* 256 = 1x, 768 = 3x το font 48 */

static lv_obj_t *s_rpm, *s_coolant, *s_intake;

static lv_obj_t *make_row(lv_obj_t *parent, const char *title, const char *unit)
{
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_set_width(row, lv_pct(100));
    lv_obj_set_flex_grow(row, 1);
    lv_obj_set_style_bg_color(row, lv_color_hex(0x111111), 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_radius(row, 16, 0);
    lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *t = lv_label_create(row);
    lv_label_set_text(t, title);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(t, lv_color_hex(0x9e9e9e), 0);
    lv_obj_align(t, LV_ALIGN_TOP_LEFT, 10, 10);

    lv_obj_t *u = lv_label_create(row);
    lv_label_set_text(u, unit);
    lv_obj_set_style_text_font(u, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(u, lv_color_hex(0x9e9e9e), 0);
    lv_obj_align(u, LV_ALIGN_BOTTOM_RIGHT, -10, -10);

    lv_obj_t *v = lv_label_create(row);
    lv_label_set_text(v, "--");
    lv_obj_set_style_text_font(v, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(v, lv_color_white(), 0);
    lv_obj_set_style_transform_scale(v, BIG_SCALE, 0);
    lv_obj_set_style_transform_pivot_x(v, lv_pct(50), 0);
    lv_obj_set_style_transform_pivot_y(v, lv_pct(50), 0);
    lv_obj_align(v, LV_ALIGN_CENTER, 0, 0);
    return v;
}

static void set_value(lv_obj_t *lbl, bool ok, int value, lv_color_t color)
{
    if (ok) {
        lv_label_set_text_fmt(lbl, "%d", value);
    } else {
        lv_label_set_text(lbl, "--");
    }
    lv_obj_set_style_text_color(lbl, color, 0);
}

static void refresh_cb(lv_timer_t *t)
{
    car_data_t d = can_get();
    lv_color_t white = lv_color_white();
    lv_color_t red = lv_color_hex(0xff3b30);
    lv_color_t blue = lv_color_hex(0x40a9ff);

    set_value(s_rpm, d.rpm_ok, (int)d.rpm, d.rpm > 6000 ? red : white);
    set_value(s_coolant, d.coolant_ok, (int)d.coolant_c,
              d.coolant_c > 105 ? red : (d.coolant_c < 70 ? blue : white));
    set_value(s_intake, d.intake_ok, (int)d.intake_c, d.intake_c > 60 ? red : white);
}

void ui_create(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_pad_all(scr, 16, 0);
    lv_obj_set_style_pad_row(scr, 16, 0);
    lv_obj_set_flex_flow(scr, LV_FLEX_FLOW_COLUMN);

    s_rpm = make_row(scr, "RPM", "rpm");
    s_coolant = make_row(scr, "COOLANT", "°C");
    s_intake = make_row(scr, "INTAKE AIR", "°C");

    lv_timer_create(refresh_cb, 100, NULL);
}
