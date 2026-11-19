#include "ui.h"
#include "lvgl.h"
#include "obd.h"

#define RPM_MAX 7000

static lv_obj_t *s_rpm_arc;
static lv_obj_t *s_rpm_lbl;
static lv_obj_t *s_speed_lbl;
static lv_obj_t *s_coolant_lbl;
static lv_obj_t *s_boost_lbl;
static lv_obj_t *s_iat_lbl;
static lv_obj_t *s_volt_lbl;
static lv_obj_t *s_status_lbl;

static lv_obj_t *make_tile(lv_obj_t *parent, const char *title, lv_obj_t **value)
{
    lv_obj_t *box = lv_obj_create(parent);
    lv_obj_set_size(box, 170, 100);
    lv_obj_set_style_bg_color(box, lv_color_hex(0x1a1a1a), 0);
    lv_obj_set_style_border_width(box, 0, 0);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *t = lv_label_create(box);
    lv_label_set_text(t, title);
    lv_obj_set_style_text_color(t, lv_color_hex(0x9e9e9e), 0);
    lv_obj_align(t, LV_ALIGN_TOP_LEFT, 0, 0);

    *value = lv_label_create(box);
    lv_label_set_text(*value, "--");
    lv_obj_set_style_text_font(*value, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(*value, lv_color_white(), 0);
    lv_obj_align(*value, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    return box;
}

static void refresh_cb(lv_timer_t *t)
{
    car_data_t d = obd_get();

    lv_label_set_text(s_status_lbl, d.connected ? "CAN OK" : "No CAN");
    lv_obj_set_style_text_color(s_status_lbl,
        d.connected ? lv_color_hex(0x4caf50) : lv_color_hex(0xf44336), 0);
    if (!d.connected) {
        return;
    }

    int rpm = (int)d.rpm;
    lv_arc_set_value(s_rpm_arc, rpm > RPM_MAX ? RPM_MAX : rpm);
    lv_obj_set_style_arc_color(s_rpm_arc,
        rpm > 4500 ? lv_color_hex(0xf44336) : lv_color_hex(0x03a9f4), LV_PART_INDICATOR);
    lv_label_set_text_fmt(s_rpm_lbl, "%d", rpm);
    lv_label_set_text_fmt(s_speed_lbl, "%d km/h", (int)d.speed_kmh);

    lv_label_set_text_fmt(s_coolant_lbl, "%d °C", (int)d.coolant_c);
    lv_obj_set_style_text_color(s_coolant_lbl,
        d.coolant_c > 105 ? lv_color_hex(0xf44336) : lv_color_white(), 0);

    /* Boost/μπαταρία: μένουν "--" μέχρι να επιβεβαιωθεί η πηγή τους. */

    lv_label_set_text_fmt(s_iat_lbl, "%d °C", (int)d.intake_c);
}

void ui_create(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    /* Στροφόμετρο */
    s_rpm_arc = lv_arc_create(scr);
    lv_obj_set_size(s_rpm_arc, 360, 360);
    lv_obj_align(s_rpm_arc, LV_ALIGN_LEFT_MID, 30, 20);
    lv_arc_set_rotation(s_rpm_arc, 135);
    lv_arc_set_bg_angles(s_rpm_arc, 0, 270);
    lv_arc_set_range(s_rpm_arc, 0, RPM_MAX);
    lv_arc_set_value(s_rpm_arc, 0);
    lv_obj_remove_style(s_rpm_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(s_rpm_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(s_rpm_arc, 24, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_rpm_arc, 24, LV_PART_INDICATOR);

    s_rpm_lbl = lv_label_create(scr);
    lv_obj_set_style_text_font(s_rpm_lbl, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_rpm_lbl, lv_color_white(), 0);
    lv_label_set_text(s_rpm_lbl, "0");
    lv_obj_align_to(s_rpm_lbl, s_rpm_arc, LV_ALIGN_CENTER, 0, -10);

    s_speed_lbl = lv_label_create(scr);
    lv_obj_set_style_text_font(s_speed_lbl, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(s_speed_lbl, lv_color_hex(0xbdbdbd), 0);
    lv_label_set_text(s_speed_lbl, "0 km/h");
    lv_obj_align_to(s_speed_lbl, s_rpm_arc, LV_ALIGN_CENTER, 0, 40);

    /* Πλακίδια δεξιά */
    lv_obj_t *grid = lv_obj_create(scr);
    lv_obj_set_size(grid, 380, 340);
    lv_obj_align(grid, LV_ALIGN_RIGHT_MID, -10, 20);
    lv_obj_set_style_bg_opa(grid, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(grid, 0, 0);
    lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_style_pad_gap(grid, 12, 0);

    make_tile(grid, "Coolant", &s_coolant_lbl);
    make_tile(grid, "Boost", &s_boost_lbl);
    make_tile(grid, "Intake air", &s_iat_lbl);
    make_tile(grid, "Battery", &s_volt_lbl);

    s_status_lbl = lv_label_create(scr);
    lv_label_set_text(s_status_lbl, "No CAN");
    lv_obj_align(s_status_lbl, LV_ALIGN_TOP_RIGHT, -20, 15);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "LEON 1P");
    lv_obj_set_style_text_color(title, lv_color_hex(0x9e9e9e), 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 20, 15);

    lv_timer_create(refresh_cb, 100, NULL);
}
