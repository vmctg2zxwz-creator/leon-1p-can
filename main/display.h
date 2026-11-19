#pragma once
#include "lvgl.h"

/* Ανάλυση οθόνης. ΑΛΛΑΞΕ ανάλογα με την πλακέτα. */
#define DISP_HOR_RES 800
#define DISP_VER_RES 480

/* Αρχικοποιεί την οθόνη (MIPI-DSI panel) και επιστρέφει το LVGL display. */
lv_display_t *display_init(void);
