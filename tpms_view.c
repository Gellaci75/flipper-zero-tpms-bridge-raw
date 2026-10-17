#include "tpms_view.h"
#include "laszlo_portrait_40.h"
#include "tpms_radio.h"

#include <furi.h>
#include <gui/elements.h>
#include <string.h>

/* List layout: header, 10 px rows, footer with key hints. */
#define TPMS_ROW_TOP    14
#define TPMS_ROW_HEIGHT 10
#define TPMS_LIST_LEFT  2

/* Columns of a list row. Exact dBm live on the detail screen: there is no
 * room for them in a row that would not be taken from the readings. */
#define TPMS_COL_PRESSURE 42
#define TPMS_COL_TEMP     78
#define TPMS_COL_BARS     104

/* Rows show the last six digits of an id. Some protocols have eight, and
 * a row is not wide enough for all of them next to the readings; six are
 * plenty to tell one wheel from another, and the detail screen has the
 * whole thing. */
#define TPMS_ROW_ID_DIGITS 6
#define TPMS_WAKE_FEEDBACK_MS 1500

/** Bars in the signal level indicator. */
#define TPMS_BAR_COUNT 4
#define TPMS_BAR_WIDTH 3
#define TPMS_BAR_STEP  4

/* Lines never exceed the screen width, but keep some headroom anyway. */
#define TPMS_TEXT_MAX 48

/** Level in bars. Thresholds are tuned for a sensor inside a wheel: right
 * against the Flipper that is about -60 dBm, across the car body about
 * -90 dBm. */
static uint8_t tpms_view_bars(int16_t rssi_x10) {
    const int32_t dbm = rssi_x10 / 10;
    if(dbm >= -65) return 4;
    if(dbm >= -75) return 3;
    if(dbm >= -85) return 2;
    if(dbm >= -95) return 1;
    return 0;
}

/** Ladder of bars. Stale readings are drawn as outlines: the value is old
 * because the sensor has been silent for a while. */
static void
    tpms_view_draw_signal(Canvas* canvas, int32_t x, int32_t bottom, uint8_t bars, bool filled) {
    for(uint8_t i = 0; i < TPMS_BAR_COUNT; i++) {
        const int32_t height = 2 + i * 2;
        const int32_t bar_x = x + i * TPMS_BAR_STEP;

        if(i < bars) {
            if(filled) {
                canvas_draw_box(canvas, bar_x, bottom - height, TPMS_BAR_WIDTH, height);
            } else {
                canvas_draw_frame(canvas, bar_x, bottom - height, TPMS_BAR_WIDTH, height);
            }
        } else {
            canvas_draw_dot(canvas, bar_x + 1, bottom - 1);
        }
    }
}

/** Pressure in bar, two decimals. Everything is carried in hundredths of
 * a kPa, and a bar is a hundred kPa. */
static void tpms_view_format_bar(char* out, size_t size, int32_t pressure_kpa_x100) {
    const int32_t bar_x100 = pressure_kpa_x100 / 100;
    snprintf(out, size, "%ld.%02ld", (long)(bar_x100 / 100), (long)(bar_x100 % 100));
}

/** A signed value in tenths -> "-86.5". */
static void tpms_view_format_x10(char* out, size_t size, int32_t value_x10) {
    const int32_t magnitude = value_x10 < 0 ? -value_x10 : value_x10;
    snprintf(
        out,
        size,
        "%s%ld.%ld",
        value_x10 < 0 ? "-" : "",
        (long)(magnitude / 10),
        (long)(magnitude % 10));
}

static void tpms_view_format_age(char* out, size_t size, uint32_t age_ticks) {
    const uint32_t seconds = age_ticks / furi_ms_to_ticks(1000);
    if(seconds < 100) {
        snprintf(out, size, "%lus", (unsigned long)seconds);
    } else {
        snprintf(out, size, "%lum", (unsigned long)(seconds / 60));
    }
}

/** Sensor id, as wide as its protocol makes it. */
static void tpms_view_format_id(char* out, size_t size, const TpmsSensor* sensor, uint8_t digits) {
    if(digits == 0) {
        digits = sensor->protocol < tpms_protocol_count ?
                     tpms_protocols[sensor->protocol].id_digits :
                     6;
    }
    const uint32_t mask = digits >= 8 ? 0xFFFFFFFFUL : ((1UL << (digits * 4)) - 1);
    snprintf(out, size, "%0*lx", (int)digits, (unsigned long)(sensor->id & mask));
}

static void tpms_view_draw_light_icon(Canvas* canvas, bool on) {
    const int32_t x = 61;
    const int32_t y = 2;
    if(on) {
        canvas_draw_frame(canvas, x + 1, y + 3, 6, 5);
        canvas_draw_line(canvas, x + 2, y + 8, x + 6, y + 8);
        canvas_draw_line(canvas, x + 3, y + 9, x + 5, y + 9);
        canvas_draw_dot(canvas, x + 4, y);
        canvas_draw_dot(canvas, x + 1, y + 1);
        canvas_draw_dot(canvas, x + 7, y + 1);
    } else {
        canvas_draw_box(canvas, x + 1, y + 3, 6, 5);
        canvas_draw_line(canvas, x + 2, y + 8, x + 6, y + 8);
        canvas_draw_line(canvas, x + 3, y + 9, x + 5, y + 9);
    }
}

static void tpms_view_draw_header(Canvas* canvas, TpmsBridgeApp* app) {
    /* The radio setting takes the place a title would: which band and
     * modulation the app is listening on is the one thing that decides
     * whether anything can show up at all. While scanning, AUTO says the
     * setting is about to move on by itself. */
    const bool scanning = tpms_config_is_auto((TpmsConfig)app->config);
    const char* band = tpms_slot_frequency(app->active_slot) == tpms_frequencies[0] ? "433" :
                                                                                      "315";
    const char mode = tpms_slot_modulation(app->active_slot) == TpmsModulationOok ?
                          (scanning ? 'o' : 'O') :
                          (scanning ? 'f' : 'F');

    char setting[TPMS_TEXT_MAX];
    snprintf(setting, sizeof(setting), "%s%s%c", scanning ? "auto" : "", band, mode);

    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, TPMS_LIST_LEFT, 10, setting);
    tpms_view_draw_light_icon(canvas, app->keep_backlight_on);

    const char* radio = "RX OFF";
    if(app->exit_blocked) {
        radio = "wait";
    } else if(app->usb_streaming) {
        radio = "USB";
    } else if(app->rx_device_ready) {
        radio = app->rx_device_external ? "RX EXT" : "RX INT";
    } else if(app->rx_enabled && app->radio_start_error) {
        radio = "RX ERR";
    } else if(app->rx_enabled) {
        radio = "RX wait";
    }

    char status[TPMS_TEXT_MAX];
    snprintf(status, sizeof(status), "%u %s", (unsigned)app->store.count, radio);

    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str_aligned(canvas, 126, 10, AlignRight, AlignBottom, status);
}

static void tpms_view_draw_row(
    Canvas* canvas,
    const TpmsSensor* sensor,
    int32_t top,
    bool selected,
    bool fresh) {
    if(selected) {
        canvas_draw_box(canvas, 0, top, 124, TPMS_ROW_HEIGHT);
        canvas_set_color(canvas, ColorWhite);
    }

    const int32_t baseline = top + 8;
    char text[TPMS_TEXT_MAX];

    char id[TPMS_TEXT_MAX];
    tpms_view_format_id(id, sizeof(id), sensor, TPMS_ROW_ID_DIGITS);
    if(sensor->frames > 1) {
        char count[16];
        snprintf(count, sizeof(count), "%lux ", (unsigned long)sensor->frames);
        strlcpy(text, count, sizeof(text));
        strlcat(text, id, sizeof(text));
    } else {
        strlcpy(text, id, sizeof(text));
    }
    canvas_draw_str(canvas, TPMS_LIST_LEFT, baseline, text);

    if(sensor->have & TPMS_HAS_PRESSURE) {
        char bar[16];
        tpms_view_format_bar(bar, sizeof(bar), sensor->pressure_kpa_x100);
        snprintf(text, sizeof(text), "%sb", bar);
    } else {
        snprintf(text, sizeof(text), "--");
    }
    canvas_draw_str(canvas, TPMS_COL_PRESSURE, baseline, text);

    if(sensor->have & TPMS_HAS_TEMP) {
        snprintf(text, sizeof(text), "%dC", (int)sensor->temperature_c);
    } else {
        snprintf(text, sizeof(text), "--");
    }
    canvas_draw_str(canvas, TPMS_COL_TEMP, baseline, text);

    tpms_view_draw_signal(
        canvas, TPMS_COL_BARS, top + TPMS_ROW_HEIGHT - 1, tpms_view_bars(sensor->rssi_x10), fresh);

    if(selected) canvas_set_color(canvas, ColorBlack);
}

static void tpms_view_draw_list(Canvas* canvas, TpmsBridgeApp* app) {
    tpms_view_draw_header(canvas, app);
    canvas_set_font(canvas, FontSecondary);
    const uint32_t now = furi_get_tick();
    const bool wake_feedback =
        app->wake_feedback_tick &&
        (now - app->wake_feedback_tick < furi_ms_to_ticks(TPMS_WAKE_FEEDBACK_MS));

    char wake_text[16];
    if(!app->auto_wake) {
        snprintf(wake_text, sizeof(wake_text), "Wake Off");
    } else if(wake_feedback) {
        snprintf(wake_text, sizeof(wake_text), "Wake pulse");
    } else if(app->local_rx && app->wake_next_tick != 0) {
        int32_t ticks_left = (int32_t)(app->wake_next_tick - now);
        uint32_t seconds = 1;
        if(ticks_left > 0) {
            seconds = (uint32_t)((ticks_left + (int32_t)furi_ms_to_ticks(999)) /
                                 (int32_t)furi_ms_to_ticks(1000));
        }
        if(seconds > 9) seconds = 9;
        snprintf(wake_text, sizeof(wake_text), "Wake %lus", (unsigned long)seconds);
    } else {
        snprintf(wake_text, sizeof(wake_text), "Wake On");
    }

    if(app->store.count == 0) {
        const char* spinner[] = {"|", "/", "-", "\\"};
        const char* rx = app->rx_device_ready ? spinner[(now / 250) % 4] : "off";
        char watch[TPMS_TEXT_MAX];
        if(app->rx_device_ready) {
            snprintf(watch, sizeof(watch), "RX %s watching %s", app->rx_device_external ? "EXT" : "INT", rx);
        } else if(app->rx_enabled && app->radio_start_error) {
            snprintf(watch, sizeof(watch), "RX ERROR - retrying");
        } else {
            snprintf(watch, sizeof(watch), app->rx_enabled ? "RX starting" : "RX OFF");
        }
        canvas_draw_str_aligned(canvas, 64, 27, AlignCenter, AlignBottom, watch);
        snprintf(
            watch,
            sizeof(watch),
            "Frames:%lu IDs:%u",
            (unsigned long)app->store.total_frames,
            (unsigned)app->store.count);
        canvas_draw_str_aligned(canvas, 64, 39, AlignCenter, AlignBottom, watch);
        canvas_draw_str_aligned(
            canvas,
            64,
            50,
            AlignCenter,
            AlignBottom,
            wake_feedback ? "Wake pulse" : "Up/Down: band, mode");
    } else {
        for(uint8_t row = 0; row < TPMS_VIEW_ROWS; row++) {
            const uint8_t index = app->scroll + row;
            if(index >= app->store.count) break;

            const TpmsSensor* sensor = &app->store.items[index];
            tpms_view_draw_row(
                canvas,
                sensor,
                TPMS_ROW_TOP + row * TPMS_ROW_HEIGHT,
                index == app->selected,
                !tpms_sensor_is_stale(sensor, now));
        }

        if(app->store.count > TPMS_VIEW_ROWS) {
            elements_scrollbar_pos(
                canvas,
                128,
                TPMS_ROW_TOP,
                TPMS_VIEW_ROWS * TPMS_ROW_HEIGHT,
                app->selected,
                app->store.count);
        }
    }

    /* Spread the hints across the width: they do not fit as one string. */
    canvas_draw_str(canvas, TPMS_LIST_LEFT, 62, app->raw_save ? "RAW On" : "RAW Off");
    canvas_draw_str_aligned(canvas, 64, 62, AlignCenter, AlignBottom, wake_text);
    canvas_draw_str_aligned(canvas, 126, 62, AlignRight, AlignBottom, "R:files");
}

static void tpms_view_draw_detail(Canvas* canvas, TpmsBridgeApp* app) {
    const TpmsSensor* sensor = &app->store.items[app->selected];
    char text[TPMS_TEXT_MAX];
    char left[16];
    char right[16];

    canvas_set_font(canvas, FontPrimary);
    tpms_view_format_id(left, sizeof(left), sensor, 0);
    snprintf(text, sizeof(text), "#%u %s", (unsigned)(app->selected + 1), left);
    canvas_draw_str(canvas, TPMS_LIST_LEFT, 10, text);

    tpms_view_draw_signal(
        canvas,
        TPMS_COL_BARS,
        11,
        tpms_view_bars(sensor->rssi_x10),
        !tpms_sensor_is_stale(sensor, furi_get_tick()));

    canvas_set_font(canvas, FontSecondary);

    /* Pressure is shown in all three units: bar is what most people use,
     * PSI is printed on the sensors themselves, kPa is what most of these
     * protocols carry. */
    if(sensor->have & TPMS_HAS_PRESSURE) {
        const int32_t kpa_x100 = sensor->pressure_kpa_x100;
        const int32_t psi_x10 = kpa_x100 * 1000 / 68948;

        tpms_view_format_bar(left, sizeof(left), kpa_x100);
        snprintf(text, sizeof(text), "%s bar", left);
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 22, text);
        tpms_view_format_x10(right, sizeof(right), psi_x10);
        snprintf(text, sizeof(text), "%s PSI", right);
        canvas_draw_str_aligned(canvas, 126, 22, AlignRight, AlignBottom, text);

        snprintf(
            text, sizeof(text), "%ld.%02ld kPa", (long)(kpa_x100 / 100), (long)(kpa_x100 % 100));
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 32, text);
    } else {
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 22, "no pressure");
    }

    if(sensor->have & TPMS_HAS_TEMP) {
        snprintf(text, sizeof(text), "%d C", (int)sensor->temperature_c);
        canvas_draw_str_aligned(canvas, 126, 32, AlignRight, AlignBottom, text);
    }

    tpms_view_format_x10(left, sizeof(left), sensor->rssi_x10);
    snprintf(text, sizeof(text), "%s dBm", left);
    canvas_draw_str(canvas, TPMS_LIST_LEFT, 42, text);
    /* Whole dBm are enough for the peak: it is there to compare wheels on
     * the move, not to measure precisely. */
    snprintf(text, sizeof(text), "peak %ld", (long)(sensor->peak_rssi_x10 / 10));
    canvas_draw_str_aligned(canvas, 126, 42, AlignRight, AlignBottom, text);

    snprintf(text, sizeof(text), "frames %lu", (unsigned long)sensor->frames);
    canvas_draw_str(canvas, TPMS_LIST_LEFT, 52, text);
    tpms_view_format_age(left, sizeof(left), furi_get_tick() - sensor->last_tick);
    snprintf(text, sizeof(text), "age %s", left);
    canvas_draw_str_aligned(canvas, 126, 52, AlignRight, AlignBottom, text);

    /* Which protocol this frame turned out to be, and its status bits. */
    canvas_draw_str(canvas, TPMS_LIST_LEFT, 62, tpms_protocol_label(sensor->protocol));
    if(sensor->have & TPMS_BATTERY_LOW) {
        snprintf(text, sizeof(text), "bat %02x", (unsigned)sensor->flags);
    } else {
        snprintf(text, sizeof(text), "%02x", (unsigned)sensor->flags);
    }
    canvas_draw_str_aligned(canvas, 126, 62, AlignRight, AlignBottom, text);
}

static void tpms_view_draw_raw_title(Canvas* canvas, const char* title) {
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, TPMS_LIST_LEFT, 10, title);
}

static void tpms_view_draw_raw_files(Canvas* canvas, TpmsBridgeApp* app) {
    tpms_view_draw_raw_title(canvas, "RAW files");
    canvas_set_font(canvas, FontSecondary);

    if(app->raw_file_count == 0) {
        canvas_draw_str_aligned(canvas, 64, 30, AlignCenter, AlignBottom, "No .sub files");
        canvas_draw_str_aligned(canvas, 64, 42, AlignCenter, AlignBottom, "Record first");
    } else {
        for(uint8_t row = 0; row < TPMS_RAW_FILE_ROWS; row++) {
            const uint8_t index = app->raw_file_scroll + row;
            if(index >= app->raw_file_count) break;

            const int32_t top = TPMS_ROW_TOP + row * TPMS_ROW_HEIGHT;
            if(index == app->raw_file_selected) {
                canvas_draw_box(canvas, 0, top, 118, TPMS_ROW_HEIGHT);
                canvas_set_color(canvas, ColorWhite);
            }

            char text[TPMS_TEXT_MAX];
            snprintf(text, sizeof(text), "%u %s", (unsigned)(index + 1), app->raw_file_names[index]);
            if(strlen(text) > 28) {
                text[25] = '.';
                text[26] = '.';
                text[27] = '.';
                text[28] = '\0';
            }
            canvas_draw_str(canvas, TPMS_LIST_LEFT, top + 8, text);

            if(index == app->raw_file_selected) canvas_set_color(canvas, ColorBlack);
        }

        if(app->raw_file_count > TPMS_RAW_FILE_ROWS) {
            elements_scrollbar_pos(
                canvas,
                128,
                TPMS_ROW_TOP,
                TPMS_RAW_FILE_ROWS * TPMS_ROW_HEIGHT,
                app->raw_file_selected,
                app->raw_file_count);
        }
    }

    canvas_draw_str(canvas, TPMS_LIST_LEFT, 62, "OK:menu");
    canvas_draw_str_aligned(canvas, 64, 62, AlignCenter, AlignBottom, "R:help");
    canvas_draw_str_aligned(canvas, 126, 62, AlignRight, AlignBottom, "Back");
}

static const char* tpms_view_raw_action_label(uint8_t action) {
    switch(action) {
    case 0:
        return "Open";
    case 1:
        return "Play";
    case 2:
        return "Raw info";
    case 3:
        return "Rename";
    case 4:
        return "Delete";
    case 5:
        return "Delete all";
    default:
        return "";
    }
}

static void tpms_view_draw_raw_menu(Canvas* canvas, TpmsBridgeApp* app) {
    tpms_view_draw_raw_title(canvas, "RAW action");
    canvas_set_font(canvas, FontSecondary);

    canvas_draw_str(canvas, TPMS_LIST_LEFT, 18, app->raw_file_names[app->raw_file_selected]);
    for(uint8_t row = 0; row < 6; row++) {
        const int32_t top = 19 + row * 7;
        const char* label = tpms_view_raw_action_label(row);
        if(row == app->raw_menu_selected) {
            canvas_draw_box(canvas, TPMS_LIST_LEFT, top, canvas_string_width(canvas, label) + 8, 7);
            canvas_set_color(canvas, ColorWhite);
        }
        canvas_draw_str(canvas, TPMS_LIST_LEFT + 4, top + 6, label);
        if(row == app->raw_menu_selected) canvas_set_color(canvas, ColorBlack);
    }

    canvas_draw_str_aligned(canvas, 126, 62, AlignRight, AlignBottom, "Back");
}

static void tpms_view_draw_raw_info(Canvas* canvas, TpmsBridgeApp* app) {
    char text[TPMS_TEXT_MAX];

    tpms_view_draw_raw_title(canvas, "RAW info");
    canvas_set_font(canvas, FontSecondary);

    canvas_draw_str(canvas, TPMS_LIST_LEFT, 20, app->raw_file_names[app->raw_file_selected]);
    snprintf(text, sizeof(text), "Size:%lu", (unsigned long)app->raw_file_size);
    canvas_draw_str(canvas, TPMS_LIST_LEFT, 31, text);
    snprintf(text, sizeof(text), "F:%lu", (unsigned long)app->raw_file_frequency);
    canvas_draw_str(canvas, TPMS_LIST_LEFT, 41, app->raw_file_frequency ? text : "F:-");
    snprintf(text, sizeof(text), "P:%s", app->raw_file_protocol[0] ? app->raw_file_protocol : "-");
    canvas_draw_str(canvas, TPMS_LIST_LEFT, 51, text);
    canvas_draw_str(canvas, TPMS_LIST_LEFT, 61, app->raw_file_tpms_valid ? "TPMS meta: yes" : "TPMS meta: no");
}

static void tpms_view_draw_raw_open(Canvas* canvas, TpmsBridgeApp* app) {
    char text[TPMS_TEXT_MAX];

    tpms_view_draw_raw_title(canvas, "Open");
    canvas_set_font(canvas, FontSecondary);

    canvas_draw_str(canvas, TPMS_LIST_LEFT, 20, app->raw_file_names[app->raw_file_selected]);
    if(app->raw_file_tpms_valid) {
        const uint8_t proto = app->raw_file_tpms_protocol;
        snprintf(
            text,
            sizeof(text),
            "%s ID:%0*lx",
            tpms_protocol_label(proto),
            proto < tpms_protocol_count ? tpms_protocols[proto].id_digits : 6,
            (unsigned long)app->raw_file_tpms_id);
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 30, text);

        if(app->raw_file_tpms_have & TPMS_HAS_PRESSURE) {
            char bar[16];
            tpms_view_format_bar(bar, sizeof(bar), app->raw_file_tpms_pressure_kpa_x100);
            snprintf(text, sizeof(text), "P:%sbar", bar);
        } else {
            snprintf(text, sizeof(text), "P:-");
        }
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 40, text);

        if(app->raw_file_tpms_have & TPMS_HAS_TEMP) {
            snprintf(text, sizeof(text), "T:%dC", (int)app->raw_file_tpms_temperature_c);
        } else {
            snprintf(text, sizeof(text), "T:-");
        }
        if(app->raw_file_tpms_have & TPMS_HAS_BATTERY) {
            snprintf(
                text + strlen(text),
                sizeof(text) - strlen(text),
                " Bat:%s",
                (app->raw_file_tpms_have & TPMS_BATTERY_LOW) ? "LOW" : "OK");
        }
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 50, text);
        if(app->raw_file_tpms_rssi_x10 != 0) {
            char rssi[16];
            tpms_view_format_x10(rssi, sizeof(rssi), app->raw_file_tpms_rssi_x10);
            snprintf(text, sizeof(text), "RSSI:%s Bat:%s", rssi, (app->raw_file_tpms_have & TPMS_HAS_BATTERY) ? ((app->raw_file_tpms_have & TPMS_BATTERY_LOW) ? "LOW" : "OK") : "--");
        } else {
            snprintf(text, sizeof(text), "RSSI:-- Bat:%s", (app->raw_file_tpms_have & TPMS_HAS_BATTERY) ? ((app->raw_file_tpms_have & TPMS_BATTERY_LOW) ? "LOW" : "OK") : "--");
        }
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 60, text);
    } else {
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 36, app->raw_message[0] ? app->raw_message : "No TPMS data");
    }
}

static void tpms_view_draw_raw_confirm_delete(Canvas* canvas, TpmsBridgeApp* app) {
    tpms_view_draw_raw_title(canvas, "Delete RAW?");
    canvas_set_font(canvas, FontSecondary);

    canvas_draw_str(canvas, TPMS_LIST_LEFT, 27, app->raw_file_names[app->raw_file_selected]);
    canvas_draw_str(canvas, TPMS_LIST_LEFT, 42, "OK deletes file");

    canvas_draw_str(canvas, TPMS_LIST_LEFT, 62, "OK:delete");
    canvas_draw_str_aligned(canvas, 126, 62, AlignRight, AlignBottom, "Back");
}
static void tpms_view_draw_raw_confirm_delete_all(Canvas* canvas) {
    tpms_view_draw_raw_title(canvas, "Delete all RAW?");
    canvas_set_font(canvas, FontSecondary);

    canvas_draw_str(canvas, TPMS_LIST_LEFT, 30, "All .sub files");
    canvas_draw_str(canvas, TPMS_LIST_LEFT, 42, "OK deletes all");

    canvas_draw_str(canvas, TPMS_LIST_LEFT, 62, "OK:delete");
    canvas_draw_str_aligned(canvas, 126, 62, AlignRight, AlignBottom, "Back");
}

static void tpms_view_draw_raw_message(Canvas* canvas, TpmsBridgeApp* app) {
    tpms_view_draw_raw_title(canvas, "RAW");
    canvas_set_font(canvas, FontSecondary);

    canvas_draw_str_aligned(canvas, 64, 34, AlignCenter, AlignBottom, app->raw_message);
    canvas_draw_str_aligned(canvas, 126, 62, AlignRight, AlignBottom, "Back");
}

static void tpms_view_draw_raw_tx(Canvas* canvas, TpmsBridgeApp* app) {
    tpms_view_draw_raw_title(canvas, "RAW play");
    canvas_set_font(canvas, FontSecondary);

    const bool tx_visible =
        app->raw_tx_active || app->raw_tx_finishing ||
        (app->raw_tx_display_until_tick != 0 &&
         (int32_t)(furi_get_tick() - app->raw_tx_display_until_tick) < 0);
    const char* status = tx_visible ? (app->tx_device_external ? "TX EXT" : "TX INT") : "TX OFF";
    canvas_draw_str_aligned(canvas, 64, 34, AlignCenter, AlignBottom, status);
    if(!app->raw_tx_active && !app->raw_tx_finishing) {
        canvas_draw_str_aligned(canvas, 64, 46, AlignCenter, AlignBottom, "Press OK");
    }
    canvas_draw_str(canvas, TPMS_LIST_LEFT, 62, "Hold OK");
    canvas_draw_str_aligned(canvas, 126, 62, AlignRight, AlignBottom, "Back");
}

static void tpms_view_draw_raw_help(Canvas* canvas, TpmsBridgeApp* app) {
    char title[TPMS_TEXT_MAX];
    snprintf(title, sizeof(title), "Help %u/5", (unsigned)(app->raw_help_page + 1));
    tpms_view_draw_raw_title(canvas, title);
    canvas_set_font(canvas, FontSecondary);

    switch(app->raw_help_page) {
    case 0:
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 22, "Main screen:");
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 32, "OK: RX on/off");
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 42, "Hold OK: light");
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 52, "R: RAW files");
        break;
    case 1:
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 22, "RAW save:");
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 32, "Hold L: ON/OFF");
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 42, "L: wake pulse");
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 52, "Save after decode");
        break;
    case 2:
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 22, "Files:");
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 32, "OK: file actions");
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 42, "Open shows TPMS");
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 52, "Delete asks first");
        break;
    case 3:
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 22, "Important:");
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 32, "Play uses Sub-GHz");
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 42, "Rename: type name");
        canvas_draw_str(canvas, TPMS_LIST_LEFT, 52, "Delete asks first");
        break;
    default:
        canvas_draw_xbm(
            canvas,
            6,
            16,
            LASZLO_PORTRAIT_40_WIDTH,
            LASZLO_PORTRAIT_40_HEIGHT,
            laszlo_portrait_40);
        canvas_draw_str(canvas, 55, 32, TPMS_APP_TITLE);
        canvas_draw_str(canvas, 55, 42, TPMS_APP_VERSION);
        canvas_draw_str(canvas, 55, 52, "By Laszlo");
        break;
    }

    canvas_draw_str(canvas, TPMS_LIST_LEFT, 62, "Up/Down");
    canvas_draw_str_aligned(canvas, 126, 62, AlignRight, AlignBottom, "Back");
}

void tpms_view_draw(Canvas* canvas, TpmsBridgeApp* app) {
    canvas_clear(canvas);
    canvas_set_color(canvas, ColorBlack);

    if(app->screen == TpmsScreenRawFiles) {
        tpms_view_draw_raw_files(canvas, app);
    } else if(app->screen == TpmsScreenRawMenu && app->raw_file_count > 0) {
        tpms_view_draw_raw_menu(canvas, app);
    } else if(app->screen == TpmsScreenRawOpen && app->raw_file_count > 0) {
        tpms_view_draw_raw_open(canvas, app);
    } else if(app->screen == TpmsScreenRawInfo && app->raw_file_count > 0) {
        tpms_view_draw_raw_info(canvas, app);
    } else if(app->screen == TpmsScreenRawConfirmDelete && app->raw_file_count > 0) {
        tpms_view_draw_raw_confirm_delete(canvas, app);
    } else if(app->screen == TpmsScreenRawConfirmDeleteAll && app->raw_file_count > 0) {
        tpms_view_draw_raw_confirm_delete_all(canvas);
    } else if(app->screen == TpmsScreenRawMessage) {
        tpms_view_draw_raw_message(canvas, app);
    } else if(app->screen == TpmsScreenRawTx) {
        tpms_view_draw_raw_tx(canvas, app);
    } else if(app->screen == TpmsScreenRawHelp) {
        tpms_view_draw_raw_help(canvas, app);
    } else if(app->screen == TpmsScreenDetail && app->selected < app->store.count) {
        tpms_view_draw_detail(canvas, app);
    } else {
        tpms_view_draw_list(canvas, app);
    }
}

void tpms_view_follow_selection(TpmsBridgeApp* app) {
    if(app->store.count == 0) {
        app->selected = 0;
        app->scroll = 0;
        return;
    }

    if(app->selected >= app->store.count) app->selected = app->store.count - 1;

    if(app->selected < app->scroll) {
        app->scroll = app->selected;
    } else if(app->selected >= app->scroll + TPMS_VIEW_ROWS) {
        app->scroll = app->selected - TPMS_VIEW_ROWS + 1;
    }

    if(app->store.count <= TPMS_VIEW_ROWS) app->scroll = 0;
}
