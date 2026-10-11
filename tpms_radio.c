#include "tpms_radio.h"

const uint32_t tpms_frequencies[TPMS_FREQUENCY_COUNT] = {
    433920000UL, /* Europe */
    315000000UL, /* North America, Japan */
};

uint32_t tpms_slot_frequency(uint8_t slot) {
    return tpms_frequencies[(slot / 2) % TPMS_FREQUENCY_COUNT];
}

TpmsModulation tpms_slot_modulation(uint8_t slot) {
    return (slot & 1) ? TpmsModulationOok : TpmsModulationFsk;
}

bool tpms_config_is_auto(TpmsConfig config) {
    return config == TpmsConfigAuto433 || config == TpmsConfigAuto315;
}

uint8_t tpms_config_first_slot(TpmsConfig config) {
    switch(config) {
    case TpmsConfigAuto315:
        return 2;
    case TpmsConfig315Fsk:
    case TpmsConfig315Ook:
        return 2;
    default:
        return 0;
    }
}

uint8_t tpms_config_slot_count(TpmsConfig config) {
    return tpms_config_is_auto(config) ? 2 : 1;
}
