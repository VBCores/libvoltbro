#pragma once

#include <stddef.h>
#include <stdint.h>

#define VOLTBRO_BASE_CONFIG_TYPE_ID UINT32_C(0x01234567)
#define VOLTBRO_DEFAULT_SERIAL_BAUD UINT32_C(115200)
#define VOLTBRO_DEFAULT_CAN_NOMINAL_BAUD 4U
#define VOLTBRO_DEFAULT_CAN_DATA_BAUD 3U

/* Stable EEPROM prefix shared by C bootloaders and C++ applications. */
typedef struct __attribute__((packed)) BaseConfigData {
    uint32_t type_id;
    uint32_t serial_baud;
    uint8_t node_id;
    uint8_t fdcan_nominal_baud;
    uint8_t fdcan_data_baud;
    uint8_t was_configured;
    char name[16];
} BaseConfigData;

#ifdef __cplusplus
static_assert(sizeof(BaseConfigData) == 28);
static_assert(offsetof(BaseConfigData, serial_baud) == 4);
static_assert(offsetof(BaseConfigData, node_id) == 8);
static_assert(offsetof(BaseConfigData, name) == 12);
#else
_Static_assert(sizeof(BaseConfigData) == 28, "BaseConfigData ABI");
_Static_assert(offsetof(BaseConfigData, serial_baud) == 4, "Serial baud ABI");
_Static_assert(offsetof(BaseConfigData, node_id) == 8, "Node ID ABI");
_Static_assert(offsetof(BaseConfigData, name) == 12, "Name ABI");
#endif

static inline int voltbro_serial_baud_valid(uint32_t baud) {
    switch (baud) {
        case 9600: case 19200: case 38400: case 57600: case 115200:
        case 230400: case 460800: case 921600: case 1000000: return 1;
        default: return 0;
    }
}

/* Node zero is unassigned; applications decide whether a CAN node is required. */
static inline int voltbro_base_config_valid(const BaseConfigData* config) {
    return config->type_id == VOLTBRO_BASE_CONFIG_TYPE_ID && config->node_id <= 127 &&
        config->fdcan_nominal_baud <= 4 && config->fdcan_data_baud <= 3 &&
        voltbro_serial_baud_valid(config->serial_baud) && config->name[15] == '\0';
}

static inline uint8_t voltbro_can_nominal_prescaler(uint8_t baud) {
    return baud <= 4 ? (uint8_t)(16U >> baud) : 0;
}

static inline uint8_t voltbro_can_data_prescaler(uint8_t baud) {
    return baud <= 3 ? (uint8_t)(8U >> baud) : 0;
}
