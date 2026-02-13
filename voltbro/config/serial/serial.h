#pragma once
#if defined(STM32G4) || defined(STM32_G)
#include "stm32g4xx_hal.h"
#if defined(HAL_UART_MODULE_ENABLED)

#include "usart.h"
#include <cyphal/cyphal.h>
#include <voltbro/eeprom/eeprom.hpp>

#include <nanoprintf.h>
#include <cstdint>
#include <string>
#include <memory>
#include <cstring>

static constexpr size_t MAX_SIZE = 512;
extern char buffer[MAX_SIZE];

// TODO: check that buffer is not used by 2 accumulators at the same time
// NOTE: This is a simple implementation, it can be much improved
class UARTResponseAccumulator {
private:
    size_t pos;
    UART_HandleTypeDef* huart;
public:
    UARTResponseAccumulator(UART_HandleTypeDef* huart): pos(0), huart(huart) {}

    ~UARTResponseAccumulator() {
        if (pos > 0) {
            HAL_UART_Transmit_DMA(huart, reinterpret_cast<uint8_t*>(buffer), pos);
        }
    }

    void append(const char* fmt, ...) {
        if (pos >= MAX_SIZE) return;
        if (pos > 0) {
            // Overwrite the last \0
            pos--;
        }

        va_list args;
        va_start(args, fmt);
        int written = npf_vsnprintf(buffer + pos, MAX_SIZE - pos, fmt, args);
        va_end(args);

        if (written > 0) {
            pos += written;
        }
    }
};

bool safe_stoi(const std::string& str, int& out_val);
bool safe_stof(const std::string& str, float& out_val);

enum class FDCANNominalBaud : uint8_t {
    KHz62,
    KHz125,
    KHz250,
    KHz500,
    KHz1000
};

enum class FDCANDataBaud : uint8_t {
    KHz1000,
    KHz2000,
    KHz4000,
    KHz8000
};

enum class AppState {
    INIT,
    RUNNING,
    CONFIG
};

static constexpr std::string NODE_ID_PARAM = "node_id";
static constexpr std::string FDCAN_DATA_PARAM = "data_baud";
static constexpr std::string FDCAN_NOMINAL_PARAM = "nominal_baud";

struct BaseConfigData {
    static constexpr uint32_t TYPE_ID = 0x12345678;
    uint32_t type_id = BaseConfigData::TYPE_ID;
    bool was_configured = false;

    CanardNodeID node_id = 11;
    FDCANNominalBaud fdcan_nominal_baud = FDCANNominalBaud::KHz1000;
    FDCANDataBaud fdcan_data_baud = FDCANDataBaud::KHz8000;
};

bool get_base_params(BaseConfigData* data, const std::string& param, UARTResponseAccumulator& responses);

bool set_base_params(
    BaseConfigData* data,
    const std::string& param,
    const std::string& value,
    UARTResponseAccumulator& responses,
    bool& was_found
);

template <typename ConfigT, uint16_t config_location>
class AppConfigurator {
protected:
    static constexpr uint16_t UART_RX_BUFFER_SIZE = 32;
    const std::function<void(void)> turn_on;
    const std::function<void(void)> turn_off;
    char uart_rx_buffer[UART_RX_BUFFER_SIZE + 1];
    AppState app_state = AppState::INIT;
    ConfigT config_data;

    UART_HandleTypeDef* huart;
    EEPROM& eeprom;

    void save_config() {
        auto status = eeprom.write<ConfigT>(&config_data, config_location);
        if (status != HAL_OK) {
            Error_Handler();
        }
    }

    void load_config() {
        UARTResponseAccumulator responses(huart);

        auto status = eeprom.read<ConfigT>(&config_data, config_location);
        if (status != HAL_OK) {
            Error_Handler();
        }
        responses.append("Got config_data type_id: <0x%08lX>\r\n", config_data.type_id);
        config_data.print_self(responses);

        if (config_data.type_id != ConfigT::TYPE_ID) {
            config_data = ConfigT();
            responses.append("Saving config...\r\n");
            save_config();
        }

        if (!config_data.was_configured || !config_data.are_required_params_set()) {
            turn_off();
            responses.append("Controller was not configured!\r\n");
        }
        else {
            turn_on();
            app_state = AppState::RUNNING;
            responses.append("Controller is configured, starting\r\n");
        }
    }

    void start_uart_recv_it() {
        std::memset(uart_rx_buffer, '\0', UART_RX_BUFFER_SIZE + 1);
        /*
        / If TX uses DMA, next line is not needed,
        / if TX is IT or direct, next line is required due to bug in HAL.
        / Kept here so I don't forget
        HAL_UART_Abort_IT(&huart2);
        */
        auto status = HAL_UARTEx_ReceiveToIdle_DMA(huart, reinterpret_cast<uint8_t*>(uart_rx_buffer), UART_RX_BUFFER_SIZE);
        if (status != HAL_OK) {
            // TODO
        }
    }

    void enable_config_mode() {
        app_state = AppState::CONFIG;
        turn_off();
    }

    void disable_config_mode() {
        app_state = AppState::RUNNING;
        turn_on();
    }

    void process_command(std::string command) {
        UARTResponseAccumulator responses(huart);

        command.erase(command.find_last_not_of(" \t\n\r") + 1);
        if (command.size() == 0) {
            return;
        }

        bool do_save = false;

        if (command == "START") {
            enable_config_mode();
            responses.append("CONFIG MODE ENABLED\n\r");
        }
        else if (command == "APPLY") {
            NVIC_SystemReset();
        }
        else if (command == "RESET") {
            responses.append("Setting default config\n\r");
            config_data = ConfigT();
            responses.append("NOTE: config changes not applied! To apply, run APPLY or reset controller\n\r");
            do_save = true;
        }
        else if (command == "STOP") {
            disable_config_mode();
            responses.append("NOTE: config changes not applied! To apply, run APPLY or reset controller\n\r");
        }
        else {
            // Если не в режиме конфигурации, игнорируем команды (кроме START)
            if (app_state != AppState::CONFIG) {
                return;
            }

            // Обработка запроса параметра (формат "param_name:?")
            size_t colon_pos = command.find(':');
            if (colon_pos == std::string::npos) {
                responses.append("ERROR: Unknown command\n\r");
                return;
            }

            std::string param = command.substr(0, colon_pos);
            std::string value = command.substr(colon_pos + 1);

            if (value == "?") {
                // GET
                config_data.get(param, responses);
            }
            else {
                // SET
                do_save = config_data.set(param, value, responses);
            }
        }

        if (do_save) {
            if (config_data.are_required_params_set()) {
                config_data.was_configured = true;
                responses.append("All essential parameters set, board will start after APPLY\n\r");
            }

            save_config();
            responses.append("Saved config\n\r");
        }
    }
public:
    AppConfigurator(
        UART_HandleTypeDef* huart,
        EEPROM& eeprom,
        std::function<void(void)> turn_on,
        std::function<void(void)> turn_off
    ): turn_on(turn_on), turn_off(turn_off), huart(huart), eeprom(eeprom) {}

    ConfigT& get_config() {
        return config_data;
    }

    void init() {
        load_config();
        start_uart_recv_it();
    }

    void handle_uart_event(UART_HandleTypeDef* incoming_uart, uint16_t size) {
        if (huart != incoming_uart) {
            return; // Not the UART we are interested in
        }
        uart_rx_buffer[size] = '\0';
        auto command_string = std::string(uart_rx_buffer);
        process_command(command_string);
        start_uart_recv_it();
    }

    CanardNodeID get_node_id() const {
        return config_data.node_id;
    }
    uint8_t get_nom_prescaler() const {
        switch (config_data.fdcan_nominal_baud) {
            case FDCANNominalBaud::KHz62:
                return 16;
            case FDCANNominalBaud::KHz125:
                return 8;
            case FDCANNominalBaud::KHz250:
                return 4;
            case FDCANNominalBaud::KHz500:
                return 2;
            case FDCANNominalBaud::KHz1000:
                return 1;
        }
        // TODO, NOTE: seems that due to ConfigT being a template, compiler can't correctly track this enum
        //             but not sure - investigate later?
        __builtin_unreachable();
    }
    uint8_t get_data_prescaler()  const {
        switch (config_data.fdcan_data_baud) {
            case FDCANDataBaud::KHz1000:
                return 8;
            case FDCANDataBaud::KHz2000:
                return 4;
            case FDCANDataBaud::KHz4000:
                return 2;
            case FDCANDataBaud::KHz8000:
                return 1;
        }
        // TODO, NOTE: seems that due to ConfigT being a template, compiler can't correctly track this enum
        //             but not sure - investigate later?
        __builtin_unreachable();
    }
    bool is_app_running() const {
        return app_state == AppState::RUNNING;
    }

};

#define CHECK_AND_PRINT_PARAM_INT(field_name, param_name) \
    else if (param == param_name) { \
        responses.append("%s: %d\n\r", #field_name, field_name); \
    }

#define CHECK_AND_PRINT_PARAM_FLOAT(field_name, param_name) \
    else if (param == param_name) { \
        responses.append("%s: %f\n\r", #field_name, field_name); \
    }

#define CHECK_AND_SET_PARAM_FLOAT(field_name, param_name) \
    else if (param == param_name) { \
        field_name = new_float_value; \
        responses.append("OK: %s :%f\n\r", #field_name, field_name); \
    }

#endif
#endif
