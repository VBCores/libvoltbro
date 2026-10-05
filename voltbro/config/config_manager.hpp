#pragma once

#include "base_config.h"
#include <cstdint>
#include <type_traits>

/** Two independent schemas stored together; Storage read/write return zero on success.
 * The application owns validation of required fields and runtime side effects.
 * CONFIG edits and committed transport writes share no mutable staging state.
 */
template<class AppConfig, class Storage>
class ConfigManager {
public:
    struct __attribute__((packed)) Data {
        BaseConfigData base;
        AppConfig app;

        bool are_required_params_set() const {
            return voltbro_base_config_valid(&base) && app.are_required_params_set(base);
        }
    };
    static_assert(std::is_trivially_copyable_v<Data>);

private:
    Storage& storage;
    const BaseConfigData& defaults;
    const uint32_t storage_address;
    Data data;
    Data committed_before_config{};
    bool editing = false;
    bool dirty = false;
    bool committed_dirty = false;

    bool persist(Data& value) {
        value.base.was_configured = value.are_required_params_set();
        return storage.template write<Data>(&value, storage_address) == 0;
    }

public:
    ConfigManager(Storage& storage, const BaseConfigData& defaults, uint32_t storage_address = 0)
        : storage(storage), defaults(defaults), storage_address(storage_address), data{defaults, AppConfig{}} {}

    Data& get_config() { return data; }
    const Data& get_config() const { return data; }
    Data& get_committed_config() { return editing ? committed_before_config : data; }
    const Data& get_committed_config() const { return editing ? committed_before_config : data; }
    bool is_editing() const { return editing; }
    bool needs_save() const { return dirty; }

    bool load() {
        if (storage.template read<Data>(&data, storage_address) != 0) return false;
        editing = dirty = committed_dirty = false;
        if (!voltbro_base_config_valid(&data.base)) {
            data.base = defaults;
            dirty = true;
        }
        if (data.app.type_id != AppConfig::TYPE_ID) {
            data.app = AppConfig{};
            dirty = true;
        }
        return true;
    }

    void begin() {
        if (editing) return;
        committed_before_config = data;
        committed_dirty |= dirty;
        dirty = false;
        editing = true;
    }

    void discard() {
        if (!editing) return;
        data = committed_before_config;
        editing = false;
        dirty = false;
    }

    void reset() {
        data = Data{defaults, AppConfig{}};
        dirty = true;
    }

    void mark_changed() {
        data.base.was_configured = data.are_required_params_set();
        dirty = true;
    }

    void mark_committed_changed() {
        auto& value = get_committed_config();
        value.base.was_configured = value.are_required_params_set();
        committed_dirty = true;
    }

    bool save() {
        if (dirty && !persist(data)) return false;
        dirty = false;
        return true;
    }

    bool commit() {
        if (!save()) return false;
        editing = false;
        committed_dirty = false;
        return true;
    }

    bool save_pending() {
        if (committed_dirty && !persist(get_committed_config())) return false;
        committed_dirty = false;
        return true;
    }
};
