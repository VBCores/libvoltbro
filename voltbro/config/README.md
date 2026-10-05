# Common configuration

`base_config.h` defines the C-compatible, 28-byte `BaseConfigData` format used
by applications and bootloaders. Its type ID describes only the common schema.
An application stores its own type ID in its own configuration struct.

`ConfigManager<AppConfig, Storage>` owns a packed `{base, app}` record and a
CONFIG snapshot. `AppConfig` must be trivially copyable, have `TYPE_ID` and
`type_id`, initialize its defaults, and implement
`bool are_required_params_set(const BaseConfigData&) const`.
`Storage` supplies `read<T>(T*, address)` and `write<T>(T*, address)` with zero
indicating success. The constructor accepts `uint32_t storage_address = 0`;
all reads and writes use that byte address. One write saves the combined record.
The storage backend owns address interpretation and erase/program operations.
The passed common defaults object must outlive the manager.

- `load()` independently validates/resets the two schemas; `save()` persists
  changed editable data. Callers handle an I/O failure explicitly.
- `begin()` snapshots committed data. `get_config()` exposes editable data;
  `get_committed_config()` isolates transport writes from pending Serial edits.
- After an editable write, call `mark_changed()`. `commit()` saves and exits;
  `discard()` restores the latest committed data. `reset()` stages defaults.
- After a committed transport write, call `mark_committed_changed()` and
  service `save_pending()` in thread mode. It never saves Serial staging.

The manager contains no transport formatting, HAL calls, motor actions, or
dynamic allocation. A changed application schema does not reset a valid common
block. Storage placement of other records, such as motor calibration, belongs
to the application and must be checked independently.

VBDrive and VBBoot select the address with `-DVB_CONFIG_ADDRESS=0` (decimal or
hexadecimal). VBDrive validates the complete record against its EEPROM capacity
and fixed calibration/encoder regions. The common and application schema IDs do
not depend on the storage address. Changing the address selects another record;
it does not copy data.
