# CLAUDE.md: omi-bridge-firmware

The ESP32-S3 NimBLE central that bridges the Omi CV1 pendant to omi-monologue-pipeline's ingest.
README.md has the user-facing details.

## Ground rules
- **Spec source:** `inxaos-repo/omi-monologue-pipeline` `docs/omi-ble-protocol.md`, `docs/ingest.md`,
  `src/omi_pipeline/omi_protocol.py`, and `tests/test_omi_protocol.py`, which holds the vectors.
  When the protocol changes there, mirror it in `components/omi_proto` and `test/host`.
- **Never send pendant-wiping commands.** No builders for `RING_CLEAR` (0x13) or 3.0.19
  `DELETE_FILE`. All writes to the storage characteristic go through `ble_storage_write()` →
  `omi_storage_write_allowed()`. Keep the guard test (`test_guard_never_wipes`) passing.
- **This is the ONLY bridge firmware repo** (boss decision 2026-11-12). A duplicate once lived in
  omi-monologue-pipeline `firmware/esp32s3-bridge`; it was merged here and removed there.
- **3.0.24 custody (primary target):** `ADVANCE_ID` only after the ingest returned 2xx for exactly the
  packets received. Never spill custody data to flash or ack on a flash write. Live marks are acked
  only after the backlog is drained and live audio through the mark index was ingested 2xx with no
  spill/drop/reject in that connection.
- **3.0.21 legacy storage sync defaults to OFF.** Reading the 3.0.21 ring consumes pendant data. Always
  persist to the flash spool and fsync **before** `RING_ADVANCE`.
- **SuperMini (4 MB / 2 MB quad PSRAM) is the default board** (`sdkconfig.defaults`). XIAO is the
  `sdkconfig.board.xiao` overlay. CI fails if the app exceeds 95% of the smallest OTA slot.
- **No secrets in git.** `sdkconfig`, `nvs_config.csv` and `*.nvs.bin` are git-ignored. Only
  `sdkconfig.defaults` and `nvs_config.csv.example` are committed.
- Keep `components/omi_proto` free of ESP-IDF includes so it builds on the host.
- Don't flash devices or connect to the real pendant from automation.

## Workflow
- Branch `bob/*` → PR → CI green (`host-tests` and the `build-esp32s3` matrix: supermini (default) + xiao). Bob created this repo, so Bob may
  self-merge after green CI.
- Local checks:
  - `make -C test/host test`
  - `docker run --rm -v $PWD:/project -w /project espressif/idf:v5.3.2 bash -c '. $IDF_PATH/export.sh && idf.py set-target esp32s3 build'`
- The IDF version is pinned in `.github/workflows/ci.yml` and `main/idf_component.yml` (`idf >=5.3`).
  LittleFS comes from the component manager (`joltwallet/littlefs`).

## Architecture notes
- NimBLE host task: GAP/GATT callbacks only. No flash I/O, no blocking. Audio frames go into
  `uplink_live_frame()`, a mutex-protected memcpy into the PSRAM batch. Storage notifications go into
  a PSRAM FreeRTOS ring buffer that `ring_sync` consumes.
- Uplink task (core 1): closes idle batches, spills to flash, POSTs the flash spool first (oldest),
  then the RAM queue, with backoff.
- `CONFIG_BT_NIMBLE_EXT_ADV=n` keeps legacy `BLE_GAP_EVENT_DISC` scanning. 50-feature support stays
  on for the 2M PHY.
