# omi-bridge-firmware

ESP32-S3 firmware that acts as a **BLE → WiFi bridge** for the Omi CV1 pendant. It connects to the
pendant over BLE (NimBLE central), reassembles the live Opus audio stream, and uploads it in batches
to the ingest API of [omi-monologue-pipeline](https://github.com/inxaos-repo/omi-monologue-pipeline)
(`omi-pipeline --serve`, `POST /v1/frames`). The bridge never decodes audio.

> Status: **compiles in CI and the protocol code has host unit tests. Not yet run on hardware or
> against a real pendant.** See "Verified vs untested" below.

## What it does

1. Scans for the pendant: advertised name prefix `Omi` or the audio service `19B10000-…`, optionally
   restricted to one MAC. Connects with a 7.5–15 ms connection interval and asks for MTU 247, DLE 251
   and 2M PHY.
2. Discovers GATT and reads **Device Information firmware revision (0x2A26)**. 3.0.21+ means the ring
   storage protocol and ≤3.0.19 means the file protocol. It then reads the codec id (`19B10002`),
   which must be 20 or 21 (Opus).
3. Optionally writes the SNTP epoch to the pendant RTC (`19B10031`, u32 LE).
4. Subscribes to live audio (`19B10001`) and reassembles frames exactly like the pipeline's
   `FrameAssembler`: a new frame starts at fragment index 0, and a packet-id gap drops the partial frame.
5. Batches about 10 s of frames as `lp16` (`[len u16 LE][frame]…`) in PSRAM and sends
   `POST {ingest}/v1/frames?device=<id>&start=<epoch>&format=lp16&codec=21` with
   `Authorization: Bearer <token>`.
6. **Never loses a batch on a WiFi or ingest outage:**
   - Batches wait in a PSRAM queue.
   - Anything not uploaded within `OMI_SPILL_AFTER_S` (30 s), or above `OMI_RAM_QUEUE_KB`, is written
     atomically (tmp → fsync → rename) to a LittleFS spool on the `spool` flash partition (~4 MB ≈
     17 min of audio).
   - Uploads retry with exponential backoff (1 s → 30 s) and drain the oldest data (flash) first.
   - 400/413/415 responses go to `/spool/bad/` and are not retried forever.
7. **Storage sync (fw 3.0.21 ring) is OFF by default.** Reading the ring frees pendant storage as it is
   sent. Turn it on with `OMI_STORAGE_SYNC` or NVS `storage_sync=1`. When enabled, the bridge:
   - issues bounded `RING_READ`s only when the spool has room for the whole chunk,
   - strips the 4-byte timestamp from each 444-byte packet and writes the 440-byte blocks to the
     flash spool,
   - fsyncs **before** sending `RING_ADVANCE`,
   - uploads the blocks later as `format=omi-sd`.

   **`RING_CLEAR` (0x13) and the 3.0.19 `DELETE_FILE` are never sent.** No builder exists for either,
   and every storage write goes through `omi_storage_write_allowed()` (unit-tested). The 3.0.19 file
   protocol is not implemented; those pendants get live streaming only.

   Caveat: the 3.0.21 firmware checkpoints on tx-complete, not on our ADVANCE. Data that has been sent
   but not yet persisted (≤ 32 packets ≈ 3.5 s, or more if the BLE→task ring buffer overflows) is lost
   if the bridge crashes mid-read.

## Hardware

- Any **ESP32-S3 with octal PSRAM and 8 MB+ flash**. Reference: Espressif **ESP32-S3-DevKitC-1-N8R8**,
  or Seeed **XIAO ESP32S3** (8 MB flash / 8 MB PSRAM, with external antenna).
- It boots without PSRAM (`CONFIG_SPIRAM_IGNORE_NOTFOUND`), but then the RAM queue is tiny.
- USB 5 V power. Keep it within a few metres of where the pendant is worn or charged.
- Health LED: set `OMI_LED_GPIO` for a plain LED. It is not a WS2812 driver, so the DevKitC RGB LED
  on GPIO48 is not supported. Patterns:
  - blip every 2 s: no WiFi
  - slow blink: WiFi up, no pendant
  - solid: streaming
  - fast blink: upload backlog or failures
- Logs on the USB serial console include a `health:` line every 60 s with queue, spool, posts and
  losses.

## Build

```bash
# ESP-IDF v5.3.x
idf.py set-target esp32s3
idf.py menuconfig        # "Omi bridge" menu (optional, see Config)
idf.py build
# or in docker, exactly like CI:
docker run --rm -v $PWD:/project -w /project espressif/idf:v5.3.2 \
  bash -c '. $IDF_PATH/export.sh && idf.py set-target esp32s3 build'
```

Host unit tests for the protocol code (`components/omi_proto`) run without ESP-IDF:

```bash
make -C test/host test
```

## Config (no secrets in git)

`sdkconfig.defaults` is a secret-free template. You can set values in two ways:

- **menuconfig → Omi bridge.** This writes to the local `sdkconfig` (git-ignored). Fine for a desk test.
- **NVS (preferred).** Copy `nvs_config.csv.example` to `nvs_config.csv` (git-ignored), fill it in, and
  flash it to the `nvs` partition. NVS values override Kconfig:

```bash
python $IDF_PATH/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py \
  generate nvs_config.csv nvs.bin 0x6000
esptool.py --chip esp32s3 write_flash 0x9000 nvs.bin
```

| Key (NVS ns `omi`) | Kconfig | Meaning |
|---|---|---|
| `wifi_ssid`, `wifi_pass` | `OMI_WIFI_SSID/PASSWORD` | IoT VLAN WiFi |
| `ingest_url` | `OMI_INGEST_URL` | e.g. `http://192.168.2.X:8080` (plain HTTP LB IP, see pipeline `docs/ingest.md`) |
| `token` | `OMI_INGEST_TOKEN` | `OMI_INGEST_TOKEN` bearer |
| `device_id` | `OMI_DEVICE_ID` | `?device=`. A `test…` prefix routes to the TEST wiki prefix |
| `pendant_mac` | `OMI_PENDANT_MAC` | `aa:bb:cc:dd:ee:ff`, empty = first Omi seen |
| `ntp` | `OMI_NTP_SERVER` | SNTP server (use the gateway if the IoT VLAN has no internet) |
| `storage_sync` (u8) | `OMI_STORAGE_SYNC` | 0 (default) / 1. **1 consumes pendant data** |

Batch length, queue limits, spill timeout, ring chunk size, LED GPIO and the time sync toggle are
Kconfig-only.

## Flash

```bash
idf.py -p /dev/ttyACM0 flash monitor
```

The partition table (`partitions.csv`, 8 MB) has two OTA app slots (`ota_0`/`ota_1`), so OTA can be
added later without repartitioning. No OTA client is implemented yet.

## Layout

```
components/omi_proto/   pure C protocol: frame assembler, lp16, stored-frame iterator, ring cmds/notify,
                        ring packet assembler, fw-version -> protocol, storage write guard
main/ble_central.c      NimBLE scan/connect/discover/subscribe step machine
main/uplink.c           live batching, PSRAM queue, spill, HTTP POST + retry
main/spool.c            LittleFS spool (atomic records, .prt recovery, quarantine)
main/ring_sync.c        3.0.21 ring storage sync (off by default)
main/net.c, config.c, led.c, app_main.c
test/host/              host unit tests (vectors from omi-monologue-pipeline tests/test_omi_protocol.py)
```

## Verified vs untested

Verified (CI):
- `idf.py build` for esp32s3 with `espressif/idf:v5.3.2`.
- Host unit tests (ASan/UBSan) for every function in `omi_proto`, using the same vectors as the Python
  reference.

Untested (needs hardware, tracked in issues):
- Scanning and connecting to a real CV1. The advertised name and UUIDs in the advertisement are
  assumed.
- MTU/2M PHY negotiation and real throughput with WiFi coexistence.
- The end-to-end POST to a live ingest.
- LittleFS behaviour on a full spool, and power-loss recovery.
- The ring sync against a real 3.0.21 pendant.
- Board choice, pinout and LED.
- Pendant battery drain under continuous streaming.
