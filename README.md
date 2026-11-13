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
     17 min of audio on 8 MB boards, 1.25 MB ≈ 5 min on the 4 MB SuperMini).
   - Uploads retry with exponential backoff (1 s → 30 s) and drain the oldest data (flash) first.
   - 400/413/415 responses go to `/spool/bad/` and are not retried forever.
7. **Backlog custody sync (fw 3.0.24) is ON.** Protocol spec (shared with the receiver):
   [omi-monologue-pipeline `docs/omi-ble-protocol.md`](https://github.com/inxaos-repo/omi-monologue-pipeline/blob/main/docs/omi-ble-protocol.md).
   If the pendant's `RING_INFO` carries a ring id, the bridge sends `CUSTODY_ENABLE` (0x14, caps
   app-ack-reclaim | live-persist | idempotent-advance | ring-id). Then it loops: `RING_INFO`
   (backlog end = write_seq at session start) → `RING_READ(read_seq, n)` into a PSRAM batch
   (`min(128 KB, PSRAM/16)`; 128 KB ≈ 290 packets on both boards) → `POST format=omi-sd` →
   **`ADVANCE_ID` (0x15, ring id + seq) only after the ingest answers 2xx**. The target never goes past
   the packets actually received. Custody data is never spilled to flash and never acked on a flash
   write: a failed POST, a lost notification or a WiFi outage just means the pendant keeps the data and
   it is re-read later (duplicates are possible, loss is not). ADVANCE status 11 (ring id mismatch)
   resets the session.

   Live persistence: the pendant also writes live audio to its ring and sends `LIVE_MARK`
   (ring id, seq, live packet index). Once the backlog is drained, the bridge acks a mark with
   `ADVANCE_ID` when live audio through that packet index has been ingested (2xx). If any live batch
   of the connection is spilled to flash, dropped or rejected, live marks are not acked for the rest
   of that connection (the pendant re-sends that span as backlog next time).

8. **Legacy storage sync (fw 3.0.21 ring) is OFF by default.** Reading the ring frees pendant storage as it is
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

Two board targets, both built in CI:

| Board | Flash / PSRAM | Config | LED |
|---|---|---|---|
| Aideepen **ESP32-S3 Mini / SuperMini** (ESP32-S3FH4R2, S3-Zero style) — **default** | 4 MB / 2 MB quad | `sdkconfig.defaults`, `partitions_4mb.csv` | WS2812 on GPIO21 |
| Seeed **XIAO ESP32S3** (also ESP32-S3-DevKitC-1-N8R8) | 8 MB / 8 MB octal | `+ sdkconfig.board.xiao`, `partitions.csv` | none by default (`OMI_LED_GPIO` for a plain LED) |

- Buffer sizes adapt to the PSRAM detected at boot (`components/omi_sizing`, host-tested): the RAM
  queue is `min(OMI_RAM_QUEUE_KB, 35% of PSRAM)` before spilling to flash, hard cap 2x that (1.5x below
  4 MB PSRAM), the ring-sync buffer is `min(256 KB, PSRAM/8)` and the custody batch `min(128 KB, PSRAM/16)`.
  With 8 MB: 2 MB/4 MB/256 KB/128 KB. With 2 MB: ~717 KB/~1.07 MB/256 KB/128 KB (~3 min of audio in RAM,
  ~0.5 MB PSRAM left for live batches, spool reads and the stacks). Logged at boot.
- It boots without PSRAM (`CONFIG_SPIRAM_IGNORE_NOTFOUND`), but then the RAM queue is 64 KB.
- USB 5 V power. Keep it within a few metres of where the pendant is worn or charged.
- Logs on the USB serial console include a `health:` line every 60 s with queue, spool, posts and
  losses.

### Status LED

WS2812 (SuperMini, `OMI_LED_WS2812`, brightness `OMI_LED_BRIGHTNESS`=16/255):

- **blue**: WiFi up, scanning for / connecting to the pendant
- **green**: pendant streaming, uploads OK
- **amber**: buffering in PSRAM (ingest unreachable or slow)
- **red**: data in the flash spool, or no WiFi (error)

If red and green look swapped, enable `OMI_LED_WS2812_RGB_ORDER`.

Plain LED (`OMI_LED_GPIO` without WS2812): blip every 2 s = no WiFi, slow blink = scanning, solid =
streaming, fast blink = buffering/spooling.

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
| `ingest_url` | `OMI_INGEST_URL` | `http://192.168.2.172:8080` (plain-HTTP MetalLB LB `omi-ingest-lan`; see pipeline `docs/ingest.md`) |
| `token` | `OMI_INGEST_TOKEN` | `OMI_INGEST_TOKEN` bearer |
| `device_id` | `OMI_DEVICE_ID` | `?device=`. A `test…` prefix routes to the TEST wiki prefix |
| `pendant_mac` | `OMI_PENDANT_MAC` | `aa:bb:cc:dd:ee:ff`, empty = first Omi seen |
| `ntp` | `OMI_NTP_SERVER` | SNTP server (use the gateway if the IoT VLAN has no internet) |
| `storage_sync` (u8) | `OMI_STORAGE_SYNC` | 0 (default) / 1. 3.0.21 legacy only. **1 consumes 3.0.21 pendant data**. 3.0.24 custody does not need it |

Batch length, queue limits, spill timeout, ring chunk size, LED GPIO/type and the time sync toggle are
Kconfig-only (or per-board in `sdkconfig.board.*`).

## Flash

XIAO (alternate board):

```bash
B=build-xiao
idf.py -B $B -D SDKCONFIG=$B/sdkconfig \
  -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.board.xiao" set-target esp32s3 build
idf.py -B $B -p /dev/ttyACM0 flash monitor
```

### ESP32-S3 SuperMini (4 MB / 2 MB, default)

1. Build (plain defaults; or download the `omi-bridge-esp32s3-supermini` CI artifact):

   ```bash
   B=build-supermini
   idf.py -B $B -D SDKCONFIG=$B/sdkconfig set-target esp32s3 build
   ```

2. Make the config. Copy `nvs_config.csv.example` to `nvs_config.csv` and set:
   - `wifi_ssid`, `wifi_pass`: IoT VLAN WiFi
   - `ingest_url`: `http://192.168.2.172:8080`
   - `token`: the `OMI_INGEST_TOKEN` value
   - `device_id`: `omi-test-bridge` (the `test` prefix routes to the TEST wiki prefix)
   - `storage_sync`: `0` (**OFF for the first test**; 1 consumes pendant data)

   ```bash
   python $IDF_PATH/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py \
     generate nvs_config.csv nvs.bin 0x6000
   ```

3. Plug in over USB-C (native USB, shows up as `/dev/ttyACM0`). A blank or freshly bought board
   usually enumerates right away. If it doesn't, or the flash fails to connect: hold **BOOT**,
   tap **RESET** (or plug in while holding BOOT), then release BOOT.

4. Flash app and config, then watch the console (USB-Serial-JTAG):

   ```bash
   idf.py -B $B -p /dev/ttyACM0 flash
   esptool.py --chip esp32s3 -p /dev/ttyACM0 write_flash 0x9000 nvs.bin
   idf.py -B $B -p /dev/ttyACM0 monitor
   ```

   The boot log should say `board s3-supermini, PSRAM 2048 KB`. If it says `PSRAM 0 KB`, the PSRAM
   mode is wrong for your chip. The LED should go red → blue → green once the pendant streams.

The SuperMini table (`partitions_4mb.csv`) has two 1.31 MB OTA slots and a **1.25 MB spool ≈ 5 min of
audio** (vs ~17 min on 8 MB boards).

The partition table (`partitions.csv`, 8 MB) has two OTA app slots (`ota_0`/`ota_1`), so OTA can be
added later without repartitioning. No OTA client is implemented yet.

## Layout

```
components/omi_proto/   pure C protocol: frame assembler, lp16, stored-frame iterator, ring cmds/notify,
                        ring packet assembler, fw-version -> protocol, storage write guard
main/ble_central.c      NimBLE scan/connect/discover/subscribe step machine
main/uplink.c           live batching, PSRAM queue, spill, HTTP POST + retry
main/spool.c            LittleFS spool (atomic records, .prt recovery, quarantine)
main/ring_sync.c        3.0.24 custody backlog/live-mark sync; 3.0.21 legacy ring sync (off by default)
components/omi_sizing/  pure C PSRAM-based buffer sizing (host-tested)
main/led.c              WS2812 / plain / no-op health LED
main/net.c, config.c, app_main.c
test/host/              host unit tests (vectors from omi-monologue-pipeline tests/test_omi_protocol.py)
```

## Verified vs untested

Verified (CI):
- `idf.py build` for esp32s3 (XIAO and SuperMini) with `espressif/idf:v5.3.2`.
- Host unit tests (ASan/UBSan) for every function in `omi_proto`, using the same vectors as the Python
  reference.

Untested (needs hardware, tracked in issues):
- Scanning and connecting to a real CV1. The advertised name and UUIDs in the advertisement are
  assumed.
- MTU/2M PHY negotiation and real throughput with WiFi coexistence.
- The end-to-end POST to a live ingest.
- LittleFS behaviour on a full spool, and power-loss recovery.
- The ring sync against a real 3.0.21 pendant.
- Both boards (nothing has run on hardware), SuperMini quad-PSRAM detection, WS2812 colour order.
- Pendant battery drain under continuous streaming.
