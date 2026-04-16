# GPS Tracker V1.0 — ESP32-C3 Firmware

> A production-grade, multi-tasked GPS tracker firmware built on the **ESP32-C3** with GSM uplink, OLED display, OTA updates, and a cloud-connected backend.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Hardware](#hardware)
  - [Components](#components)
  - [Pin Wiring](#pin-wiring)
- [Features](#features)
- [Architecture](#architecture)
  - [FreeRTOS Task Map](#freertos-task-map)
  - [Module Overview](#module-overview)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Configuration](#configuration)
  - [Building & Flashing](#building--flashing)
- [Display Screens](#display-screens)
- [Button Controls](#button-controls)
- [Cloud Integration](#cloud-integration)
  - [Data Uplink (GPRS)](#data-uplink-gprs)
  - [OTA Updates (GitHub + Supabase)](#ota-updates-github--supabase)
  - [Remote Commands](#remote-commands)
- [Partition Layout](#partition-layout)
- [Dependencies](#dependencies)
- [Project Structure](#project-structure)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

**GPS Tracker V1.0** is a fully self-contained tracking device built around the **Espressif ESP32-C3** microcontroller. It continuously parses NMEA data from a **u-blox NEO-6M** GPS module, buffers packets in flash (LittleFS), and drains them to a cloud backend via a **SIM800L** GSM modem over GPRS. A **SH1107 128×128 OLED** display with a two-button UI provides on-device telemetry, and deep-sleep support extends battery life between tracking windows.

Key design goals:
- **Fire-and-forget** upload pipeline — no retry queue; prevents stale-data reupload.
- **Speed-adaptive** sampling — faster packet generation when moving, slower when idle.
- **Zero blocking** — GSM initialisation offloaded to a dedicated FreeRTOS task to keep the UI alive from first boot.
- **OTA-capable** — firmware can be updated remotely via GitHub Releases without physical access.

---

## Hardware

### Components

| Component | Part | Notes |
|-----------|------|-------|
| Microcontroller | ESP32-C3 DevKitM-1 | RISC-V, 4 MB Flash, Wi-Fi + BLE |
| GPS Module | u-blox NEO-6M | UART, 9600 baud, 5 Hz update |
| GSM Modem | SIM800L | 2G GPRS, UART, Hutch SL APN |
| Display | SH1107 128×128 OLED | I²C, 0x3C address |
| Buttons | Tactile × 2 | SELECT (BTN_A) + BACK (BTN_B) |

### Pin Wiring

```
ESP32-C3          Peripheral
──────────────────────────────────────────
GPIO 20 (TX)  ──► SIM800L RX
GPIO 21 (RX)  ◄── SIM800L TX
GPIO  0       ──► SIM800L RST / PWRKEY
GPIO 10       ◄── SIM800L Ring Indicator (RI)

GPIO  5 (TX)  ──► NEO-6M RX
GPIO  4 (RX)  ◄── NEO-6M TX

GPIO  8 (SDA) ──► SH1107 OLED SDA  (pull-up secures strapping HIGH)
GPIO  9 (SCL) ──► SH1107 OLED SCL  (pull-up secures strapping HIGH)

GPIO  3       ──► Button A (SELECT / CALL)
GPIO  1       ──► Button B (BACK / HANG)
```

> **ℹ️ GPIO 8/9 strapping:** The 10 kΩ pull-ups required by I²C conveniently hold GPIO8 and GPIO9 HIGH during reset, satisfying ESP32-C3 strapping requirements without extra components.

---

## Features

- **Real-time GPS** at up to 5 Hz with TinyGPSPlus (latitude, longitude, altitude, speed, course, HDOP, satellites)
- **GSM/GPRS data uplink** to a Cloudflare-proxied REST API backed by Supabase
- **Speed-adaptive bufferring** — 10 s packet interval moving or idle
- **LittleFS packet buffer** — stores up to 2,000 compact binary packets (≈ 48 bytes each) in flash; survives power loss
- **7-screen OLED UI** — main telemetry, GSM status, coordinates, system info, uplink diagnostics, TCP debug, module diags
- **Two-button navigation** — short/long press actions per screen
- **Wi-Fi OTA** — over-the-air firmware updates via GitHub Releases checked every 6 hours
- **Remote command polling** — dashboard can push commands (schedule updates, reboot, etc.) after each successful uplink
- **Tracking Schedule** — configurable time windows (up to 4) stored in NVS; checked every minute
- **Deep Sleep** — long-press BTN_B enters deep sleep; SIM800L RTC alarm can wake the device
- **Battery monitoring** — voltage + percentage via `AT+CBC`, updated every 60 s
- **Uplink diagnostics** — HTTP code, TCP stage, bytes sent all visible on-screen

---

## Architecture

### FreeRTOS Task Map

```
┌─────────────────────────────────────────────────────────────┐
│  Core 0 (ESP32-C3 single-core)                              │
│                                                             │
│  ┌─────────────┐  Priority 3  GPS Task      (200 ms tick)  │
│  │ gpsTask     │  Parses NMEA, fills DeviceState            │
│  └─────────────┘                                           │
│                                                             │
│  ┌─────────────┐  Priority 2  Buffer Task   (10 s tick)    │
│  │ bufferTask  │  Writes GPSPacket to LittleFS              │
│  └─────────────┘                                           │
│                                                             │
│  ┌─────────────┐  Priority 2  Uplink Task   (continuous)   │
│  │ uplinkTask  │  GSM init → GPRS drain → command poll      │
│  └─────────────┘                                           │
│                                                             │
│  ┌─────────────┐  Priority 1  Display Task  (100 ms tick)  │
│  │ displayTask │  Renders 7 OLED screens at 10 Hz           │
│  └─────────────┘                                           │
│                                                             │
│  ┌─────────────┐  Priority 1  Button Task   (50 ms poll)   │
│  │ buttonTask  │  Debounced button events → state changes   │
│  └─────────────┘                                           │
│                                                             │
│  ┌─────────────┐  Priority 1  Schedule Task (60 s tick)    │
│  │scheduleTask │  Checks NVS tracking windows               │
│  └─────────────┘                                           │
│                                                             │
│  Shared state: DeviceState struct (mutex-protected)         │
└─────────────────────────────────────────────────────────────┘
```

### Module Overview

| Module | File(s) | Responsibility |
|--------|---------|----------------|
| **GPS Manager** | `gps_manager.cpp/.h` | HardwareSerial + TinyGPSPlus, UBX config, fills `DeviceState` |
| **GSM Manager** | `gsm_manager.cpp/.h` | TinyGSM SIM800L wrapper, GPRS connect/reconnect, RTC alarm |
| **Packet Buffer** | `packet_buffer.cpp/.h` | LittleFS ring-buffer for binary GPS packets (max 2 000) |
| **Server Comm** | `server_comm.cpp/.h` | Raw TCP HTTP POST via TinyGSM client, JSON batch builder |
| **Display Manager** | `display_manager.cpp/.h` | Adafruit SH1107 driver, 7 screen renderers |
| **Button Handler** | `button_handler.cpp/.h` | GPIO polling, short/long press detection with debounce |
| **OTA Manager** | `ota_manager.cpp/.h` | GitHub Release + Supabase version check, ESP OTA flash |
| **Schedule Manager** | `schedule_manager.cpp/.h` | NVS-persisted tracking windows, UTC→local time conversion |
| **Power Manager** | `power_manager.cpp/.h` | Deep sleep entry, battery ADC reading (`AT+CBC`) |
| **Wi-Fi Manager** | `wifi_manager.cpp/.h` | Arduino WiFi bring-up for OTA / command channel |
| **Command Handler** | `command_handler.cpp/.h` | Parses JSON commands from server, dispatches to modules |

---

## Getting Started

### Prerequisites

1. **VS Code** with the [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) extension installed.
2. A **SIM card** with a data plan active on a 2G-compatible network (Hutch SL used by default — change `GPRS_APN` in `config.h` for other carriers).
3. A **Cloudflare Worker** (or any HTTPS proxy) forwarding `POST /uplink` to your Supabase `locations` table. (See [Cloud Integration](#cloud-integration).)

### Configuration

> **⚠️ `config.h` is excluded from version control because it contains credentials.** Copy the template and fill in your values:

```bash
cp include/config.h.example include/config.h
```

Then edit `include/config.h`:

```c
// — Wi-Fi (OTA & commands only) —
#define WIFI_SSID     "YourNetwork"
#define WIFI_PASS     "YourPassword"

// — GPRS —
#define GPRS_APN      "internet"   // Your carrier's APN
#define GPRS_USER     ""
#define GPRS_PASS     ""

// — Cloudflare Proxy —
#define PROXY_HOST    "api.yourdomain.com"
#define PROXY_PATH    "/uplink"
#define SUPABASE_TABLE "locations"

// — GitHub OTA —
#define GITHUB_REPO_OWNER  "youruser"
#define GITHUB_REPO_NAME   "gps-tracker-firmware"
#define GITHUB_TOKEN       ""   // Optional: for private repos

// — Device Identity —
#define DEVICE_ID     "GPS-001"
#define FW_VERSION    "1.0.0"
```

All other timing and pin constants are safe defaults and can be left as-is.

### Building & Flashing

```bash
# Open the project in VS Code with PlatformIO, then:

# Build
pio run

# Flash over USB
pio run --target upload

# Monitor serial output (115200 baud)
pio device monitor
```

Or use the PlatformIO sidebar buttons: **Build → Upload → Monitor**.

---

## Display Screens

Cycle through screens with **BTN_A short press**.

| Screen # | Name | Content |
|----------|------|---------|
| 0 | **Main** | GPS fix status, speed, altitude, time, satellite count |
| 1 | **GSM** | Network name, signal bars, GPRS status, packets sent |
| 2 | **Coordinates** | Latitude, longitude, course/compass, HDOP |
| 3 | **System** | Firmware version, uptime, battery voltage/%, buffer count |
| 4 | **Uplink** | Last HTTP code, last response, uplink timestamp |
| 5 | **TCP Debug** | TCP stage, header bytes sent, body bytes sent/expected |
| 6 | **Module Diags** | Wi-Fi SSID, RSSI, enabled/connected status |

---

## Button Controls

| Button | Press Type | Action |
|--------|-----------|--------|
| **BTN_A** | Short | Next screen |
| **BTN_A** | Long | Toggle display on/off |
| **BTN_B** | Short (Screen 4) | Trigger connectivity ping test |
| **BTN_B** | Short (Screen 6) | Toggle Wi-Fi on/off |
| **BTN_B** | Long | Enter deep sleep |

---

## Cloud Integration

### Data Uplink (GPRS)

#### Why a Cloudflare Proxy is Required

Supabase's REST API exclusively accepts **HTTPS connections secured with TLS 1.2 or TLS 1.3**. The SIM800L GSM modem, however, only supports legacy SSL (SSLv3 / TLS 1.0) through its `AT+CIPSSL` command — modern HTTPS endpoints actively reject these older handshakes. This means the SIM800L **cannot reach Supabase directly over HTTPS**.

To work around this hardware limitation, the uplink uses the following bridge architecture:

```
  ESP32-C3 + SIM800L                 Internet
  ───────────────────                ─────────────────────────────────────────

  Plain HTTP POST         ──────►  Cloudflare Worker          HTTPS POST
  (port 80, no TLS)                (api.yourdomain.com)  ──►  (Supabase REST API)
  JSON body                         receives plain HTTP        TLS 1.3, full SSL
                                    forwards securely
                                    as HTTPS to Supabase
```

1. **SIM800L → Cloudflare Worker** — The device sends a plain `HTTP POST` on port 80 with JSON in the body. No SSL handshake is required, so any 2G modem can communicate reliably.
2. **Cloudflare Worker → Supabase** — Cloudflare terminates the plain HTTP request on its edge network and re-issues it to Supabase as a proper `HTTPS POST` with a full TLS 1.3 certificate chain. Cloudflare also injects the Supabase API key (`apikey` / `Authorization` headers) server-side, so the secret never needs to be stored on the device.
3. **Supabase** — Receives a valid, authenticated HTTPS request and inserts the row into the `locations` table.

This pattern is sometimes called an **HTTP-to-HTTPS bridge** or **TLS offloading proxy** and is the standard approach for IoT devices with legacy GSM modems connecting to modern cloud backends.

#### Supabase Table Schema

```sql
CREATE TABLE locations (
  id          bigserial PRIMARY KEY,
  device_id   text,
  timestamp   bigint,       -- UTC epoch (seconds)
  latitude    double precision,
  longitude   double precision,
  altitude_m  real,
  speed_kmh   real,
  course      real,
  satellites  int,
  hdop        real,
  gsm_signal  int,
  battery_v   real
);
```

#### Setting Up the Cloudflare Worker

1. Log in to [Cloudflare Dashboard](https://dash.cloudflare.com) → **Workers & Pages** → Create a new Worker.
2. Deploy a Worker script that:
   - Accepts `POST` requests on your chosen route (e.g. `api.yourdomain.com/uplink`)
   - Reads the JSON body from the incoming plain-HTTP request
   - Forwards it to `https://<your-project>.supabase.co/rest/v1/locations` with the `apikey` and `Authorization` headers containing your Supabase service-role key
3. Assign a custom domain (e.g. `api.yourdomain.com`) to the Worker so the device can reach it by hostname.
4. Update `PROXY_HOST` and `PROXY_PATH` in `config.h` to match your Worker's hostname and route.

The uplink pipeline is **fire-and-forget** — sent packets are immediately removed from the buffer. There is no retry queue, ensuring old data is never re-uploaded after a long connectivity outage.

### OTA Updates (GitHub + Supabase)

The `OTAManager` checks for firmware updates every 6 hours (configurable via `OTA_CHECK_INTERVAL`):

1. **GitHub Releases API** — queries `https://api.github.com/repos/{owner}/{repo}/releases/latest` for a new `tag_name`.
2. **Supabase metadata** — used to fetch the signed `.bin` URL and SHA-256 checksum.
3. If the remote version differs from `FW_VERSION`, it downloads and flashes via ESP32 OTA (Update library), then reboots into the new firmware.

To publish a new release:
1. Build the firmware: `pio run`
2. Create a GitHub Release tagged `v1.x.x`.
3. Attach the compiled `.bin` (found in `.pio/build/esp32c3/firmware.bin`) as a release asset.
4. Update your Supabase OTA metadata record with the new version, URL, and SHA-256.

### Remote Commands

After each successful uplink cycle, the device polls the server for pending commands. Commands are JSON payloads that can:
- Update tracking schedule windows
- Trigger a reboot
- Change uplink intervals
- Enable/disable features

---

## Partition Layout

Custom dual-OTA partition table for seamless over-the-air updates:

```
# Name      Type  SubType  Offset    Size
nvs         data  nvs      0x009000  0x005000   (NVS storage)
otadata     data  ota      0x00E000  0x002000   (OTA slot selector)
app0        app   ota_0    0x010000  0x1A0000   (Firmware slot A ~1.6 MB)
app1        app   ota_1    0x1B0000  0x1A0000   (Firmware slot B ~1.6 MB)
spiffs      data  spiffs   0x350000  0x0B0000   (LittleFS packet buffer ~704 KB)
```

---

## Dependencies

Managed automatically by PlatformIO (`platformio.ini`):

| Library | Version | Purpose |
|---------|---------|---------|
| [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus) | ^1.1 | NMEA sentence parser |
| [Adafruit SH110X](https://github.com/adafruit/Adafruit_SH110X) | ^2.1 | SH1107 OLED driver |
| [Adafruit GFX Library](https://github.com/adafruit/Adafruit-GFX-Library) | ^1.11 | Graphics primitives |
| [TinyGSM](https://github.com/vshymanskyy/TinyGSM) | ^0.11 | SIM800L AT modem abstraction |
| [ArduinoJson](https://github.com/bblanchon/ArduinoJson) | ^7.0 | JSON serialisation |
| Wire | built-in | I²C for OLED |

Build flags set in `platformio.ini`:
```ini
build_flags =
    -DTINY_GSM_MODEM_SIM800
    -DCORE_DEBUG_LEVEL=3
    -DARDUINOJSON_ENABLE_PROGMEM=1
```

---

## Project Structure

```
GPS V1.0/
├── include/
│   └── config.h.example         # Configuration template — copy to config.h and fill in your values
├── src/
│   ├── main.cpp                 # Entry point, FreeRTOS task creation
│   ├── gps_manager.cpp/.h       # NEO-6M UART parsing, DeviceState fill
│   ├── gsm_manager.cpp/.h       # SIM800L TinyGSM wrapper
│   ├── wifi_manager.cpp/.h      # Wi-Fi bring-up for OTA
│   ├── packet_buffer.cpp/.h     # LittleFS ring-buffer (2 000 packets)
│   ├── server_comm.cpp/.h       # HTTP POST over raw GSM TCP
│   ├── display_manager.cpp/.h   # SH1107 OLED, 7-screen renderer
│   ├── button_handler.cpp/.h    # Debounced short/long press events
│   ├── ota_manager.cpp/.h       # GitHub + Supabase OTA
│   ├── schedule_manager.cpp/.h  # NVS tracking windows
│   ├── power_manager.cpp/.h     # Deep sleep + battery ADC
│   └── command_handler.cpp/.h   # Remote command dispatcher
├── partitions.csv               # Custom dual-OTA partition table
├── platformio.ini               # PlatformIO build configuration
└── README.md
```

---

## Contributing

1. Fork the repository and create your feature branch: `git checkout -b feature/my-feature`
2. Commit your changes: `git commit -m 'Add my feature'`
3. Push to the branch: `git push origin feature/my-feature`
4. Open a Pull Request

Please ensure all changes build cleanly with `pio run` before submitting.

---

## License

This project is released under the [MIT License](LICENSE).

---

*Built with ❤️ using PlatformIO + Arduino framework on ESP32-C3.*
