# GPS Tracker V1.0 — ESP32-C3 Firmware

> A production-grade, multi-tasked GPS tracker firmware built on the **ESP32-C3** with GSM uplink, OLED display, OTA updates, and a cloud-connected backend.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Hardware](#hardware)
  - [Bill of Materials](#bill-of-materials)
  - [Power System](#power-system)
  - [Complete Wiring Guide](#complete-wiring-guide)
    - [System Block Diagram](#system-block-diagram)
    - [Power Rail Connections](#power-rail-connections)
    - [ESP32-C3 → SIM800L](#esp32-c3--sim800l)
    - [ESP32-C3 → NEO-6M GPS](#esp32-c3--neo-6m-gps)
    - [ESP32-C3 → SH1107 OLED](#esp32-c3--sh1107-oled)
    - [ESP32-C3 → Buttons](#esp32-c3--buttons)
    - [Full GPIO Map](#full-gpio-map)
  - [Assembly Tips](#assembly-tips)
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

**GPS Tracker V1.0** is a fully self-contained asset tracking device built around the **Espressif ESP32-C3** microcontroller. It continuously parses NMEA data from a **u-blox NEO-6M** GPS module, buffers packets in flash (LittleFS), and drains them to a cloud backend via a **SIM800L** GSM modem over GPRS. A **SH1107 128×128 OLED** display with a two-button UI provides on-device telemetry, and deep-sleep support extends battery life between tracking windows.

Key design goals:
- **Fire-and-forget** upload pipeline — no retry queue; prevents stale-data reupload.
- **Speed-adaptive** sampling — faster packet generation when moving, slower when idle.
- **Zero blocking** — GSM initialisation offloaded to a dedicated FreeRTOS task to keep the UI alive from first boot.
- **OTA-capable** — firmware can be updated remotely via GitHub Releases without physical access.

---

## Hardware

### Bill of Materials

| # | Component | Part / Module | Qty | Notes |
|---|-----------|--------------|-----|-------|
| 1 | Microcontroller | ESP32-C3 DevKitM-1 | 1 | RISC-V core, 4 MB Flash, Wi-Fi + BLE |
| 2 | GPS Module | u-blox NEO-6M (GY-NEO6MV2) | 1 | UART, 9600 baud, 3.3 V logic, ceramic patch antenna |
| 3 | GSM Modem | SIM800L EVB | 1 | 2G GPRS quad-band, UART, needs **3.4–4.4 V** |
| 4 | Display | SH1107 128×128 OLED (1.5") | 1 | I²C, 3.3 V, address 0x3C |
| 5 | Tactile Buttons | 6×6 mm momentary switch | 2 | SELECT (BTN_A) + BACK (BTN_B) |
| 6 | Battery Cells | 18650 Li-ion × 2 | 2 | Connected **in parallel** (same voltage, doubled capacity) |
| 7 | Protection Board | 3A BMS for 3.7 V 18650 | 1 | Over-charge, over-discharge, short-circuit protection |
| 8 | Charging Module | IP2312 USB-C Fast Charge (3A) | 1 | 5 V USB-C in → 4.2 V Li-ion charge out |
| 9 | Resistors | 10 kΩ × 2 | 2 | I²C pull-ups on SDA/SCL |
| 10 | Capacitor | 100 µF electrolytic | 1 | Bulk decoupling on SIM800L VCC rail |
| 11 | Wire / Jumpers | 22–26 AWG | — | Keep SIM800L power wires short and thick (≥22 AWG) |

---

### Power System

The tracker runs from a **dual 18650 Li-ion pack** managed by dedicated BMS and charging ICs. Understanding the power chain is essential before wiring anything.

#### Power Chain Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        POWER CHAIN                                  │
│                                                                     │
│  USB-C Port                                                         │
│  (5 V input)                                                        │
│      │                                                              │
│      ▼                                                              │
│  ┌──────────────────────┐                                           │
│  │  IP2312              │  Fast-charge IC                           │
│  │  USB-C Charge Module │  5 V → 4.2 V CC/CV, up to 3 A           │
│  └──────────┬───────────┘                                           │
│             │  VBAT (3.7–4.2 V)                                     │
│             ▼                                                        │
│  ┌──────────────────────┐                                           │
│  │  3A BMS              │  Balancing, over-charge/discharge,        │
│  │  Protection Board    │  short-circuit protection                 │
│  └──────────┬───────────┘                                           │
│             │  VBAT_PROTECTED (3.7–4.2 V)                           │
│             ├──────────────────────────────────────────────────┐    │
│             │                                                  │    │
│             ▼                                                  ▼    │
│  ┌─────────────────┐                              ┌─────────────────┐│
│  │  18650 Cell A   │◄──── Parallel ────►          │  18650 Cell B   ││
│  │  (3.7 V nom.)   │                              │  (3.7 V nom.)   ││
│  └─────────────────┘                              └─────────────────┘│
│                                                                     │
│  VBAT_PROTECTED also feeds the peripherals:                         │
│      ├──► SIM800L VCC  (3.4–4.4 V direct — do NOT regulate down)   │
│      │                                                              │
│      └──► ESP32-C3 DevKitM-1 (via onboard 3.3 V LDO on the devkit) │
│               └──► NEO-6M VCC (3.3 V from ESP32-C3 3.3 V pin)      │
│               └──► SH1107 OLED VCC (3.3 V)                         │
└─────────────────────────────────────────────────────────────────────┘
```

#### Power Component Details

**① 18650 Cells — 2× in Parallel**
- Connect **positive-to-positive** and **negative-to-negative** between both cells.
- Parallel configuration keeps voltage at **3.7 V nominal / 4.2 V full**, doubles capacity (e.g., 2× 3000 mAh = 6000 mAh).
- ⚠️ **Always use matched cells** (same capacity, same brand, same charge level at assembly).

**② 3A BMS Protection Board**
- Insert **between the cell pack and the rest of the circuit** — *never* power peripherals directly from bare cells.
- Provides: over-charge cut-off (≈4.25 V), over-discharge cut-off (≈2.5 V), over-current and short-circuit protection.
- Wiring: `B+` → cell pack positive, `B-` → cell pack negative; `P+/C+` → load/charger positive, `P-/C-` → load/charger negative.

**③ IP2312 USB-C Fast Charge Module**
- Handles **5 V USB-C PD/BC1.2 input** and outputs regulated **4.2 V CC/CV** to the BMS/battery.
- Up to **3 A** charge current (set by the `ISET` resistor on the module).
- Connect: `VIN+/VIN-` to the USB-C port 5 V rail; `VBAT+/VBAT-` to the BMS `C+/C-` (charge) pins.
- The IP2312 typically has a built-in LED indicator for charge status — no extra components needed.

> **⚠️ SIM800L Power Warning:** The SIM800L can draw **up to 2 A in burst** during GSM transmission. Power it **directly from `VBAT_PROTECTED`** (3.7–4.2 V), NOT from the ESP32-C3's 3.3 V pin (which is limited to ~600 mA). Add a **100 µF electrolytic capacitor** across the SIM800L VCC/GND pins to absorb transient spikes. Use **at least 22 AWG wire** for all SIM800L power connections.

---

### Complete Wiring Guide

#### System Block Diagram

```
                         ┌─────────────────────────────────────────────────────┐
                         │            GPS TRACKER V1.0 — BLOCK DIAGRAM         │
                         └─────────────────────────────────────────────────────┘

  ┌───────────────┐     VBAT (3.7–4.2V)      ┌──────────────────────────────┐
  │  2× 18650     ├───────────────────────────►  IP2312 USB-C Charger        │
  │  (Parallel)   │                           │  (charges via USB-C 5V)     │
  └───────┬───────┘                           └──────────────────────────────┘
          │ VBAT
          ▼
  ┌───────────────┐
  │  3A BMS       │  ← short-circuit & over-voltage protection
  └───────┬───────┘
          │ VBAT_PROTECTED
          ├─────────────────────────────────────────────────────────┐
          │                                                         │
          ▼                                                         ▼
  ┌────────────────────┐  UART2 (TX20/RX21)   ┌──────────────────────────────┐
  │                    ├─────────────────────►│  SIM800L GSM Modem           │
  │                    │  RST: GPIO0           │  (GPRS uplink, 2G network)  │
  │                    │  RI:  GPIO10          │                              │
  │                    │                       │  Antenna ─────────► GSM ANT │
  │   ESP32-C3         │                       └──────────────────────────────┘
  │   DevKitM-1        │
  │   (3.3V LDO on     │  UART1 (TX5/RX4)     ┌──────────────────────────────┐
  │    board)          ├─────────────────────►│  NEO-6M GPS                  │
  │                    │                       │  (NMEA @ 9600 baud)          │
  │                    │                       │  Antenna ─────────► GPS ANT  │
  │                    │                       └──────────────────────────────┘
  │                    │
  │                    │  I2C (SDA8/SCL9)      ┌──────────────────────────────┐
  │                    ├─────────────────────►│  SH1107 128×128 OLED         │
  │                    │                       │  (status display, I2C 0x3C)  │
  │                    │                       └──────────────────────────────┘
  │                    │
  │                    │  GPIO3 ───────────── Button A (SELECT)
  │                    │  GPIO1 ───────────── Button B (BACK)
  └────────────────────┘
```

---

#### Power Rail Connections

| From | To | Wire | Notes |
|------|----|------|-------|
| Cell Pack `B+` | BMS `B+` | 22 AWG red | Match both cells first |
| Cell Pack `B-` | BMS `B-` | 22 AWG black | |
| BMS `P+` | IP2312 `VBAT+` | 22 AWG red | Charge path |
| BMS `P-` | IP2312 `VBAT-` | 22 AWG black | |
| BMS `P+` | SIM800L `VCC` | 22 AWG red | Direct battery voltage — crucial! |
| BMS `P-` | SIM800L `GND` | 22 AWG black | |
| BMS `P+` → 100µF cap → GND | SIM800L `VCC` | — | Electrolytic cap across SIM800L power pins |
| BMS `P+` | ESP32-C3 `VIN` | 22 AWG red | DevKit onboard LDO → 3.3 V |
| BMS `P-` | ESP32-C3 `GND` | 22 AWG black | Common ground |
| ESP32-C3 `3.3V` | NEO-6M `VCC` | 26 AWG red | GPS runs on 3.3 V |
| ESP32-C3 `GND` | NEO-6M `GND` | 26 AWG black | |
| ESP32-C3 `3.3V` | OLED `VCC` | 26 AWG red | |
| ESP32-C3 `GND` | OLED `GND` | 26 AWG black | |

> **ℹ️ Common Ground:** All GND pins across all modules (ESP32-C3, SIM800L, NEO-6M, OLED, BMS) must be connected together as a single common ground reference.

---

#### ESP32-C3 → SIM800L

| ESP32-C3 Pin | SIM800L Pin | Signal | Notes |
|-------------|-------------|--------|-------|
| `GPIO 20` (TX) | `RXD` | UART TX | 3.3 V logic — SIM800L is 5 V tolerant |
| `GPIO 21` (RX) | `TXD` | UART RX | SIM800L outputs 2.8 V logic — safe for ESP32 |
| `GPIO 0` | `RST` or `PWRKEY` | Reset / Power Key | Pull LOW for 1 s to toggle power |
| `GPIO 10` | `RI` | Ring Indicator | Optional — wake-on-SMS |
| `GND` | `GND` | Ground | Must share common GND |
| *(VBAT from BMS)* | `VCC` | Power | **3.7–4.2 V direct** — do NOT use 3.3 V pin |

> **⚠️ Do NOT connect SIM800L VCC to the ESP32-C3's 3.3 V pin.** The SIM800L bursts up to 2 A and will brown-out or permanently damage the ESP32-C3's onboard LDO. Power it directly from the battery-protected rail.

---

#### ESP32-C3 → NEO-6M GPS

| ESP32-C3 Pin | NEO-6M Pin | Signal | Notes |
|-------------|-----------|--------|-------|
| `GPIO 5` (TX) | `RX` | UART TX | Sends config commands to GPS |
| `GPIO 4` (RX) | `TX` | UART RX | Receives NMEA sentences |
| `3.3V` | `VCC` | Power | 3.3 V — do not exceed |
| `GND` | `GND` | Ground | |

> **ℹ️ Antenna placement:** Mount the NEO-6M with the ceramic patch antenna facing upward, ideally near a window or with a clear sky view. Avoid placing it near metal enclosures or the SIM800L antenna.

---

#### ESP32-C3 → SH1107 OLED

| ESP32-C3 Pin | OLED Pin | Signal | Notes |
|-------------|---------|--------|-------|
| `GPIO 8` | `SDA` | I²C Data | Add 10 kΩ pull-up to 3.3 V |
| `GPIO 9` | `SCL` | I²C Clock | Add 10 kΩ pull-up to 3.3 V |
| `3.3V` | `VCC` | Power | |
| `GND` | `GND` | Ground | |

> **ℹ️ GPIO 8/9 Strapping Pins:** GPIO8 and GPIO9 are ESP32-C3 strapping pins that must be HIGH at boot. The 10 kΩ I²C pull-up resistors naturally hold them HIGH, satisfying this requirement without any extra components. Connect one end of each 10 kΩ resistor to 3.3 V and the other end to the respective GPIO/OLED pin.

---

#### ESP32-C3 → Buttons

Both buttons are wired as **active-low** (pressed = LOW). The firmware uses the internal pull-up (`INPUT_PULLUP` mode).

| ESP32-C3 Pin | Button | Other Terminal | Signal |
|-------------|--------|----------------|--------|
| `GPIO 3` | Button A (SELECT / CALL) | `GND` | Active LOW |
| `GPIO 1` | Button B (BACK / HANG) | `GND` | Active LOW |

No external resistors are required — the ESP32-C3's internal pull-ups are used.

---

#### Full GPIO Map

| GPIO | Direction | Function | Peripheral |
|------|-----------|----------|------------|
| 0 | OUT | SIM800L RST/PWRKEY | GSM Modem |
| 1 | IN | Button B (BACK) | Tactile switch |
| 3 | IN | Button A (SELECT) | Tactile switch |
| 4 | IN (UART1 RX) | GPS NMEA receive | NEO-6M TX |
| 5 | OUT (UART1 TX) | GPS config transmit | NEO-6M RX |
| 8 | I/O (I²C SDA) | Display data | SH1107 OLED |
| 9 | OUT (I²C SCL) | Display clock | SH1107 OLED |
| 10 | IN | SIM800L Ring Indicator | GSM Modem |
| 20 | OUT (UART2 TX) | GSM command transmit | SIM800L RXD |
| 21 | IN (UART2 RX) | GSM response receive | SIM800L TXD |

---

### Assembly Tips

> 💡 **For beginners — recommended build order:**

1. **Test each module independently first.** Flash a simple sketch that only talks to one peripheral at a time before combining everything.
2. **Double-check power rails** before connecting anything: confirm BMS `P+` outputs ~3.7–4.2 V with a multimeter.
3. **SIM800L power first** — if it doesn't register on the network, check its VCC voltage (should be 3.7–4.2 V) and look for the `NETLIGHT` LED blinking every 3 s (registered).
4. **Keep wire lengths short** — especially power wires to the SIM800L. Long wires add resistance and inductance that cause brownout resets during GSM transmit bursts.
5. **Strain-relieve antennas** — the GSM and GPS antenna connectors (U.FL/IPEX) are fragile. Secure the antenna cable so it can't pull on the connector.
6. **Serial monitor is your friend** — open the PlatformIO serial monitor at 115200 baud; every module logs its status with a `[TAG]` prefix.
7. **Common faults checklist:**
   - OLED blank → check SDA/SCL not swapped, and I²C address is 0x3C
   - No GPS fix → antenna facing sky? Wait up to 2 minutes for cold-start
   - SIM800L rebooting → power issue; add 100 µF cap, check wire gauge
   - GPRS not connecting → verify APN in `config.h` matches your network carrier

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

Each GPS packet is serialised as a JSON batch and `POST`ed over a raw TCP connection to a **Cloudflare Worker** acting as an HTTPS proxy. The worker forwards it to a **Supabase** table with the following schema (minimum required columns):

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
│   ├── config.h             # ⚠️  Local only — contains credentials (gitignored)
│   └── config.h.example     # Template — safe to commit
├── src/
│   ├── main.cpp             # Entry point, FreeRTOS task creation
│   ├── gps_manager.cpp/.h   # NEO-6M UART parsing, DeviceState fill
│   ├── gsm_manager.cpp/.h   # SIM800L TinyGSM wrapper
│   ├── wifi_manager.cpp/.h  # Wi-Fi bring-up for OTA
│   ├── packet_buffer.cpp/.h # LittleFS ring-buffer (2 000 packets)
│   ├── server_comm.cpp/.h   # HTTP POST over raw GSM TCP
│   ├── display_manager.cpp/.h # SH1107 OLED, 7-screen renderer
│   ├── button_handler.cpp/.h  # Debounced short/long press events
│   ├── ota_manager.cpp/.h   # GitHub + Supabase OTA
│   ├── schedule_manager.cpp/.h # NVS tracking windows
│   ├── power_manager.cpp/.h # Deep sleep + battery ADC
│   └── command_handler.cpp/.h # Remote command dispatcher
├── partitions.csv           # Custom dual-OTA partition table
├── platformio.ini           # PlatformIO build configuration
├── build.log                # (gitignored) Last build output
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
