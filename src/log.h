#pragma once

// ============================================================
//  Diagnostic logging — disabled
// ============================================================
// On the ESP32-C3 hardware used by this project the Arduino "Serial"
// object (UART0) is wired in two conflicting ways:
//
//   * Before gps_manager.begin() runs, Serial defaults to GPIO 20/21,
//     which on this board are the SIM800L UART pins. Any byte written
//     to Serial at that point is interpreted by the modem as garbage
//     AT input — the modem then refuses to register, never opens GPRS,
//     and uplink silently fails.
//
//   * After gps_manager.begin(), Serial is remapped to the NEO-6M GPS
//     UART (GPIO 4/5). Bytes written there are still harmless to the
//     receiver but waste UART bandwidth at 9600/38400 baud.
//
// The previous code used Serial.print/println/printf liberally for
// diagnostics. Those calls are now no-ops via the LOG macro below; the
// original strings stay in place as documentation but emit no code.
// Use the OLED diagnostic screens (DeviceState fields) for runtime
// visibility instead.
//
// If you ever need to re-enable trace output for bench debugging, point
// it at the Wire / I2C bus or USB-CDC — never at the bare UART.
#define LOG(...) ((void)0)
