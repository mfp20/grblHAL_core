/*
  driver_opts.h - for preprocessing options from my_machine.h or compiler symbols

  NOTE: This file is not used by the core, it may be used by drivers

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// NOTE: do NOT change options here - edit the driver specific my_machine.h instead!
//

#pragma once

#include "hal.h"
#include "nuts_bolts.h"

#ifndef X_GANGED
#define X_GANGED            0
#endif
#ifndef X_AUTO_SQUARE
#define X_AUTO_SQUARE       0
#endif
#ifndef Y_GANGED
#define Y_GANGED            0
#endif
#ifndef Y_AUTO_SQUARE
#define Y_AUTO_SQUARE       0
#endif
#ifndef Z_GANGED
#define Z_GANGED            0
#endif
#ifndef Z_AUTO_SQUARE
#define Z_AUTO_SQUARE       0
#endif
#ifndef X_GANGED_LIM_MAX
#define X_GANGED_LIM_MAX    0
#endif
#ifndef Y_GANGED_LIM_MAX
#define Y_GANGED_LIM_MAX    0
#endif
#ifndef Z_GANGED_LIM_MAX
#define Z_GANGED_LIM_MAX    0
#endif

#if X_AUTO_SQUARE && !X_GANGED
#undef X_GANGED
#define X_GANGED 1
#endif
#if Y_AUTO_SQUARE && !Y_GANGED
#undef Y_GANGED
#define Y_GANGED 1
#endif
#if Z_AUTO_SQUARE && !Z_GANGED
#undef Z_GANGED
#define Z_GANGED 1
#endif

#define N_GANGED (X_GANGED + Y_GANGED + Z_GANGED)
#define N_AUTO_SQUARED (X_AUTO_SQUARE + Y_AUTO_SQUARE + Z_AUTO_SQUARE)
#define N_ABC_MOTORS (N_ABC_AXIS + N_GANGED)

#ifndef USB_SERIAL_CDC
#define USB_SERIAL_CDC      0 // for UART comms
#endif
#ifndef USB_SERIAL_WAIT
#define USB_SERIAL_WAIT     0
#endif
#ifndef SDCARD_ENABLE
#define SDCARD_ENABLE       0
#endif
#ifndef KEYPAD_ENABLE
#define KEYPAD_ENABLE       0
#endif
#ifndef EEPROM_ENABLE
#define EEPROM_ENABLE       0
#endif
#ifndef EEPROM_IS_FRAM
#define EEPROM_IS_FRAM      0
#endif
#ifndef SPINDLE_SYNC_ENABLE
#define SPINDLE_SYNC_ENABLE 0
#endif
#ifndef TRINAMIC_ENABLE
#define TRINAMIC_ENABLE     0
#endif
#ifndef TRINAMIC_I2C
#define TRINAMIC_I2C        0
#endif
#if TRINAMIC_ENABLE && TRINAMIC_I2C
#define TRINAMIC_MOTOR_ENABLE 1
#else
#define TRINAMIC_MOTOR_ENABLE 0
#endif
#ifndef TRINAMIC_DEV
#define TRINAMIC_DEV        0
#endif
#ifndef PWM_RAMPED
#define PWM_RAMPED          0
#endif
#ifndef PLASMA_ENABLE
#define PLASMA_ENABLE       0
#endif
#ifndef PPI_ENABLE
#define PPI_ENABLE          0
#endif
#ifndef SPINDLE_HUANYANG
#define SPINDLE_HUANYANG    0
#endif
#ifndef QEI_ENABLE
#define QEI_ENABLE          0
#endif
#ifndef ODOMETER_ENABLE
#define ODOMETER_ENABLE     0
#endif
#ifndef BLUETOOTH_ENABLE
#define BLUETOOTH_ENABLE    0
#endif
#ifndef OPENPNP_ENABLE
#define OPENPNP_ENABLE      0
#endif
#ifndef FANS_ENABLE
#define FANS_ENABLE         0
#endif
#ifndef LIMITS_OVERRIDE_ENABLE
#define LIMITS_OVERRIDE_ENABLE  0
#endif

#ifndef ESTOP_ENABLE
  #if COMPATIBILITY_LEVEL <= 1
    #define ESTOP_ENABLE    1
  #else
    #define ESTOP_ENABLE    0
  #endif
#elif ESTOP_ENABLE && COMPATIBILITY_LEVEL > 1
  #warning "Enabling ESTOP may not work with all senders!"
#endif

#ifndef ETHERNET_ENABLE
#define ETHERNET_ENABLE     0
#endif
#ifndef TELNET_ENABLE
#define TELNET_ENABLE       0
#endif
#ifndef WEBSOCKET_ENABLE
#define WEBSOCKET_ENABLE    0
#endif
#ifndef FTP_ENABLE
#define FTP_ENABLE          0
#elif !SDCARD_ENABLE
#undef FTP_ENABLE
#define FTP_ENABLE          0
#endif

#if ETHERNET_ENABLE
#ifndef NETWORK_HOSTNAME
#define NETWORK_HOSTNAME        "GRBL"
#endif
#ifndef NETWORK_IPMODE
#define NETWORK_IPMODE          1 // 0 = static, 1 = DHCP, 2 = AutoIP
#endif
#ifndef NETWORK_IP
#define NETWORK_IP              "192.168.5.1"
#endif
#ifndef NETWORK_GATEWAY
#define NETWORK_GATEWAY         "192.168.5.1"
#endif
#ifndef NETWORK_MASK
#define NETWORK_MASK            "255.255.255.0"
#endif
#ifndef NETWORK_TELNET_PORT
#define NETWORK_TELNET_PORT     23
#endif
#ifndef NETWORK_WEBSOCKET_PORT
#define NETWORK_WEBSOCKET_PORT  80
#endif
#ifndef NETWORK_HTTP_PORT
#define NETWORK_HTTP_PORT       80
#endif
#if NETWORK_IPMODE < 0 || NETWORK_IPMODE > 2
#error "Invalid IP mode selected!"
#endif
#endif

/*EOF*/
