/*
  motion.h - General motion structures

  Part of grblHAL

  Copyright (c) 2021 Anichang

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

/*! \file
    \brief General motion structures.
*/

#ifndef _MOTION_H_
#define _MOTION_H_

#include "grbl.h"

#define MOTION_VERSION 1

#ifndef BOARD_OFFLOAD_TO_CORE
#define BOARD_OFFLOAD_TO_CORE 0
#endif

#ifndef BOARD_OFFLOAD_TO_HOST
#define BOARD_OFFLOAD_TO_HOST 0
#endif

#ifndef BOARD_OFFLOAD_GRBL_CORE
#define BOARD_OFFLOAD_GRBL_CORE 0
#endif

// BOARD_OFFLOAD_GRBL_CORE implies BOARD_OFFLOAD_TO_HOST
#if BOARD_OFFLOAD_GRBL_CORE
#undef BOARD_OFFLOAD_TO_HOST
#define BOARD_OFFLOAD_TO_HOST 1
#endif

typedef struct {
    /*! \brief Execute one motion block (gcode or raw steps).

    For single core boards without host offloading it is set to execute_gcode().
    On multicore boards or offloading motion to host, it is set to st_push_segment().
    \returns current parser status.
    */
    status_code_t (*protocol_execute_motion_data)(char *block, char *message);
} grbl_motion_t;

extern grbl_motion_t motion; //!< Global Motion struct.

#endif
