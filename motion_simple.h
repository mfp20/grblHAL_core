/*
  motion_simple.h - Motion entry points structures and capabilities type

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
    \brief Motion function pointers and data structures definitions.
*/

#ifndef _MOTION_H_
#define _MOTION_H_

#include "grbl.h"

#define MOTION_VERSION 1

typedef struct {
    /*! \brief Execute one motion block (gcode or raw steps).

    For single core boards without host offloading it is set to gc_execute_block().
    On multicore boards or offloading motion to host, it is set to simple_execute_block().
    \returns current parser status.
    */
    status_code_t (*protocol_execute_block)(char *block, char *message);
} grbl_motion_t;

extern grbl_motion_t motion; //!< Global Motion struct.

// Execute one block of simple raw steps
status_code_t simple_execute_block (char *block, char *message);

#endif
