/*
  motion.h - General motion structures and methods

  (Wannabe) Part of grblHAL

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
    \brief General motion structures and methods.
*/

#include "hal.h"
#include "gcode.h"
#include "stepper.h"

void motion_computing_loop(void) {
    bool running = true;
    char *block;
    char *message;

    while (running) {
        // get new block if any (non-blocking queue read)
        if (hal.pop_motion_data(block, message))
            execute_gcode(char *block, char *message);
        // prep segment and steps buffers
        st_prep_buffer(true, true);
    }
}
