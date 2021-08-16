/*
  protocol_bin.h - binary protocol for grblhk

  (Wannabe) Part of grblhk

  Copyright (c) 2021 Anichang

  Grblhk is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grblhk is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grblhk.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "protocol_bin.h"

bool binary_mode = false;

crc_ptr crc = NULL;
binary_parser_ptr command_parser = NULL;
binary_parser_ptr gcode_parser = NULL;
binary_parser_ptr buffer_data = NULL;
binary_parser_ptr user_input = NULL;
binary_parser_ptr unknown_binary_type = NULL;

// search for next COBS_DELIMITER in message, returns zero if none
static uint16_t protocol_binary_search_delimiter(uint16_t size, uint8_t *message, uint16_t index) {
    for (int i=1;index+i<size;i++) {
        if (message[index+i] == COBS_DELIMITER)
            return i;
    }
    return 0;
}

// COBS encoder with CRC16
void protocol_binary_put(uint16_t size, uint8_t *message)
{
    uint8_t frame_code = 0;
    uint16_t frame_crc16 = 0;
    bool eom = false;

    for (int i=0;i<size;i++) {
        if (message[i] == COBS_DELIMITER)
        { // encode delimiters
            frame_code = protocol_binary_search_delimiter(size, message, i);
            if (frame_code == 0) { // no delimiters until the end of the message
                frame_code = size-i+1; // next delimiter will be the end one
                eom = true;
            }
            frame_crc16 = crc(frame_crc16, frame_code);
            binary_send(frame_code);
        }
        else
        { // raw byte as is
            frame_crc16 = crc(frame_crc16, message[i]);
            binary_send(message[i]);
        }
    }
    //
    binary_send(COBS_DELIMITER);
    // adds crc16
    binary_send((uint8_t)(frame_crc16 >> 8));
    binary_send((uint8_t)(frame_crc16 & 0xff));
}

// frame dispatcher
static void protocol_binary_dispatch(uint16_t count, char *data) {
    switch (data[0]) {
        case MSG_TYPE_COMMAND:
            if (command_parser)
                command_parser(count, data);
            break;
        case MSG_TYPE_GCODE:
            if (gcode_parser)
                gcode_parser(count, data);
            break;
        case MSG_TYPE_BUFFER:
            if (buffer_data)
                buffer_data(count, data);
            break;
        case MSG_TYPE_USER:
            if (user_input)
                user_input(count, data);
            break;
        default:
            if (unknown_binary_type)
                unknown_binary_type(count, data);
            break;
    }
}

// COBS message decoder and CRC16 check
bool protocol_binary_get(char c) {
    static uint8_t frame_code = 0;
    static uint8_t frame_code_index = 0;
    static uint8_t frame_buffer[BIN_BUFFER_SIZE];
    static uint16_t frame_buffer_write_index = 0;
    static uint16_t frame_crc16 = 0;

    if (binary_mode) {
        if (frame_code_index==0) { // COBS first byte
            if ((uint8_t)c == COBS_DELIMITER)
            { // empty frame, abort binary mode
                return false;
            }
            else
            { // first COBS code
                frame_code = (uint8_t)c;
                // recompute crc
                if (crc) frame_crc16 = crc(frame_crc16, frame_code);
                frame_code_index++;
            }
        }
        else
        { // COBS 2nd byte and following bytes
            if (frame_code_index>frame_code)
            { // error malformed frame, all data trashed, abort binary mode
                return false;
            } 
            else if (frame_code_index==frame_code)
            { // this byte is new code
                if ((uint8_t)c == COBS_DELIMITER) { // this was last byte, trash delimiter
                    // crc check
                    if (crc == NULL || ( (uint16_t)(frame_buffer[frame_buffer_write_index-1]) + (((uint16_t)frame_buffer[frame_buffer_write_index-2])<<8) ) == frame_crc16)) {
                        // dispatch the frame
                        protocol_binary_dispatch(frame_buffer_write_index, frame_buffer);
                        return false;
                    }
                    else
                    {
                        // error: crc mismatch, all data trashed, abort binary mode
                        return false;
                    }
                }
                else
                { // we have more data in this frame
                    // save the new code
                    frame_code = (uint8_t)c;
                    // save the data
                    frame_buffer[frame_buffer_write_index] = COBS_DELIMITER;
                    // recompute crc
                    if (crc) frame_crc16 = crc(frame_crc16, frame_code);
                    // increment code index for next byte
                    frame_code_index = 1;
                    // increment write index for next byte
                    frame_buffer_write_index++;
                }
            }
            else
            { // this byte is data
                frame_buffer[frame_buffer_write_index] = (uint8_t)c;
                if (crc) frame_crc16 = crc((uint8_t)c);
                // increment code index for next byte
                frame_code_index++;
                // increment write index for next byte
                frame_buffer_write_index++;
            }
        }
        return true;
    }
    else
    {   // SOH, reset counters
        frame_code = 0;
        frame_code_index = 0;
        frame_buffer_write_index = 0;
        frame_crc16 = 0;
        return true;
    }
    // abort binary mode
    return false;
}
