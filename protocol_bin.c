#include "protocol_bin.h"

bool binary_mode = false;
static uint8_t binary_type = 0;

binary_parser_ptr cmd_rt_parser = NULL;
binary_parser_ptr cmd_ov_parser = NULL;
binary_parser_ptr buffer_parser = NULL;
binary_parser_ptr unknown_binary_type = NULL;

bool protocol_binary_dispatch(char c) {
    if (binary_mode) {
        switch (binary_type) {
            case MSG_TYPE_CMD_RT:
                if (cmd_rt_parser)
                    return cmd_rt_parser(c);
                break;
            case MSG_TYPE_CMD_OV:
                if (cmd_ov_parser)
                    return cmd_ov_parser(c);
                break;
            case MSG_TYPE_BUFFER:
                if (buffer_parser)
                    return buffer_parser(c);
                break;
            case MSG_TYPE_PLUGIN:
                if (unknown_binary_type)
                    return unknown_binary_type(c);
                break;
            default:
                binary_type = 0;
        }
    }
    else
    {
        switch (c) {
            case MSG_TYPE_CMD_RT:
            case MSG_TYPE_CMD_OV:
            case MSG_TYPE_BUFFER:
                binary_type = c;
                return true;
                break;
            default:
                if (unknown_binary_type)
                    return unknown_binary_type(c);
        }
    }
    // abort binary mode
    return false;
}
