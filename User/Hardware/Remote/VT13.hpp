#ifndef __VT13__

#ifdef __cplusplus
extern "C" {
#endif

#define __VT13__

#include "main.h"
#include "CRC8_CRC16.hpp"

#define RC_CH_VALUE_OFFSET ((uint16_t)1024)

typedef PACKED_STRUCT()
{
        PACKED_STRUCT()
        {
                int16_t ch[4];
                uint8_t mode_sw;
                uint8_t stop;
                uint8_t left_button; // fn;
                uint8_t right_button;
                int16_t wheel;
                uint8_t shutter;
        } rc;
        PACKED_STRUCT()
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
                uint8_t middle;
        } mouse;
        PACKED_STRUCT()
        {
                uint16_t v;
        } key;
        uint16_t crc16;
        int online;
} VT13_data_t;

extern void VT13_DataSolve(volatile const uint8_t *VT13_buf, VT13_data_t *rc_ctrl);
extern VT13_data_t VT13_data;


#ifdef __cplusplus
}
#endif
#endif // ! __RC_N2__
