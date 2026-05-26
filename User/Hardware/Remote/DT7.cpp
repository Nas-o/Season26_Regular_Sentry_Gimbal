#include "DT7.hpp"

DT7_data_s DT7_data;

void DT7_DecodeData(uint8_t *sbus_buf)
{
    DT7_data.rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    DT7_data.rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    DT7_data.rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &
                        0x07ff;
    DT7_data.rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    DT7_data.rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                       //!< Switch left
    DT7_data.rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  //!< Switch right
    DT7_data.mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    DT7_data.mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    DT7_data.mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    DT7_data.mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    DT7_data.mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    DT7_data.key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    DT7_data.rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 // NULL

    DT7_data.rc.ch[0] -= RC_CH_VALUE_OFFSET;
    DT7_data.rc.ch[1] -= RC_CH_VALUE_OFFSET;
    DT7_data.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    DT7_data.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    DT7_data.rc.ch[4] -= RC_CH_VALUE_OFFSET;
    DT7_data.online=100;
}