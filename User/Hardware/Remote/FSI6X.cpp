#include "FSI6X.hpp"

FSI6X_data_s FSI6X_data;
void FSI6X_DecodeData(uint8_t *sbus_buf,FSI6X_data_s *FSI6X_data)
{       FSI6X_data->ConnectState = 1;       
        FSI6X_data->CH1 =  (((uint16_t)sbus_buf[1] >> 0 | ((uint16_t)sbus_buf[2] << 8 )) & 0x07FF);//264 1024 1807
        FSI6X_data->CH2 =  (((uint16_t)sbus_buf[2] >> 3 | ((uint16_t)sbus_buf[3] << 5 )) & 0x07FF);//240 1024 1807
        FSI6X_data->CH3 =  (((uint16_t)sbus_buf[3] >> 6 | ((uint16_t)sbus_buf[4] << 2 ) | (int16_t)sbus_buf[5] << 10 ) & 0x07FF);//240 1024 1807
        FSI6X_data->CH4 =  (((uint16_t)sbus_buf[5] >> 1 | ((uint16_t)sbus_buf[6] << 7 )) & 0x07FF);//240  1024 1019 1807
        FSI6X_data->CH5 =  (((int16_t)sbus_buf[6] >> 4 | ((int16_t)sbus_buf[7] << 4 )) & 0x07FF);//240 1807
        FSI6X_data->CH6 =  (((int16_t)sbus_buf[7] >> 7 | ((int16_t)sbus_buf[8] << 1 ) | (int16_t)sbus_buf[9] << 9 ) & 0x07FF);//240 1807
        FSI6X_data->CH7 =  (((int16_t)sbus_buf[9] >> 2 | ((int16_t)sbus_buf[10] << 6 )) & 0x07FF);//240 1024 1807
        FSI6X_data->CH8 =  (((int16_t)sbus_buf[10] >> 5 | ((int16_t)sbus_buf[11] << 3 )) & 0x07FF);//240 1807
        FSI6X_data->CH9 =  (((int16_t)sbus_buf[12] << 0 | ((int16_t)sbus_buf[13] << 8 )) & 0x07FF);//240 1807
        FSI6X_data->CH10 = (((int16_t)sbus_buf[13] >> 3 | ((int16_t)sbus_buf[14] << 5 )) & 0x07FF);//240 1807
        FSI6X_data->CH11 = (((int16_t)sbus_buf[14] >> 6 | ((int16_t)sbus_buf[15] << 2 ) | (int16_t)sbus_buf[16] << 10 ) & 0x07FF);
        FSI6X_data->CH12 = (((int16_t)sbus_buf[16] >> 1 | ((int16_t)sbus_buf[17] << 7 )) & 0x07FF);
        FSI6X_data->CH13 = (((int16_t)sbus_buf[17] >> 4 | ((int16_t)sbus_buf[18] << 4 )) & 0x07FF);
        FSI6X_data->CH14 = (((int16_t)sbus_buf[18] >> 7 | ((int16_t)sbus_buf[19] << 1 ) | (int16_t)sbus_buf[20] << 9 ) & 0x07FF);
        FSI6X_data->CH15 = (((int16_t)sbus_buf[20] >> 2 | ((int16_t)sbus_buf[21] << 6 )) & 0x07FF);
        FSI6X_data->CH16 = (((int16_t)sbus_buf[21] >> 5 | ((int16_t)sbus_buf[22] << 3 )) & 0x07FF);
   

        FSI6X_data->CH1 -= FS_MID;
        FSI6X_data->CH2 -= FS_MID;
        FSI6X_data->CH3 -= FS_MID;
        FSI6X_data->CH4 -= FS_MID;
        FSI6X_data->online = 30;
}