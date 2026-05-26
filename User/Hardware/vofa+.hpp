/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2026-01-05 22:25:23
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-01-05 22:25:29
 * @FilePath: \Regular_Sentry_Gimbal\User\Hardware\vofa+.h
 */
#ifndef   _VOFA_H_

#ifdef __cplusplus
extern "C" {
#endif

#define   _VOFA_H_

void UploadData_vofa(float data1,float data2,float data3,float data4);
void Ausart_printf(const char *fmt, ...);


#ifdef __cplusplus
}
#endif
#endif
