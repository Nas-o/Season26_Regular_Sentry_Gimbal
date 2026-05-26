
#include "dm_motor_drv.hpp"
#include "CAN_receive_send.hpp"
#include "User_math.hpp"

#define abs(a) a > 0 ? a : -a

/**
************************************************************************
* @brief:      	dm4310_enable: 坯用DM4310电机控制模弝函数
* @param[in]:   hcan:    指坑CAN_HandleTypeDef结构的指�?
* @param[in]:   motor:   指坑DM_motor_t结构的指针，包坫电机相关信杯和控制坂�?
* @retval:     	void
* @details:    	根杮电机控制模弝坯用相应的模弝，通过CAN总线坑逝坯用命�?
*               支挝的控制模弝包括佝置模弝〝佝置速度控制模弝和速度控制模弝
************************************************************************
**/
void dm_motor_enable(FDCAN_HandleTypeDef* hcan, DM_motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case mit_mode:
			enable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case pos_mode:
			enable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case spd_mode:
			enable_motor_mode(hcan, motor->id, SPD_MODE);
			break;
		case psi_mode:
			enable_motor_mode(hcan, motor->id, PSI_MODE);
			break;
	}	
}
/**
************************************************************************
* @brief:      	dm4310_disable: 禝用DM4310电机控制模弝函数
* @param[in]:   hcan:    指坑CAN_HandleTypeDef结构的指�?
* @param[in]:   motor:   指坑DM_motor_t结构的指针，包坫电机相关信杯和控制坂�?
* @retval:     	void
* @details:    	根杮电机控制模弝禝用相应的模弝，通过CAN总线坑逝禝用命�?
*               支挝的控制模弝包括佝置模弝〝佝置速度控制模弝和速度控制模弝
************************************************************************
**/
void dm_motor_disable(FDCAN_HandleTypeDef* hcan, DM_motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case mit_mode:
			disable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case pos_mode:
			disable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case spd_mode:
			disable_motor_mode(hcan, motor->id, SPD_MODE);
			break;
		case psi_mode:
			disable_motor_mode(hcan, motor->id, PSI_MODE);
			break;
	}	
	dm_motor_clear_para(motor);
}
/**
************************************************************************
* @brief:      	dm4310_ctrl_send: 坑逝DM4310电机控制命令函数
* @param[in]:   hcan:    指坑CAN_HandleTypeDef结构的指�?
* @param[in]:   motor:   指坑DM_motor_t结构的指针，包坫电机相关信杯和控制坂�?
* @retval:     	void
* @details:    	根杮电机控制模弝坑逝相应的命令到DM4310电机
*               支挝的控制模弝包括佝置模弝〝佝置速度控制模弝和速度控制模弝
************************************************************************
**/
void dm_motor_ctrl_send(FDCAN_HandleTypeDef* hcan, DM_motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case mit_mode:
			mit_ctrl(hcan, motor, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
			break;
		case pos_mode:
			pos_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
			break;
		case spd_mode:
			spd_ctrl(hcan, motor->id, motor->ctrl.vel_set);
			break;
		case psi_mode:
			psi_ctrl(hcan, motor->id,motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.cur_set);
			break;
	}	
}

/**
************************************************************************
* @brief:      	dm4310_clear: 清除DM4310电机控制坂数函数
* @param[in]:   motor:   指坑DM_motor_t结构的指针，包坫电机相关信杯和控制坂�?
* @retval:     	void
* @details:    	将DM4310电机的命令坂数和控制坂数清零，包括佝置〝速度�?
*               比例增益(KP)〝微分增�?(KD)和扭�?
************************************************************************
**/
void dm_motor_clear_para(DM_motor_t *motor)
{
	motor->ctrl.kd_set 	= 0;
	motor->ctrl.kp_set	= 0;
	motor->ctrl.pos_set = 0;
	motor->ctrl.vel_set = 0;
	motor->ctrl.tor_set = 0;
	motor->ctrl.cur_set = 0;
}
/**
************************************************************************
* @brief:      	dm4310_clear_err: 清除DM4310电机错误函数
* @param[in]:   hcan: 	 指坑CAN控制结构体的指针
* @param[in]:  	motor:   指坑电机结构体的指针
* @retval:     	void
* @details:    	根杮电机的控制模弝，调用对应模弝的清除错误函�?
************************************************************************
**/
void dm_motor_clear_err(FDCAN_HandleTypeDef* hcan, DM_motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case mit_mode:
			clear_err(hcan, motor->id, MIT_MODE);
			break;
		case pos_mode:
			clear_err(hcan, motor->id, POS_MODE);
			break;
		case spd_mode:
			clear_err(hcan, motor->id, SPD_MODE);
			break;
		case psi_mode:
			clear_err(hcan, motor->id, PSI_MODE);
			break;
	}	
}
/**
************************************************************************
* @brief:      	dm4310_fbdata: 获坖DM4310电机坝馈数杮函数
* @param[in]:   motor:    指坑DM_motor_t结构的指针，包坫电机相关信杯和坝馈数�?
* @param[in]:   rx_data:  指坑包坫坝馈数杮的数组指�?
* @retval:     	void
* @details:    	从接收到的数杮中杝坖DM4310电机的坝馈信杯，包括电机ID�?
*               状思〝佝置〝速度〝扭矩以坊相关温度坂�?
************************************************************************
**/
void dm_motor_fbdata(DM_motor_t *motor, uint8_t *rx_data)
{
	motor->para.id = (rx_data[0])&0x0F;
	motor->para.state = (rx_data[0])>>4;
	motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->para.last_pos = motor->para.pos;
	motor->para.pos = uint_to_float(motor->para.p_int, -motor->tmp.PMAX, motor->tmp.PMAX, 16); // (-12.5,12.5)
	motor->para.vel = uint_to_float(motor->para.v_int, -motor->tmp.VMAX, motor->tmp.VMAX, 12); // (-45.0,45.0)
	motor->para.tor = uint_to_float(motor->para.t_int, -motor->tmp.TMAX, motor->tmp.TMAX, 12); // (-18.0,18.0)
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);

	// process the data
    // count cnt
    if (motor->para.last_pos > 12 && motor->para.pos < -12)
        motor->para.pos_cnt += ((P_MAX - motor->para.last_pos) + (motor->para.pos + P_MAX));
    else if (motor->para.last_pos < -12 && motor->para.pos > 12)
        motor->para.pos_cnt -= ((P_MAX - motor->para.pos) + motor->para.last_pos + P_MAX);
    else
        motor->para.pos_cnt += (motor->para.pos - motor->para.last_pos);
    // process data
     motor->para.angle_cnt = motor->para.pos_cnt * POS_TO_ANGLE;
    
        // �������ת��
        //(ptr)->round_speed = (ptr)->speed_rpm;

    // ������ԽǶ� -180~180 �������ȶ�ʧ �ܽǶȹ���ʱ
        float angle = motor->para.angle_cnt - motor->angle_zero;
        uint32_t mul = abs((int)angle) / 180;
        if (angle > 180.0f)
        {
            if (mul % 2 == 1) // ����-180��
                angle -= (mul + 1) * 180;
            else // ����180��
                angle -= mul * 180;
        }
        if (angle < -180.0f)
        {
            if (mul % 2 == 1) // ����180��
                angle += (mul + 1) * 180;
            else // ����-180��
                angle += mul * 180;
        }
        motor->para.angle = angle;
}

/**
************************************************************************
* @brief:      	enable_motor_mode: 坯用电机模弝函数
* @param[in]:   hcan:     指坑CAN_HandleTypeDef结构的指�?
* @param[in]:   motor_id: 电机ID，指定目标电�?
* @param[in]:   mode_id:  模弝ID，指定覝开坯的模弝
* @retval:     	void
* @details:    	通过CAN总线坑特定电机坑逝坯用特定模弝的命令
************************************************************************
**/
void enable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	Fdcanx_SendData(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	disable_motor_mode: 禝用电机模弝函数
* @param[in]:   hcan:     指坑CAN_HandleTypeDef结构的指�?
* @param[in]:   motor_id: 电机ID，指定目标电�?
* @param[in]:   mode_id:  模弝ID，指定覝禝用的模�?
* @retval:     	void
* @details:    	通过CAN总线坑特定电机坑逝禝用特定模弝的命令
************************************************************************
**/
void disable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	Fdcanx_SendData(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	save_pos_zero: 保存佝置零点函数
* @param[in]:   hcan:     指坑CAN_HandleTypeDef结构的指�?
* @param[in]:   motor_id: 电机ID，指定目标电�?
* @param[in]:   mode_id:  模弝ID，指定覝保存佝置零点的模�?
* @retval:     	void
* @details:    	通过CAN总线坑特定电机坑逝保存佝置零点的命令
************************************************************************
**/
void save_pos_zero(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFE;
	
	Fdcanx_SendData(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	clear_err: 清除电机错误函数
* @param[in]:   hcan:     指坑CAN_HandleTypeDef结构的指�?
* @param[in]:   motor_id: 电机ID，指定目标电�?
* @param[in]:   mode_id:  模弝ID，指定覝清除错误的模�?
* @retval:     	void
* @details:    	通过CAN总线坑特定电机坑逝清除错误的命令�?
************************************************************************
**/
void clear_err(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFB;
	
	Fdcanx_SendData(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	mit_ctrl: MIT模弝下的电机控制函数
* @param[in]:   hcan:			指坑CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电�?
* @param[in]:   pos:			佝置给定�?
* @param[in]:   vel:			速度给定�?
* @param[in]:   kp:				佝置比例系数
* @param[in]:   kd:				佝置微分系数
* @param[in]:   torq:			转矩给定�?
* @retval:     	void
* @details:    	通过CAN总线坑电机坑逝MIT模弝下的控制帧�?
************************************************************************
**/
void mit_ctrl(FDCAN_HandleTypeDef* hcan, DM_motor_t *motor, uint16_t motor_id, float pos, float vel,float kp, float kd, float tor)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos, -motor->tmp.PMAX, motor->tmp.PMAX, 16);
	vel_tmp = float_to_uint(vel, -motor->tmp.VMAX, motor->tmp.VMAX, 12);
	tor_tmp = float_to_uint(tor, -motor->tmp.TMAX, motor->tmp.TMAX, 12);
	kp_tmp  = float_to_uint(kp,  KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,  KD_MIN, KD_MAX, 12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	Fdcanx_SendData(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	pos_speed_ctrl: 佝置速度控制函数
* @param[in]:   hcan:			指坑CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电�?
* @param[in]:   vel:			速度给定�?
* @retval:     	void
* @details:    	通过CAN总线坑电机坑逝佝置速度控制命令
************************************************************************
**/
void pos_ctrl(FDCAN_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor_id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	Fdcanx_SendData(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	speed_ctrl: 速度控制函数
* @param[in]:   hcan: 		指坑CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id: 电机ID，指定目标电�?
* @param[in]:   vel: 			速度给定�?
* @retval:     	void
* @details:    	通过CAN总线坑电机坑逝速度控制命令
************************************************************************
**/
void spd_ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + SPD_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	Fdcanx_SendData(hcan, id, data, 4);
}

/**
************************************************************************
* @brief:      	pos_speed_ctrl: 混控模弝
* @param[in]:   hcan:			指坑CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电�?
* @param[in]:   pos:			佝置给定�?
* @param[in]:   vel:			速度给定�?
* @param[in]:   i:				电浝给定�?
* @retval:     	void
* @details:    	通过CAN总线坑电机坑逝佝置速度控制命令
************************************************************************
**/
void psi_ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel, float cur)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf, *ibuf;
	uint8_t data[8];
	
	uint16_t u16_vel = vel*100;
	uint16_t u16_cur  = cur*10000;
	
	id = motor_id + PSI_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&u16_vel;
	ibuf=(uint8_t*)&u16_cur;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	
	data[6] = *ibuf;
	data[7] = *(ibuf+1);
	
	Fdcanx_SendData(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	read_motor_data: 坑逝读坖寄存器命令
* @param[in]:   id:    电机can id
* @param[in]:   rid:   寄存器地址
* @retval:     	void
* @details:    	读坖电机坂数
************************************************************************
**/
void read_motor_data(uint16_t id, uint8_t rid) 
{
	uint8_t can_id_l = id & 0x0F;
	uint8_t can_id_h = (id >> 4) & 0x0F;
	
	uint8_t data[4] = {can_id_l, can_id_h, 0x33, rid};
	Fdcanx_SendData(&hfdcan1, 0x7FF, data, 4);
}
/**
************************************************************************
* @brief:      	read_motor_ctrl_fbdata: 坑逝读坖电机坝馈数杮的命令
* @param[in]:   id:    电机can id
* @retval:     	void
* @details:    	读坖电机控制坝馈的数�?
************************************************************************
**/
void read_motor_ctrl_fbdata(uint16_t id) 
{
	uint8_t can_id_l = id & 0xFF;       // �? 8 �?
    uint8_t can_id_h = (id >> 8) & 0x07; // �? 3 �?

	uint8_t data[4] = {can_id_l, can_id_h, 0xCC, 0x00};
	Fdcanx_SendData(&hfdcan1, 0x7FF, data, 4);
}
/**
************************************************************************
* @brief:      	write_motor_data: 坑逝写寄存器命�?
* @param[in]:   id:    电机can id
* @param[in]:   rid:   寄存器地址
* @param[in]:   d0-d3: 写入的数�?
* @retval:     	void
* @details:    	坑寄存器写入数杮
************************************************************************
**/
void write_motor_data(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
	uint8_t can_id_l = id & 0xFF;       // �? 8 �?
    uint8_t can_id_h = (id >> 8) & 0x07; // �? 3 �?
	
	uint8_t data[8] = {can_id_l, can_id_h, 0x55, rid, d0, d1, d2, d3};
	Fdcanx_SendData(&hfdcan1, 0x7FF, data, 8);
}
/**
************************************************************************
* @brief:      	save_motor_data: 坑逝保存命�?
* @param[in]:   id:    电机can id
* @param[in]:   rid:   寄存器地址
* @retval:     	void
* @details:    	保存写入的电机坂�?
************************************************************************
**/
void save_motor_data(uint16_t id, uint8_t rid) 
{
	uint8_t can_id_l = id & 0xFF;       // �? 8 �?
    uint8_t can_id_h = (id >> 8) & 0x07; // �? 3 �?
	
	uint8_t data[4] = {can_id_l, can_id_h, 0xAA, 0x01};
	Fdcanx_SendData(&hfdcan1, 0x7FF, data, 4);
}

