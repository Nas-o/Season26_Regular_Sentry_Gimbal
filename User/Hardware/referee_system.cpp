/**
 * @file referee_system.c
 * @author Siri (lixirui2017@outlook.com)
 * @brief 裁判系统相关代码
 * @version 0.1
 * @date 2025-03-01
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "referee_system.hpp"
#include "CRC8_CRC16.hpp"

#define REFEREE_FIFO_BUF_LENGTH 1024

// 外部调用裁判系统数据
REFEREE_DATA_t Referee_data;


// 数据缓冲区
fifo_s_t referee_fifo;       // 裁判系统处理fifo
fifo_s_t referee_image_fifo; // 图传链路处理fifo
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
uint8_t referee_image_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;
unpack_data_t referee_image_unpack_obj;

// 内部变量
/* 裁判系统通信帧头 */
frame_header_struct_t referee_receive_header; // 接收数据帧头
frame_header_struct_t referee_send_header;    // 发送数据帧头
/*----------- 赛制相关数据 -----------*/
ext_game_status_t game_status;         // 比赛状态数据(1Hz)
ext_game_result_t game_result;         // 比赛结果数据(比赛结束后发送)
ext_game_robot_HP_t game_robot_HP;     // 机器人血量数据
ext_referee_warning_t referee_warning; // 裁判警告信息
/*----------- 机器人状态数据 -----------*/
ext_robot_status_t robot_status;                 // 机器人性能数据(10Hz，含底盘、电源等状态)
ext_power_heat_data_t power_heat_data;           // 实时功率热量数据(50Hz，含底盘功率、枪口热量等)
ext_hurt_data_t hurt_data;                       // 伤害状态数据(受到攻击后发送)
ext_shoot_data_t shoot_data;                     // 实时射击数据(子弹射速/频率等)
ext_projectile_allowance_t projectile_allowance; // 弹丸剩余发射许可(弹量限制)
ext_rfid_status_t rfid_status;                   // 机器人RFID状态(检查点位状态)
/*----------- 交互数据 -----------*/
ext_robot_interaction_data_t robot_interaction_data; // 机器人交互数据(自定义通信内容)
ext_custom_info_t exchange_data;                     // 自定义UI显示数据(选手端特定位置字符显示)
/*----------- 控制数据 -----------*/


static void referee_struct_init(void)
{
    // 数据帧
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));
    // 赛制相关
    memset(&game_status, 0, sizeof(ext_game_status_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP, 0, sizeof(ext_game_robot_HP_t));
    memset(&referee_warning, 0, sizeof(ext_referee_warning_t));
    // 机器人性能数据 10hz ！attention please！
    memset(&robot_status, 0, sizeof(ext_robot_status_t));
    // 实时功率热量数据 50hz !attention please!
    memset(&power_heat_data, 0, sizeof(ext_power_heat_data_t));
    // 伤害状态数据
    memset(&hurt_data, 0, sizeof(ext_hurt_data_t));
    // 实时射击数据
    memset(&shoot_data, 0, sizeof(ext_shoot_data_t));
    // 允许发弹量
    memset(&projectile_allowance, 0, sizeof(ext_projectile_allowance_t));
    // 机器人RFID状态
    memset(&rfid_status, 0, sizeof(ext_rfid_status_t));
    // 机器人交互数据
    memset(&robot_interaction_data, 0, sizeof(ext_robot_interaction_data_t));
    // 选手端特定位置显示字符
    memset(&exchange_data, 0, sizeof(ext_custom_info_t));
    // 图传链路键鼠数据
    /* memset(&remote_control, 0, sizeof(ext_remote_control_t)); */
}
static void required_Data() //后续可根据需要自行添加
{
		Referee_data.Initial_SPEED = shoot_data.initial_speed; 
		Referee_data.Launching_Frequency = shoot_data.launching_frequency;
		Referee_data.projectile_allowance_17mm =projectile_allowance.projectile_allowance_17mm;
        Referee_data.robot_id=robot_status.robot_id;
		Referee_data.Barrel_Heat = power_heat_data.shooter_17mm_barrel_heat;  		
		Referee_data.CooLing_Value = robot_status.shooter_barrel_cooling_value;		
		Referee_data.Heat_Limit =  robot_status.shooter_barrel_heat_limit;	
		Referee_data.Buffer_Energy = power_heat_data.buffer_energy;
		Referee_data.Chassis_Power_Limit = robot_status.chassis_power_limit; 
		Referee_data.rest_time = game_status.stage_remain_time;
        Referee_data.Outpost_HP = game_robot_HP.ally_outpost_HP;
        Referee_data.Baseeee_HP = game_robot_HP.ally_base_HP;

}
static void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;
    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_status, frame + index, sizeof(ext_game_status_t));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(ext_game_result_t));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP, frame + index, sizeof(ext_game_robot_HP_t));
        }
        break;
        case FIELD_EVENTS_CMD_ID:
        {
            //memcpy(&event_data, frame + index, sizeof(ext_event_data_t));
        }
        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning, frame + index, sizeof(ext_referee_warning_t));
        }
        break;
        case DART_LAUNCH_TIME_CMD_ID:
        {
            //memcpy(&dart_remaining_time, frame + index, sizeof(ext_dart_remaining_time_t));
        }
        break;
        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_status, frame + index, sizeof(ext_robot_status_t));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data, frame + index, sizeof(ext_power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            //memcpy(&robot_pos, frame + index, sizeof(ext_robot_pos_t));
        }
        break;
        case BUFF_MUSK_CMD_ID:
        {
            //memcpy(&robot_buff, frame + index, sizeof(ext_buff_t));
        }
        break;
        /* case AERIAL_ROBOT_ENERGY_CMD_ID:
        {
            //memcpy(&air_support_data, frame + index, sizeof(ext_air_support_data_t));
        }
        break; */
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&hurt_data, frame + index, sizeof(ext_hurt_data_t));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data, frame + index, sizeof(ext_shoot_data_t));
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&projectile_allowance, frame + index, sizeof(ext_projectile_allowance_t));
        }
        break;
        case ROBOT_RFID_STATE_CMD_ID:
        {
            memcpy(&rfid_status, frame + index, sizeof(ext_rfid_status_t));
        }
        break;
        case DART_PLAYER_COMMAND_CMD_ID:
        {
            //memcpy(&dart_client_cmd, frame + index, sizeof(ext_dart_client_cmd_t));
        }
        break;
        case GROUND_ROBOT_POSITION_CMD_ID:
        {
            //memcpy(&ground_robot_position, frame + index, sizeof(ext_ground_robot_position_t));
        }
        break;
        case RADAR_MARKS_PROGRESS_CMD_ID:
        {
            //memcpy(&radar_mark_data, frame + index, sizeof(ext_radar_mark_data_t));
        }
        break;
        case ROBOT_INTERACTION_CMD_ID:
        {
            memcpy(&robot_interaction_data, frame + index, sizeof(ext_robot_interaction_data_t));
        }
        break;
        case CUSTOM_CONTROLLER_AND_BOT_INTERACTION_CMD_ID:
        {
            //memcpy(&custom_robot_data, frame + index, sizeof(ext_custom_robot_data_t));
        }
        break;
        case PLAYER_SIDE_MINIMAP_INTERACTION_CMD_ID:
        {
            //memcpy(&map_command, frame + index, sizeof(ext_map_command_t));
        }
        break;
       /*  case KEYBOARD_AND_MOUSE_REMOTE_CONTROL_CMD_ID:
        {
            memcpy(&remote_control, frame + index, sizeof(ext_remote_control_t));
        }
        break; */
        case PLAYER_MINIMAP_RECEIVES_RADAR_CMD_ID:
        {
            //memcpy(&map_robot_data, frame + index, sizeof(ext_map_robot_data_t));
        }
        break;
        case INTERACTION_DATA_BETWEEN_CONTROLLER_AND_PLAYER_CMD_ID:
        {
            //memcpy(&custom_client_data, frame + index, sizeof(ext_custom_client_data_t));
        }
        break;
        case PLAYER_MINIMAP_RECEIVES_SENTRY_CMD_ID:
        {
            //memcpy(&map_sentry_data, frame + index, sizeof(ext_map_sentry_data_t));
        }
        break;
		case SENTRY_DATA_CMD_ID:
        {
            //memcpy(&sentry_data, frame + index, sizeof(ext_sentry_info_t));
        }
        break;
		case RADAR_DATA_CMD_ID:
        {
            //memcpy(&radar_data, frame + index, sizeof(ext_radar_info_t));
        }
        break;		
		case EXCHANGE_DATA_CMD_ID:
        {
            memcpy(&exchange_data, frame + index, sizeof(ext_custom_info_t));
        }
        break;		

        default:
        {
            break;
        }
    }
}
/**
 * @brief 初始化裁判系统
 *
 */
void Refree_system_init(void)
{
    referee_struct_init();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    fifo_s_init(&referee_image_fifo, referee_image_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
}

/**
 * @brief 裁判系统解包
 * 
 * @param p_fifo 缓冲fifo
 * @param p_obj 解包流程
 */
void Referee_unpack_fifo_data(fifo_s_t* p_fifo,unpack_data_t* p_obj)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;

  while (fifo_s_used(p_fifo))
  {
    byte = fifo_s_get(p_fifo);
    switch (p_obj->unpack_step)
    {
    case STEP_HEADER_SOF:
    {
      if (byte == sof)
      {
        p_obj->unpack_step = STEP_LENGTH_LOW;
        p_obj->protocol_packet[p_obj->index++] = byte;
      }
      else
      {
        p_obj->index = 0;
      }
    }
    break;

    case STEP_LENGTH_LOW:
    {
      p_obj->data_len = byte;
      p_obj->protocol_packet[p_obj->index++] = byte;
      p_obj->unpack_step = STEP_LENGTH_HIGH;
    }
    break;

    case STEP_LENGTH_HIGH:
    {
      p_obj->data_len |= (byte << 8);
      p_obj->protocol_packet[p_obj->index++] = byte;

      if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
      {
        p_obj->unpack_step = STEP_FRAME_SEQ;
      }
      else
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }
    }
    break;
    case STEP_FRAME_SEQ:
    {
      p_obj->protocol_packet[p_obj->index++] = byte;
      p_obj->unpack_step = STEP_HEADER_CRC8;
    }
    break;

    case STEP_HEADER_CRC8:
    {
      p_obj->protocol_packet[p_obj->index++] = byte;

      if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
      {
        if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
        {
          p_obj->unpack_step = STEP_DATA_CRC16;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }
    }
    break;

    case STEP_DATA_CRC16:
    {
      if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
      }
      if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;

        if (verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          referee_data_solve(p_obj->protocol_packet);
        }
      }
    }
    break;

    default:
    {
      p_obj->unpack_step = STEP_HEADER_SOF;
      p_obj->index = 0;
    }
    break;
    }
  }
  required_Data();
}
