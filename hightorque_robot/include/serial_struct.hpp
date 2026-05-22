#ifndef _SERIAL_STRUCT_H_
#define _SERIAL_STRUCT_H_
#include <stdint.h>
#include "crc16.hpp"
#include "crc8.hpp"

#define ROS_INFO(format, ...)  printf(format "\n", ##__VA_ARGS__)
#define ROS_ERROR(format, ...) printf("\033[1;31m" format "\033[0m\n", ##__VA_ARGS__)
#define ROS_DEBUG_STREAM(x)


#define  CDC_TR_MESSAGE_DATA_LEN    256

#define  MODE_POSITION              0X80
#define  MODE_VELOCITY              0X81
#define  MODE_TORQUE                0X82
#define  MODE_VOLTAGE               0X83
#define  MODE_CURRENT               0X84
#define  MODE_TIME_OUT              0x85

#define  MODE_POS_VEL_TQE           0X90
// #define  MODE_POS_VEL_TQE_KP_KD     0X93  // Deprecated
// #define  MODE_POS_VEL_TQE_KP_KI_KD  0X98  // Deprecated
#define  MODE_POS_VEL_KP_KD         0X9E
// #define  MODE_POS_VEL_TQE_RKP_RKD   0XA3  // Deprecated
// #define  MODE_POS_VEL_RKP_RKD       0XA8  // Deprecated
#define  MODE_POS_VEL_ACC           0XAD
#define  MODE_POS_VEL_TQE_KP_KD_2    0XB0


#define  MODE_NULL                  0X00  // undefined
#define  MODE_RESET_ZERO            0X01  // reset motor zero position
#define  MODE_CONF_WRITE            0X02  // save configuration
#define  MODE_STOP                  0X03  // motor stop
#define  MODE_BRAKE                 0X04  // motor brake
#define  MODE_SET_NUM               0X05  // set number of motors per channel and query firmware version
#define  MODE_MOTOR_STATE           0X06  // motor state
// #define  MODE_CONF_LOAD             0X07  // restore configuration (deprecated)
#define  MODE_RESET                 0X08  // motor reboot
// #define  MODE_RUNZERO               0X09  // run to zero on power-up
#define  MODE_MOTOR_STATE2          0X0A  // motor state 2 (with mode and fault code)
#define  MODE_MOTOR_VERSION         0X0B  // motor version
#define  MODE_FUN_V                 0X0C  // set feature version
#define  MODE_BOOTLOADER            0X0D  // upgrade communication board firmware
#define  MODE_FDCAN_RESET           0X0E  // reinitialize FDCAN
#define  MODE_FDCAN_MOTOR_STATE     0X0F  // communication board FDCAN error codes + motor state
#define  MODE_TQE_ADJS_FLAG         0X10  // motor internal torque adjustment (supported since motor firmware v4.6.0)
#define  MODE_FDCAN_MOTOR_STATE2    0X11  // communication board FDCAN error codes + motor state 2 (with mode and fault code)

#define COMBINE_VERSION(major, minor, patch) (((major) << 12) | ((minor) << 4) | (patch))
#define GET_MAJOR_VERSION(version) (((version) >> 12) & 0xF)  // extract major version
#define GET_MINOR_VERSION(version) (((version) >> 4) & 0xFF)  // extract minor version
#define GET_PATCH_VERSION(version) ((version) & 0xF)          // extract patch version

typedef enum 
{
    fun_v1 = 1,  // v3
    fun_v2,      // v4: motor mode and fault codes
    fun_v3,      // v4: optional 5-parameter motors non-responsive
    fun_v4,
    fun_v5,
    fun_v6,
    fun_v7,
    fun_v8,
    fun_v9,

    fun_vz = 99,
} fun_version;

typedef enum __attribute__((packed))
{
    FDCAN_STATUS_UNKNOWN = -1,    // invalid status
    FDCAN_STATUS_OK,             // OK
    FDCAN_STATUS_ERROR_WARNING,  // error warning -- bit errors, CRC, ACK, format errors (recoverable)
    FDCAN_STATUS_ERROR_PASSIVE,  // error passive -- not transmitting, but can receive
    FDCAN_STATUS_BUS_OFF,        // bus off -- not transmitting or receiving any data
} __attribute__((packed)) fdcan_fault_t;

#pragma pack(1)
typedef struct 
{
    int16_t pos;
    int16_t vel;
    int16_t tqe;
} motor_pos_vel_tqe_s;

typedef struct 
{
    int16_t pos;
    int16_t vel;
    int16_t tqe;
    int16_t kp;
    int16_t kd;
} motor_pos_vel_tqe_kp_kd_s;

typedef struct 
{
    int16_t pos;
    int16_t vel;
    int16_t kp;
    int16_t kd;
} motor_pos_vel_kp_kd_s;

typedef struct 
{
    int16_t pos;
    int16_t vel;
    int16_t acc;
} motor_pos_vel_acc_s;

typedef struct 
{
    uint8_t id;
    int16_t pos;
    int16_t vel;
    int16_t tqe;
} cdc_rx_motor_state_s;

typedef struct
{
    uint8_t id;
    uint8_t mode;
    uint8_t fault;
    int16_t pos;
    int16_t vel;
    int16_t tqe;
} cdc_rx_motor_state2_s;

typedef struct 
{
    uint8_t id;
    uint8_t major;
    uint8_t minor; 
    uint8_t patch;
} cdc_rx_motor_version_s;

typedef struct 
{
    fdcan_fault_t fault;  // fault code
    uint8_t tx_err_num;    // transmit error count
    uint8_t rx_err_num;    // receive error count
} cdc_rx_fdcan_state_s;

typedef struct
{
    uint8_t id;
    uint8_t flag;
} cdc_rx_motor_flag_s;

typedef struct 
{
    cdc_rx_fdcan_state_s fdcan_state;
    union 
    {
        cdc_rx_motor_state_s motor_state[(CDC_TR_MESSAGE_DATA_LEN - sizeof(cdc_rx_fdcan_state_s)) / sizeof(cdc_rx_motor_state_s)];
        cdc_rx_motor_state2_s motor_state2[(CDC_TR_MESSAGE_DATA_LEN - sizeof(cdc_rx_fdcan_state_s)) / sizeof(cdc_rx_motor_state2_s)];
    };
} cdc_rx_fdcan_motor_state_s;

typedef struct 
{
    uint8_t head;
    uint8_t cmd;
    uint16_t len;
    uint8_t  crc8;
    uint16_t crc16;
} cdc_tr_message_head_data_s;


typedef struct
{
    union 
    {
        cdc_tr_message_head_data_s s;
        uint8_t data[sizeof(cdc_tr_message_head_data_s)];
    };
} cdc_tr_message_head_s;


typedef struct
{
    union
    {
        int16_t position[CDC_TR_MESSAGE_DATA_LEN / sizeof(int16_t)];
        int16_t velocity[CDC_TR_MESSAGE_DATA_LEN / sizeof(int16_t)];
        int16_t torque[CDC_TR_MESSAGE_DATA_LEN / sizeof(int16_t)];
        int16_t voltage[CDC_TR_MESSAGE_DATA_LEN / sizeof(int16_t)];
        int16_t current[CDC_TR_MESSAGE_DATA_LEN / sizeof(int16_t)];
        int16_t timeout[CDC_TR_MESSAGE_DATA_LEN / sizeof(int16_t)];
        motor_pos_vel_tqe_s pos_vel_tqe[CDC_TR_MESSAGE_DATA_LEN / sizeof(motor_pos_vel_tqe_s)];
        motor_pos_vel_tqe_kp_kd_s pos_vel_tqe_kp_kd[CDC_TR_MESSAGE_DATA_LEN / sizeof(motor_pos_vel_tqe_kp_kd_s)];
        motor_pos_vel_kp_kd_s pos_vel_kp_kd[CDC_TR_MESSAGE_DATA_LEN / sizeof(motor_pos_vel_kp_kd_s)];
        motor_pos_vel_acc_s pos_vel_acc[CDC_TR_MESSAGE_DATA_LEN / sizeof(motor_pos_vel_acc_s)];
        cdc_rx_motor_state_s motor_state[CDC_TR_MESSAGE_DATA_LEN / sizeof(cdc_rx_motor_state_s)];
        cdc_rx_motor_state2_s motor_state2[CDC_TR_MESSAGE_DATA_LEN / sizeof(cdc_rx_motor_state2_s)];
        cdc_rx_motor_version_s motor_version[CDC_TR_MESSAGE_DATA_LEN / sizeof(cdc_rx_motor_version_s)];
        cdc_rx_motor_flag_s motor_flag[CDC_TR_MESSAGE_DATA_LEN / sizeof(cdc_rx_motor_flag_s)];
        cdc_rx_fdcan_motor_state_s fdcan_motor_state;
        uint8_t data[CDC_TR_MESSAGE_DATA_LEN];
        int16_t data16[CDC_TR_MESSAGE_DATA_LEN / sizeof(int16_t)];
    };
} cdc_tr_message_data_s;


typedef struct 
{
    cdc_tr_message_head_s head;
    cdc_tr_message_data_s data;
} cdc_tr_message_s;


typedef struct motor_back_struct
{
    double time;
    uint8_t ID;
    uint8_t mode;
    uint8_t fault;
    float position;
    float velocity;
    float torque;
    int num;
} motor_back_t;

#pragma pack()

#define INFO_G(format, ...) ROS_INFO("\033[1;32m" format "\033[0m", ##__VA_ARGS__)

#endif