#ifndef PUBLIC_H_
#define PUBLIC_H_

#include "adc.h"
#include "tim.h"
#include "gpio.h"
#include "spi.h"
#include "arm_math.h"

#define                 int8_t                          signed char
#define                 uint8_t                         unsigned char
#define                 int16_t                         short
#define                 uint16_t                        unsigned short
#define                 int32_t                         int
#define                 uint32_t                        unsigned int

#define                 INVT_PRD                        (0.00005f)
#define                 pi                              (3.1415926f)
#define                 tpi                             (6.2831853f)

#define                 MOTOR_POLES                     (50.0f)                     //电机极对数
#define                 MICRO_NUM                       (256)                       //细分设置 2，4，6，8，16，32，64，128，512........ 8细分对应1600步每圈

/*编码器*/
//#define                 ENCODER_PHYSIC_LINE             1024                        //编码器物理线数
#define                 ENCODER_LINE                    (16384)                   //编码器采样线数
#define                 HALF_ENCODER_LINE               (8192)                   //编码器采样线数一半

#define                 CURR_LOOP_FREQ                  20000.0f                       //电流环控制频率

#define                 DEFAULT_DIR                     1                           //默认方向，可改变初始转向
#define                 SPD_UP_DELTA_TIME               5                         //变速阶段速度变化周期(5*0.00005s)

#define                 REF_VOLT                        1.50f                       //参考电压值
//#define                 REF_VOLT_ABANDON_NUM            100                         //参考电压计算舍弃前n个采样数据
//#define                 REF_VOLT_AVG_NUM                100.0f                      //参考电压计算平均值使用的数据量
#define                 CURR_MAX                        5.0f                        //最大相电流(过流阈值)
#define                 VOLT_MIN                        12.0f                       //最低电压
#define                 VOLT_MAX                        36.0f                        //最高电压

#define                 PWM_PRD                         (4250)                      //PWM最大值
#define                 PWM_HALF_PRD                    (PWM_PRD >> 1)
#define                 PWM_HALF_HALF_PRD               (PWM_HALF_PRD >> 1)
#define                 PWM_MIN                         (PWM_PRD * 0.03f)
#define                 PWM_MAX                         (PWM_PRD * 0.97)

#define                 STM32_FLASH_BASE                0x08000000                  //片内Flash起始地址
#define                 FLASH_USER_START_ADDR           0x08010000                  //编码器校准数据起始地址
#define                 FLASH_USER_PARAM_ADDR           0x08018000                  //系统参数起始地址

/*系统状态*/
typedef enum {
    REF = 0,//偏置电压计算
    IDENT,  //参数辨识
    RUN,    //正常运行
    FAULT,  //故障
    STOP    //未使能
}System_Status;

/*故障代码*/
typedef enum {
    NONE = 0,
    REF_ERR,                //采样基准电压错误
    POWER_ERR,              //过压或欠压
    CURR_ERR,               //过流
    ENCODER_ERR,            //编码器掉线
    POSSER_ERR,             //位置偏差过大
    MOTOR_ERR,              //电机缺相
    VDS_ERR                 //自定义
}System_Err;


typedef enum {
    NONE_MODE = 0,
    OPEN_LOOP,                  //开环强拖
    CURR_CLOSE_LOOP,            //电流闭环（力矩模式）
    POS_CURR_CLOSE_LOOP,        //位置-电流环
    SPEED_CURR_CLOSE_LOOP,      //速度-电流环
    POS_SPEED_CURR_CLOSE_LOOP   //位置-速度-电流环
}System_Run_Mode;

/*驱动电机类型(未使用)*/
typedef enum {
    STEP = 0,
    BLDC
}Motor_Type;

/*运动状态指示*/
typedef enum{
    ACCELERATE = 0,     //加速
    UNIFORM,            //匀速
    DECELERATE          //减速
}Motion_State;

typedef enum{
    NORMAL_RUN = 0,                     //正常运行
    LOCK_ROTOR,                         //堵转
    FREE_MOVE                           //自由转动
}Motor_State;

/*细分中间变量结构体*/
typedef struct{
    uint16_t micro_slt;
    float micro_flt;
    float micro_inv;
    float encoder_slt;
    float encoder_flt;
    float encoder_inv;
}Micro_Varible;
extern Micro_Varible micro_varible;


/*系统状态结构体*/
typedef struct{
    Motor_Type motor_type;                      //电机类型
    System_Status   sys_status;                 //系统状态
    System_Err      sys_err_code;               //错误码
    System_Run_Mode run_mode;                   //系统当前运行模式
    System_Run_Mode run_mode_last;              //上次运行模式
    Motor_State motor_state;                    //电机运行状态
    uint8_t enable_flag;                        //使能标志（1 使能  0 失能）
    float mos_temperature;                      //温度
    uint8_t enc_adjust_flag;                    //编码器校准标志
    uint8_t ident_flag;                         //参数辨识标志
    float curr_loop_band;                       //电流环参数整定band(建议400-1200)
    float curr_kp;                              //电流环Kp参数
    float curr_ki;                              //电流环Ki参数
    float power_volt;                           //母线电压
    float power_volt_inv;                       //母线电压倒数
    uint8_t reset_angle_data;                   //是否重置flash内的编码器校准数据（0 不重置  1 重置(运行编码器非线性校准)）
}System_Run_Param;
extern System_Run_Param run_param;

float thetafrac(float theta_t);
float sat(float value, float min, float max);
void Micro_Varible_Init(Micro_Varible *p);
void System_Run_Param_Init(System_Run_Param *p);


#endif

