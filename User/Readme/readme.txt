      此程序为步进电机FOC控制程序，包含电机参数辨识(电阻、电感)，电流环参数自整定，FOC三环(位置-速度-电流)控制，编码器非线性校准，位置型梯形加减速(简易版)等内容，
具体实现自己阅读代码，代码有较为详细的注释。阅读代码需要具备一定的C语言基础，最好撸过BLDC的FOC控制。
=========================================================================================================
文件简介
Public文件夹
public		系统公共变量和公共函数
system_init	系统各结构体初始化
system_it		系统中断回调

Peripheral文件夹
current_sample	ADC采样相关 电流，电源电压，热敏电阻采样计算
encoder		编码器相关，位置读取，电角度转换，速度计算
pwm_ctrl		PWM定时器相关初始化配置

Motor文件夹
foc_control	foc核心代码，主要是电流环底层计算，有实现两相的SVPWM调制，但是电压利用率低于SPWM，步进电机建议使用SPWM调制即可
motor_run_mode	电机运行模式控制 三环排列组合，每种模式都单独封装了函数方便理解
motor_task	电机运行任务流程控制 参考注释理解

Algorithm文件夹
motion_control   	运动控制相关算法  	主要是梯形加减速算法和细分脉冲转换算法。
param_identify	参数辨识，编码器校准逻辑控制
pid_control	PID控制器实现
transform		坐标变换相关

Other文件夹
ele_angle_adjust	编码器电角度非线性校准算法，内部有参考链接
my_flash		G4片内Flash操作相关函数

程序调试说明
1.开发板发货前已进行编码器非线性校准，默认运行位置模式自测程序。
2.电感辨识使用高频脉冲法，上电辨识时会有震动，属正常现象。
3.系统状态监测及模式控制都在System_Run_Param结构体内，public.h文件内有相关注释。
4.系统参数初始化函数 void System_Run_Param_Init(System_Run_Param *p)中reset_angle_data字段控制是否运行编码器非线性校准，若需要运行校准，将此字段置1，烧录运行，
运行完成后改为0，再次烧录程序。因为Flash有读写寿命限制，所以建议尽量减少校准次数。
5.系统模式切换说明
      Debug模式将run_param结构体导入到watch窗口，相关变量有run_mode和enable_flag，系统正常运行时enable_flag为1(使能状态)，当需要切换模式时，先将enable_flag置0，再
修改run_mode，然后再将enable_flag置1即可。
6.各模式介绍说明（以下所说的修改是在Debug模式watch窗口实时修改）
OPEN_LOOP			d轴电角度强拖	修改motion_control结构体中的target_speed（rpm）变量可控制转速
CURR_CLOSE_LOOP			电流环		修改motor_control结构体中的iq_ref（A）变量控制力矩大小
POS_CURR_CLOSE_LOOP		位置-电流环	修改position_loop结构体中的position_ref（脉冲数量，整数）可控制位置
SPEED_CURR_CLOSE_LOOP		速度-电流环	修改motion_control结构体中的target_speed（rpm）变量可控制转速，speed_up控制加速度
POS_SPEED_CURR_CLOSE_LOOP	位置-速度-电流环	无需操作，自动运行位置自测程序，有兴趣可阅读相关代码自由发挥
