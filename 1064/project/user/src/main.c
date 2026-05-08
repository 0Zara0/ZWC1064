#include "zf_device_imu963ra.h"
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_device_oled.h"
#include "zf_common_interrupt.h"
#include "motion_PID.h"
#include "pid_algorithm.h"
#include "move_mode.h"


int main(void)
{
    // ============================================
    // 主函数入口 - 运动控制系统初始化与主循环
    // ============================================
    
    // --------------------------------------------
    // 阶段 1: 硬件底层初始化
    // --------------------------------------------
    
    // 1.1 初始化系统时钟为 600MHz，提供系统运行所需的主频
    clock_init(SYSTEM_CLOCK_600M);
    
    // 1.2 初始化调试模块，配置串口用于 printf 输出和调试信息打印
    debug_init();
    
    // 1.3 延时 50ms，等待电源和时钟信号稳定，确保系统进入稳定工作状态
    system_delay_ms(50);
    
    // 1.4 打印系统启动标识信息，通过串口输出启动提示
    printf("   Motion Control System Starting...\r\n");
    
    // 1.5 初始化传感器子系统：编码器、IMU 惯性测量单元、速度计算器、PID 控制器
    //     调用 MotionPID_Sensor_Init() 完成所有传感器的硬件初始化和参数配置
    printf("[1/4] Initializing sensors...\r\n");
    MotionPID_Sensor_Init();
    printf("        ✓ Sensors initialized successfully\r\n");
    
    // 1.6 初始化电机驱动子系统：配置 PWM 输出通道、GPIO 引脚、电机控制参数
    //     调用 MotionPID_Motor_Init() 完成所有电机的硬件初始化和使能准备
    printf("[2/4] Initializing motors...\r\n");
    MotionPID_Motor_Init();
    printf("        ✓ Motors initialized successfully\r\n");
    
    printf("[3/4] Initializing motion control system...\r\n");
    MoveMode_Init();
    printf("        ✓ Motion control system initialized\r\n");
    
    // 清零所有编码器计数，消除初始化过程中引入的脉冲对后续控制的影响
    encoder_clear_count(QTIMER1_ENCODER1);
    encoder_clear_count(QTIMER1_ENCODER2);
    encoder_clear_count(QTIMER2_ENCODER1);
    encoder_clear_count(QTIMER2_ENCODER2);
    
    // 1.7 开启处理器全局中断允许位，使能所有已配置的中断源响应
    //     此时 PIT 定时器中断和其他已配置中断开始正常工作
    printf("[4/4] Enabling global interrupts...\r\n");
    interrupt_global_enable(0);
    printf("        ✓ Global interrupts enabled\r\n");

    printf("   System Ready! Running...\r\n");
    printf("   Control Mode: CLOSED-LOOP\r\n");

    // 设置小车前进模式，速度为 500 脉冲/秒（增大目标速度以减小低速量化噪声）
    MoveMode_Forward(500.0f);

    // --------------------------------------------
    // 阶段 2: 主循环执行非实时任务
    // --------------------------------------------
    //     主循环负责执行低优先级、非实时的后台任务
    //     实时控制任务由 PIT 中断在后台每 2ms 自动执行，不受主循环影响
    // --------------------------------------------

    uint32 loop_count = 0;  // 主循环计数器，记录循环执行次数

    while (1)
    {
        loop_count++;  // 每次循环计数器加 1

        // 主循环每 100ms 执行一次状态监控和调试信息输出
        // 条件判断：loop_count % 10 == 0 表示每 10 次循环（100ms × 10 = 1 秒）执行一次
        if (loop_count % 10 == 0)  // 每 1 秒（100ms × 10）
        {
            // 输出当前循环计数和系统运行状态标识
            printf("[Loop %d] System running... | Mode: CLOSED-LOOP\r\n",
                   loop_count);

            // 遍历所有 4 个电机，获取每个电机的实时运行参数并输出
            for (uint8 i = 0; i < ENCODER_COUNT; i++)
            {
                // 获取电机 i 的实时运行参数：滤波后的速度值、PID 偏差、PID 输出量
                float speed = EncoderSpeedCalc_GetFilteredSpeed(i);
                float error = VelocityPID_GetError(i);
                float output = VelocityPID_GetOutput(i);

                // 获取 PID 内部详细状态
                VelocityPIDController_t *pid = &g_velocity_pid_controller[i];
                float p_term = pid->Kp * pid->error;
                float i_term = pid->Ki * pid->error_sum;
                float d_term = pid->Kd * pid->error_derivative_filtered;

                // 格式化输出电机 i 的详细状态：速度（脉冲/秒）、误差值、PWM 占空比百分比
                printf("  Motor%d:\r\n", i);
                printf("    Speed=%.2f pulses/s, Target=%.2f, Error=%.2f\r\n",
                       speed, pid->target_speed, error);
                printf("    PID: Kp=%.3f, Ki=%.3f, Kd=%.4f\r\n",
                       pid->Kp, pid->Ki, pid->Kd);
                printf("    PID: P=%.2f, I=%.2f (Sum=%.2f), D=%.2f\r\n",
                       p_term, i_term, pid->error_sum, d_term);
                printf("    PID: Output=%.2f, PWM=%d%%\r\n",
                       output, g_motor_controller.motor[i].speed_percent);
            }


        }

        // 主循环延时 100ms，降低空转时的 CPU 占用率
        // 此延时不影响控制频率：PIT 中断独立运行，保持 2ms 周期的实时控制
        system_delay_ms(100);
    }
}
