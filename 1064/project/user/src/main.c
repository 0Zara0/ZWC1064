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
    
    // 1.7 检测编码器连接状态，自动选择开环或闭环模式
    //     采用电机微动脉冲检测法：对每个电机施加极短时间、极低占空比的驱动信号，
    //     主动产生少量编码器脉冲。检测完成后立即停止电机并清零编码器计数，
    //     确保检测过程不影响后续的正常数据采集和控制精度。
    printf("[3/5] Checking encoder connection (active pulse test)...\r\n");
    system_delay_ms(100);  // 等待传感器稳定
    
    uint8 encoder_connected_count = 0;  // 记录已连接的编码器数量
    
    for (uint8 i = 0; i < ENCODER_COUNT; i++)
    {
        int16 initial_value, final_value;
        encoder_index_enum enc_idx;
        
        // 获取对应编码器索引
        switch(i)
        {
            case 0: enc_idx = QTIMER1_ENCODER1; break;
            case 1: enc_idx = QTIMER1_ENCODER2; break;
            case 2: enc_idx = QTIMER2_ENCODER1; break;
            case 3: enc_idx = QTIMER2_ENCODER2; break;
            default: continue;
        }
        
        // 记录编码器初始计数值
        initial_value = encoder_get_count(enc_idx);
        
        // 施加微动脉冲驱动：5%占空比正向驱动 100ms
        // 此驱动强度足以让编码器产生数个脉冲，但不足以让机器人产生明显位移
        DCMotor_SetSpeed(&g_motor_controller.motor[i], 5);
        system_delay_ms(100);
        DCMotor_SetSpeed(&g_motor_controller.motor[i], 0);  // 立即停止电机
        
        // 延时 20ms 等待编码器信号稳定
        system_delay_ms(20);
        
        // 读取编码器最终计数值
        final_value = encoder_get_count(enc_idx);
        
        // 判断编码器响应：计数值发生变化说明编码器已正确连接并工作
        if (final_value != initial_value)
        {
            encoder_connected_count++;
            printf("        ✓ Encoder%d connected (pulse response: %d -> %d, delta=%d)\r\n",
                   i, initial_value, final_value, (int16)(final_value - initial_value));
        }
        else
        {
            printf("        ⚠ Encoder%d NOT connected or faulty (no pulse response, value=%d)\r\n",
                   i, final_value);
        }
        
        system_delay_ms(50);  // 电机检测间隔，避免连续驱动
    }
    
    // 清零所有编码器计数，消除检测过程中引入的脉冲对后续控制的影响
    encoder_clear_count(QTIMER1_ENCODER1);
    encoder_clear_count(QTIMER1_ENCODER2);
    encoder_clear_count(QTIMER2_ENCODER1);
    encoder_clear_count(QTIMER2_ENCODER2);
    
    // 根据检测结果选择控制模式
    printf("        Detected %d/%d encoders connected\r\n", encoder_connected_count, ENCODER_COUNT);
    
    if (encoder_connected_count == 0)
    {
        printf("        → Switching to OPEN-LOOP mode (no encoder detected)\r\n");
        MotionPID_SetOpenLoopMode(1);
    }
    else
    {
        printf("        → Using CLOSED-LOOP mode (%d encoder(s) detected)\r\n", encoder_connected_count);
        MotionPID_SetOpenLoopMode(0);
    }
    
    // 1.8 初始化运动控制系统（封装了传感器、电机和定时器的初始化）
    printf("[4/5] Initializing motion control system...\r\n");
    MoveMode_Init();
    printf("        ✓ Motion control system initialized\r\n");
    
    // 1.9 开启处理器全局中断允许位，使能所有已配置的中断源响应
    //     此时 PIT 定时器中断和其他已配置中断开始正常工作
    printf("[5/5] Enabling global interrupts...\r\n");
    interrupt_global_enable(0);
    printf("        ✓ Global interrupts enabled\r\n");

    printf("   System Ready! Running...\r\n");
    printf("   Control Mode: %s\r\n", MotionPID_GetOpenLoopMode() ? "OPEN-LOOP" : "CLOSED-LOOP");
    
    // 设置小车前进模式，速度为 40 脉冲/秒
    MoveMode_Forward(40.0f);
    
    // --------------------------------------------
    // 阶段 2: 主循环执行非实时任务
    // --------------------------------------------
    //     主循环负责执行低优先级、非实时的后台任务
    //     实时控制任务由 PIT 中断在后台每 5ms 自动执行，不受主循环影响
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
            printf("[Loop %d] System running... | Mode: %s\r\n", 
                   loop_count, MotionPID_GetOpenLoopMode() ? "OPEN-LOOP" : "CLOSED-LOOP");
            
            // 遍历所有 4 个电机，获取每个电机的实时运行参数并输出
            for (uint8 i = 0; i < ENCODER_COUNT; i++)
            {
                // 获取电机 i 的实时运行参数：滤波后的速度值、PID 偏差、PID 输出量
                float speed = EncoderSpeedCalc_GetFilteredSpeed(i);
                float error = VelocityPID_GetError(i);
                float output = VelocityPID_GetOutput(i);
                
                // 格式化输出电机 i 的详细状态：速度（脉冲/秒）、误差值、PWM 占空比百分比
                printf("  Motor%d: Speed=%.2f pulses/s, Error=%.2f, PWM=%d%%\r\n", 
                       i, speed, error, g_motor_controller.motor[i].speed_percent);
            }
            
            // 动态目标速度调整接口
            // 如需运行时修改运动模式或速度，取消下面注释即可
            // MoveMode_Backward(50.0f);      // 改为后退，速度 50 脉冲/秒
            // MoveMode_StrafeLeft(80.0f);    // 改为左移，速度 80 脉冲/秒
            // MoveMode_Stop();               // 停止
        }
        
        // 主循环延时 100ms，降低空转时的 CPU 占用率
        // 此延时不影响控制频率：PIT 中断独立运行，保持 5ms 周期的实时控制
        system_delay_ms(100);
    }
}

