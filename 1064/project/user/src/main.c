// ============================================================================
//  调试模式配置（原 debug_mode.h 内容已合并至此）
// ============================================================================
//  MAIN_DEBUG=1   → 调试模式
//    DEBUG_STAGE=1: 传感器检查 (编码器+IMU，电机不转)
//    DEBUG_STAGE=2: 单电机开环测试 (逐个电机 ±20% PWM, 2s)
//    DEBUG_STAGE=3: 四轮开环运动学测试 (前进/后退/左移/右移)
//    DEBUG_STAGE=4: 单电机速度PID闭环 (逐个电机阶跃响应)
//    DEBUG_STAGE=5: 联合运动+航向保持 (前进+平移, 观察Yaw纠偏)
//    DEBUG_STAGE=6: 距离控制 (定距前进/平移, 加减速验证)
//    DEBUG_STAGE=7: 路径执行端到端 (硬编码测试路径)
//    DEBUG_STAGE=8: OpenMV串口通讯调试 (触发-接收-验证-解析)
//    DEBUG_STAGE=9: 路径规划独立调试 (硬编码地图->求解->打印，电机不转)
//  MAIN_DEBUG=0   → 完整工作模式 (推箱子求解 + 路径执行)
// ============================================================================
#define MAIN_DEBUG           1
#define DEBUG_STAGE          6

// 电机旁路模式（用于纯算法调试，不驱动实际电机）
// DEBUG_MOTOR_BYPASS=1: 所有电机控制指令只打印日志，不输出PWM
#define DEBUG_MOTOR_BYPASS   1

// Stage 8: OpenMV串口通讯调试配置
#define DEBUG_OMV_TEST_CYCLES       10
#define DEBUG_OMV_PRINT_RAW_MAP     1
#define DEBUG_OMV_PRINT_ASCII_MAP   1
#define DEBUG_OMV_PRINT_INT_MAP     1

// Stage 9: 路径规划调试配置
#define DEBUG_PATH_USE_BUILTIN_MAP  1
#define DEBUG_PATH_PRINT_SOLVED_MAP 1
#define DEBUG_PATH_MOTOR_BYPASS     1

// ============================================================================
//  推箱子地图参数（与 OpenMV main.py 匹配）
// ============================================================================
#define SOKOMAP_ROW         10u
#define SOKOMAP_COL         14u
#define SOKOBAN_MAP_ROW     10
#define SOKOBAN_MAP_COL     14

// ============================================================================
//  系统级公共头文件
// ============================================================================
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_device_oled.h"
#include "zf_common_interrupt.h"
#include "zf_driver_uart.h"
// ============================================================================
//  运动控制模块头文件（三种模式共用）
// ============================================================================
#include "motion_PID.h"
#include "pid_algorithm.h"
#include "move_mode.h"
#include "dc_motor.h"

// ============================================================================
//  推箱子专用模块（条件编译）
// ============================================================================
#if !MAIN_DEBUG || (MAIN_DEBUG && DEBUG_STAGE >= 8)
#include "soko_map_tools.h"
#include "sokoban_solver.h"
#include "omv_uart.h"
#endif

// ============================================================================
//  调试辅助函数和常量（必须在所有头文件包含之后定义）
// ============================================================================
#if DEBUG_MOTOR_BYPASS
#define MOTOR_BYPASS_LOG_PREFIX "[MOTOR_BYPASS] "

static inline void Debug_MotorBypass_Log(uint8 motor_index, float speed)
{
    const char *mot_name[4] = { "RF", "LF", "RR", "LR" };
    printf("%sMotor %d (%s) -> speed=%+.1f%% (NOT ACTUALLY DRIVEN)\r\n",
           MOTOR_BYPASS_LOG_PREFIX, motor_index, mot_name[motor_index], speed);
}

static inline void Debug_MotorBypass_LogAll(float speed)
{
    printf("%sAll motors -> speed=%+.1f%% (NOT ACTUALLY DRIVEN)\r\n",
           MOTOR_BYPASS_LOG_PREFIX, speed);
}

static inline void Debug_MotorBypass_LogMecanum(float vx, float vy)
{
    printf("%sMecanum -> vx=%+.1f, vy=%+.1f (NOT ACTUALLY DRIVEN)\r\n",
           MOTOR_BYPASS_LOG_PREFIX, vx, vy);
}

static inline void Debug_MotorBypass_LogPathStep(char dir, int32 count)
{
    const char *dir_name = "?";
    switch (dir) {
        case 'W': dir_name = "Forward"; break;
        case 'S': dir_name = "Backward"; break;
        case 'A': dir_name = "StrafeLeft"; break;
        case 'D': dir_name = "StrafeRight"; break;
    }
    printf("%sPath step: %s x %ld (NOT ACTUALLY EXECUTED)\r\n",
           MOTOR_BYPASS_LOG_PREFIX, dir_name, (long)count);
}
#endif

// 内置测试地图（Stage 9 使用）
#if DEBUG_PATH_USE_BUILTIN_MAP
static const uint8 g_debug_builtin_ascii_map[140] = {
    '1','1','1','1','1','1','1','1','1','1','1','1','1','1',
    '1','0','0','0','0','0','0','0','0','0','0','0','0','1',
    '1','0','0','0','0','0','0','0','0','0','0','0','0','1',
    '1','0','0','1','1','1','1','1','1','1','1','0','0','1',
    '1','0','0','1','0','0','0','0','0','0','1','0','0','1',
    '1','0','0','1','0','3','0','0','0','4','1','0','0','1',
    '1','0','0','1','0','0','0','2','0','0','1','0','0','1',
    '1','0','0','1','0','0','0','0','0','0','1','0','0','1',
    '1','0','0','1','1','1','1','1','1','1','1','0','0','1',
    '1','1','1','1','1','1','1','1','1','1','1','1','1','1',
};
#endif

// ============================================================================
//  推箱子全局缓冲区（条件编译）
// ============================================================================
#if !MAIN_DEBUG || (MAIN_DEBUG && DEBUG_STAGE >= 8)
static uint8  s_omv_buffer[OMV_MAP_TOTAL+10];
static uint8  s_ascii_map[SOKOMAP_ARRAY_LEN];
static int    s_num_map[SOKOMAP_ARRAY_LEN];
static int    s_game_map[SOKOMAP_ROW][SOKOMAP_COL];
static char   s_path[SOKOBAN_MAX_STEP];
#endif

// ============================================================================
//  推箱子辅助函数
// ============================================================================
#if !MAIN_DEBUG

static void mark_all_objects(int map[SOKOMAP_ROW][SOKOMAP_COL])
{
    char tag;
    SokoMapStatus status_box, status_goal;

    for (tag = 'A'; tag <= 'Z'; tag++) {
        status_box  = SokoMap_MarkNearest(map, (uint8_t)tag);
        status_goal = SokoMap_MarkNearest(map, (uint8_t)(tag + 32));

        if (status_box != SOKOMAP_OK || status_goal != SOKOMAP_OK) {
            break;
        }
    }
}
#endif

// ============================================================================
//  main() — 系统入口
// ============================================================================
int main(void)
{
	uint8_t dat[10]={0};
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
		//imu963_init_with_calibration(300u,100u);
		imu963_init();
//	uart_init(UART_4,115200, UART4_TX_C16, UART4_RX_C17);
    system_delay_ms(50);
#if MAIN_DEBUG
    printf("   Motion Control Test System - Stage %d\r\n", DEBUG_STAGE);
#else
    printf("   Motion Control Test System\r\n");
#endif

    printf("[1] Initializing sensors...\r\n");
    MotionPID_Sensor_Init();
    printf("    IMU + Encoder initialized\r\n");

    encoder_clear_count(QTIMER2_ENCODER1);
    encoder_clear_count(QTIMER1_ENCODER2);
    encoder_clear_count(QTIMER2_ENCODER2);
    encoder_clear_count(QTIMER1_ENCODER1);
    printf("    Encoders cleared\r\n");

#if MAIN_DEBUG && DEBUG_STAGE >= 2
    printf("[2] Initializing motors...\r\n");
    MotionPID_Motor_Init();
    printf("    4 motors initialized\r\n");
#endif

#if MAIN_DEBUG && DEBUG_STAGE >= 4
    printf("[3] Initializing PIT timer (PID loop)...\r\n");
    MotionPID_Timer_Init();
    printf("[4] Enabling interrupts...\r\n");
    interrupt_global_enable(0);
    printf("    PID loop active (2ms)\r\n");
    system_delay_ms(300);
#endif

// ============================================================================
//  Debug Stages
// ============================================================================
#if MAIN_DEBUG

    // Stage 1: Sensor Check ==================================================
#if DEBUG_STAGE == 1
    {
        uint32 tick = 0;
        int32  last_e0 = 0, last_e1 = 0, last_e2 = 0, last_e3 = 0;

        printf("\r\n========================================\r\n");
        printf("   === Stage 1: Sensor Check (Encoder + IMU) ===\r\n");
        printf("========================================\r\n");
        printf("   [Motor]  All stopped (no PWM output)\r\n");
        printf("   [Encoder] Turn each wheel manually, observe raw count + dir\r\n");
        printf("   [IMU]     Rotate chassis, observe Yaw angle + GyroZ\r\n");
        printf("========================================\r\n\r\n");
        printf("   T(s)     ENC_RF      ENC_LF      ENC_RR      ENC_LR      IMU_Yaw     IMU_GyroZ\r\n");
        printf("   ----     ------      ------      ------      ------      --------    ---------\r\n");

        while (1)
        {
            system_delay_ms(50);
            tick++;
            if (tick % 4 == 0)
            {
                int32 e0 = encoder_get_count(QTIMER2_ENCODER1);
                int32 e1 = encoder_get_count(QTIMER1_ENCODER2);
                int32 e2 = encoder_get_count(QTIMER2_ENCODER2);
                int32 e3 = encoder_get_count(QTIMER1_ENCODER1);
                float yaw   = imu963_get_angle();
                float gyroZ = imu963_data.gyro_z;

                char dir0 = (e0 > last_e0) ? '+' : ((e0 < last_e0) ? '-' : '.');
                char dir1 = (e1 > last_e1) ? '+' : ((e1 < last_e1) ? '-' : '.');
                char dir2 = (e2 > last_e2) ? '+' : ((e2 < last_e2) ? '-' : '.');
                char dir3 = (e3 > last_e3) ? '+' : ((e3 < last_e3) ? '-' : '.');

                printf("  %4.1f   ENC0:%5d%c  ENC1:%5d%c  ENC2:%5d%c  ENC3:%5d%c  IMU_Yaw:%7.1f  IMU_GyroZ:%+8.1f\r\n",
                       tick * 0.05f,
                       e0, dir0, e1, dir1, e2, dir2, e3, dir3,
                       yaw, gyroZ);

                last_e0 = e0; last_e1 = e1; last_e2 = e2; last_e3 = e3;
            }
        }
    }

    // Stage 2: Single Motor Open-Loop ========================================
#elif DEBUG_STAGE == 2
    {
        const encoder_index_enum enc_ch[4] = {
            QTIMER2_ENCODER1, QTIMER1_ENCODER2,
            QTIMER2_ENCODER2, QTIMER1_ENCODER1
        };
        const char *mot_name[4] = { "RF", "LF", "RR", "LR" };

        printf("\r\n========================================\r\n");
        printf("   === Stage 2: Single Motor Open-Loop ===\r\n");
        printf("========================================\r\n");
        printf("   Each motor: +20%% PWM 2s -> stop 1s -> -20%% 2s -> stop 1s\r\n");
        printf("   Expect: +PWM forward, encoder count increases\r\n");
        printf("   Display: raw ENC only (no sign correction)\r\n");
        printf("========================================\r\n");

        for (int m = 0; m < 4; m++)
        {
            printf("\r\n-------- Motor %d (%s) +20%% PWM --------\r\n", m, mot_name[m]);
            printf("  Time    ENC_raw\r\n");
            printf("  ----    -------\r\n");
            encoder_clear_count(enc_ch[m]);
            DCMotor_SetSpeed(&g_motor_controller.motor[m], 20.0f);
            for (int t = 0; t < 10; t++)
            {
                system_delay_ms(200);
                int32 raw = encoder_get_count(enc_ch[m]);
                printf("  %4.1fs   %5d\r\n",
                       (t + 1) * 0.2f, raw);
            }
            DCMotor_SetSpeed(&g_motor_controller.motor[m], 0.0f);
            system_delay_ms(1000);

            printf("\r\n-------- Motor %d (%s) -20%% PWM --------\r\n", m, mot_name[m]);
            printf("  Time    ENC_raw\r\n");
            printf("  ----    -------\r\n");
            encoder_clear_count(enc_ch[m]);
            DCMotor_SetSpeed(&g_motor_controller.motor[m], -20.0f);
            for (int t = 0; t < 10; t++)
            {
                system_delay_ms(200);
                int32 raw = encoder_get_count(enc_ch[m]);
                printf("  %4.1fs   %5d\r\n",
                       (t + 1) * 0.2f, raw);
            }
            DCMotor_SetSpeed(&g_motor_controller.motor[m], 0.0f);
            system_delay_ms(1000);
        }

        printf("\r\n=== Stage 2 Complete ===\r\n");
        printf("Verify: +PWM -> note ENC direction for each motor\r\n");
        while (1) { system_delay_ms(1000); }
    }

    // Stage 3: Four-Wheel Open-Loop Kinematics ===============================
#elif DEBUG_STAGE == 3
    {
        const encoder_index_enum enc_ch[4] = {
            QTIMER2_ENCODER1, QTIMER1_ENCODER2,
            QTIMER2_ENCODER2, QTIMER1_ENCODER1
        };
        const char *mode_name[] = { "Forward", "Backward", "Strafe Left", "Strafe Right" };

        printf("\r\n========================================\r\n");
        printf("   === Stage 3: Four-Wheel Open-Loop ===\r\n");
        printf("========================================\r\n");
        printf("   Each mode: 30%% PWM for 3s, then stop 1.5s\r\n");
        printf("   Observe: chassis moves in expected direction\r\n");
        printf("========================================\r\n");

        int32 speeds_fwd[4]   = {  30,  30,  30,  30 };
        int32 speeds_bwd[4]   = { -30, -30, -30, -30 };
        int32 speeds_left[4]  = {  30, -30, -30,  30 };
        int32 speeds_right[4] = { -30,  30,  30, -30 };
        int32 *speeds_table[4] = { speeds_fwd, speeds_bwd, speeds_left, speeds_right };

        for (int mode = 0; mode < 4; mode++)
        {
            int32 *spd = speeds_table[mode];

            printf("\r\n========================================\r\n");
            printf("  %s  START\r\n", mode_name[mode]);
            printf("  M0(RF):%+3d  M1(LF):%+3d  M2(RR):%+3d  M3(LR):%+3d\r\n",
                   spd[0], spd[1], spd[2], spd[3]);
            printf("========================================\r\n");

            for (int i = 0; i < 4; i++)
                encoder_clear_count(enc_ch[i]);

            for (int i = 0; i < 4; i++)
                DCMotor_SetSpeed(&g_motor_controller.motor[i], (float)spd[i]);

            for (int t = 0; t < 15; t++)
            {
                system_delay_ms(200);
                int32 e0 = encoder_get_count(enc_ch[0]);
                int32 e1 = encoder_get_count(enc_ch[1]);
                int32 e2 = encoder_get_count(enc_ch[2]);
                int32 e3 = encoder_get_count(enc_ch[3]);

                printf("  %.1fs  ENC0:%5d  ENC1:%5d  ENC2:%5d  ENC3:%5d  GyroZ:%+5.1f\r\n",
                       (t + 1) * 0.2f,
                       e0, e1, e2, e3,
                       imu963_data.gyro_z);
            }

            for (int i = 0; i < 4; i++)
                DCMotor_SetSpeed(&g_motor_controller.motor[i], 0.0f);

            printf("----------------------------------------\r\n");
            printf("  %s  STOP (1.5s)\r\n\r\n", mode_name[mode]);
            system_delay_ms(1500);
        }

        printf("========================================\r\n");
        printf("=== Stage 3 Complete ===\r\n");
        printf("========================================\r\n");
        while (1) { system_delay_ms(1000); }
    }

    // Stage 4: Single Motor Velocity PID Closed-Loop =========================
#elif DEBUG_STAGE == 4
    {
        const char *mot_name[4] = { "RF", "LF", "RR", "LR" };

        printf("\r\n========================================\r\n");
        printf("   === Stage 4: Single Motor PID Step Response ===\r\n");
        printf("========================================\r\n");
        printf("   Each motor: target 5000 pps for 3s, then stop 1s\r\n");
        printf("   PIT interrupt drives PID at 2ms; main loop prints @ 100ms\r\n");
        printf("   ERROR counts: forward-motor +ERROR means too slow, more PWM\r\n");
        printf("========================================\r\n");

        for (int m = 0; m < 4; m++)
        {
            printf("\r\n-------- Motor %d (%s) target=5000 pps --------\r\n", m, mot_name[m]);
            printf("  Time     Target   Actual   Error    I-Sum     D-Filt    PWM%%\r\n");
            printf("  ----     ------   ------   -----    -----     ------    ----\r\n");

            for (int i = 0; i < 4; i++)
                g_motor_controller.target_speed[i] = 0.0f;
            VelocityPID_Reset(m);

            g_motor_controller.target_speed[m] = 5000.0f;

            for (int t = 0; t < 30; t++)
            {
                system_delay_ms(100);
                float actual = MotionPID_GetActualSpeed(m);

                printf("  %4.1fs   %5.0f     %5.0f    %+5.0f   %+6.1f   %+6.1f   %+4.0f\r\n",
                       t * 0.1f,
                       5000.0f,
                       actual,
                       g_velocity_pid_controller[m].error,
                       g_velocity_pid_controller[m].error_sum,
                       g_velocity_pid_controller[m].error_derivative_filtered,
                       g_velocity_pid_controller[m].output);
            }

            g_motor_controller.target_speed[m] = 0.0f;
            VelocityPID_Reset(m);
            system_delay_ms(1000);
        }

        printf("\r\n=== Stage 4 Complete ===\r\n");
        while (1) { system_delay_ms(1000); }
    }

    // Stage 5: Combined Motion + Heading Hold ================================
#elif DEBUG_STAGE == 5
    {
        printf("\r\n========================================\r\n");
        printf("   === Stage 5: Combined Motion + Heading Hold ===\r\n");
        printf("========================================\r\n");
        printf("   Test1: Forward 300pps for 5s (heading hold ON)\r\n");
        printf("   Test2: Strafe Left 300pps for 5s (heading hold OFF)\r\n");
        printf("   Test3: Strafe Right 300pps for 5s (heading hold OFF)\r\n");
        printf("========================================\r\n");

        // Test 1: Forward with heading hold
        printf("\r\n--- Forward 300pps, heading hold ON ---\r\n");
        printf("  Time    Yaw_Tgt  Yaw_Cur  Error    AngCorr   Speed0    Speed1    Speed2    Speed3\r\n");
        printf("  ----    -------  -------  -----    -------   ------    ------    ------    ------\r\n");

        MoveMode_SetSpeed(FORWARD, 300.0f);

        for (int t = 0; t < 50; t++)
        {
            system_delay_ms(100);
            float yaw_cur = imu963_get_angle();
            float yaw_err = g_heading_target - yaw_cur;
            if (yaw_err > 180.0f) yaw_err -= 360.0f;
            if (yaw_err < -180.0f) yaw_err += 360.0f;

            printf("  %4.1fs  %6.1f   %6.1f  %+5.1f   %+6.1f   %5.0f      %5.0f      %5.0f      %5.0f\r\n",
                   t * 0.1f,
                   g_heading_target, yaw_cur, yaw_err,
                   g_angle_pid_controller.output,
                   EncoderSpeedCalc_GetFilteredSpeed(0),
                   EncoderSpeedCalc_GetFilteredSpeed(1),
                   EncoderSpeedCalc_GetFilteredSpeed(2),
                   EncoderSpeedCalc_GetFilteredSpeed(3));
        }

        MoveMode_SetSpeed(STOP, 0.0f);
        system_delay_ms(1500);

        // Test 2: Strafe Left (no heading hold)
        printf("\r\n--- Strafe Left 300pps ---\r\n");
        printf("  Time    Speed_RF  Speed_LF  Speed_RR  Speed_LR  Yaw  GyroZ\r\n");
        printf("  ----    --------  --------  --------  --------  ---- -----\r\n");

        MoveMode_SetSpeed(STRAFE_LEFT, 300.0f);

        for (int t = 0; t < 50; t++)
        {
            system_delay_ms(100);
            printf("  %4.1fs  %+6.0f    %+6.0f    %+6.0f    %+6.0f    %5.1f %+5.1f\r\n",
                   t * 0.1f,
                   EncoderSpeedCalc_GetFilteredSpeed(0),
                   EncoderSpeedCalc_GetFilteredSpeed(1),
                   EncoderSpeedCalc_GetFilteredSpeed(2),
                   EncoderSpeedCalc_GetFilteredSpeed(3),
                   imu963_get_angle(),
                   imu963_data.gyro_z);
        }

        MoveMode_SetSpeed(STOP, 0.0f);
        system_delay_ms(1500);

        // Test 3: Strafe Right (no heading hold)
        printf("\r\n--- Strafe Right 300pps ---\r\n");
        printf("  Time    Speed_RF  Speed_LF  Speed_RR  Speed_LR  Yaw  GyroZ\r\n");
        printf("  ----    --------  --------  --------  --------  ---- -----\r\n");

        MoveMode_SetSpeed(STRAFE_RIGHT, 300.0f);

        for (int t = 0; t < 50; t++)
        {
            system_delay_ms(100);
            printf("  %4.1fs  %+6.0f    %+6.0f    %+6.0f    %+6.0f    %5.1f %+5.1f\r\n",
                   t * 0.1f,
                   EncoderSpeedCalc_GetFilteredSpeed(0),
                   EncoderSpeedCalc_GetFilteredSpeed(1),
                   EncoderSpeedCalc_GetFilteredSpeed(2),
                   EncoderSpeedCalc_GetFilteredSpeed(3),
                   imu963_get_angle(),
                   imu963_data.gyro_z);
        }

        MoveMode_SetSpeed(STOP, 0.0f);

        printf("\r\n=== Stage 5 Complete ===\r\n");
        while (1) { system_delay_ms(1000); }
    }

    // Stage 6: Distance Control ==============================================
#elif DEBUG_STAGE == 6
    {
        printf("\r\n========================================\r\n");
        printf("   === Stage 6: Distance Control ===\r\n");
        printf("========================================\r\n");
        printf("   Cycle: Forward 1-cell -> StrafeLeft 1-cell ->\r\n");
        printf("          Backward 1-cell -> StrafeRight 1-cell\r\n");
        printf("   4 cycles total (draws a small square)\r\n");
        printf("========================================\r\n");

        for (int cycle = 3; cycle < 4; cycle++)
        {
            printf("\r\n--- Cycle %d: Forward 1 cell ---\r\n", cycle + 1);
            printf("  Time     ENC_avg  Speed_RF  Speed_LF  Speed_RR  Speed_LR\r\n");
            printf("  ----     -------  --------  --------  --------  --------\r\n");

            MoveMode_ForwardDistance(10, MOVE_DEFAULT_SPEED);
            uint32 t = 0;
            while (!MoveMode_IsFinished())
            {
                system_delay_ms(10);
                MoveMode_DistanceUpdate();
                t++;
                float raw;
                raw = EncoderSpeedCalc_GetFilteredSpeed(0); float s0 = raw < 0 ? -raw : raw;
                raw = EncoderSpeedCalc_GetFilteredSpeed(1); float s1 = raw < 0 ? -raw : raw;
                raw = EncoderSpeedCalc_GetFilteredSpeed(2); float s2 = raw < 0 ? -raw : raw;
                raw = EncoderSpeedCalc_GetFilteredSpeed(3); float s3 = raw < 0 ? -raw : raw;
                float avg_enc = (s0 + s1 + s2 + s3) / 4.0f;
                printf("  %4.1fs    %5.0f    %+6.0f     %+6.0f     %+6.0f     %+6.0f\r\n",
                       t * 0.1f, avg_enc,
                       EncoderSpeedCalc_GetFilteredSpeed(0),
                       EncoderSpeedCalc_GetFilteredSpeed(1),
                       EncoderSpeedCalc_GetFilteredSpeed(2),
                       EncoderSpeedCalc_GetFilteredSpeed(3));
            }
            printf("  --- Forward done ---\r\n");
            system_delay_ms(500);

					 MoveMode_BackwardDistance(0, MOVE_DEFAULT_SPEED);
            t = 0;
            while (!MoveMode_IsFinished())
            {
                system_delay_ms(10);
                MoveMode_DistanceUpdate();
                t++;
								float raw;
                raw = EncoderSpeedCalc_GetFilteredSpeed(0); float s0 = raw < 0 ? -raw : raw;
                raw = EncoderSpeedCalc_GetFilteredSpeed(1); float s1 = raw < 0 ? -raw : raw;
                raw = EncoderSpeedCalc_GetFilteredSpeed(2); float s2 = raw < 0 ? -raw : raw;
                raw = EncoderSpeedCalc_GetFilteredSpeed(3); float s3 = raw < 0 ? -raw : raw;
                float avg_enc = (s0 + s1 + s2 + s3) / 4.0f;
                printf("  %4.1fs    %5.0f    %+6.0f     %+6.0f     %+6.0f     %+6.0f\r\n",
                       t * 0.1f, avg_enc,
                       EncoderSpeedCalc_GetFilteredSpeed(0),
                       EncoderSpeedCalc_GetFilteredSpeed(1),
                       EncoderSpeedCalc_GetFilteredSpeed(2),
                       EncoderSpeedCalc_GetFilteredSpeed(3));
            }
            printf("  --- StrafeLeft done ---\r\n");
            system_delay_ms(500);

						
            printf("\r\n--- Cycle %d: StrafeLeft 1 cell ---\r\n", cycle + 1);
            printf("  Time     ENC_avg  Speed_RF  Speed_LF  Speed_RR  Speed_LR\r\n");
            printf("  ----     -------  --------  --------  --------  --------\r\n");

            MoveMode_StrafeLeftDistance(0, MOVE_DEFAULT_SPEED);
            t = 0;
            while (!MoveMode_IsFinished())
            {
                system_delay_ms(10);
                MoveMode_DistanceUpdate();
                t++;
								float raw;
                raw = EncoderSpeedCalc_GetFilteredSpeed(0); float s0 = raw < 0 ? -raw : raw;
                raw = EncoderSpeedCalc_GetFilteredSpeed(1); float s1 = raw < 0 ? -raw : raw;
                raw = EncoderSpeedCalc_GetFilteredSpeed(2); float s2 = raw < 0 ? -raw : raw;
                raw = EncoderSpeedCalc_GetFilteredSpeed(3); float s3 = raw < 0 ? -raw : raw;
                float avg_enc = (s0 + s1 + s2 + s3) / 4.0f;
                printf("  %4.1fs    %5.0f    %+6.0f     %+6.0f     %+6.0f     %+6.0f\r\n",
                       t * 0.1f, avg_enc,
                       EncoderSpeedCalc_GetFilteredSpeed(0),
                       EncoderSpeedCalc_GetFilteredSpeed(1),
                       EncoderSpeedCalc_GetFilteredSpeed(2),
                       EncoderSpeedCalc_GetFilteredSpeed(3));
            }
            printf("  --- StrafeLeft done ---\r\n");
            system_delay_ms(500);
						
						
						MoveMode_StrafeRightDistance(0, MOVE_DEFAULT_SPEED);
            t = 0;
            while (!MoveMode_IsFinished())
            {
                system_delay_ms(10);
                MoveMode_DistanceUpdate();
                t++;
                float raw;
                raw = EncoderSpeedCalc_GetFilteredSpeed(0); float s0 = raw < 0 ? -raw : raw;
                raw = EncoderSpeedCalc_GetFilteredSpeed(1); float s1 = raw < 0 ? -raw : raw;
                raw = EncoderSpeedCalc_GetFilteredSpeed(2); float s2 = raw < 0 ? -raw : raw;
                raw = EncoderSpeedCalc_GetFilteredSpeed(3); float s3 = raw < 0 ? -raw : raw;
                float avg_enc = (s0 + s1 + s2 + s3) / 4.0f;
                printf("  %4.1fs    %5.0f    %+6.0f     %+6.0f     %+6.0f     %+6.0f\r\n",
                       t * 0.1f, avg_enc,
                       EncoderSpeedCalc_GetFilteredSpeed(0),
                       EncoderSpeedCalc_GetFilteredSpeed(1),
                       EncoderSpeedCalc_GetFilteredSpeed(2),
                       EncoderSpeedCalc_GetFilteredSpeed(3));
            }
            printf("  --- Forward done ---\r\n");
            system_delay_ms(500);
        }

        MoveMode_Stop();
        printf("\r\n=== Stage 6 Complete ===\r\n");
        while (1) { system_delay_ms(1000); }
    }

    // Stage 7: Path Execution End-to-End =====================================
#elif DEBUG_STAGE == 7
    {
        printf("\r\n========================================\r\n");
        printf("   === Stage 7: Path Execution E2E ===\r\n");
        printf("========================================\r\n");
        printf("   Test path: \"WWAA\" = Forward 2 cells, StrafeLeft 2 cells\r\n");
        printf("========================================\r\n\r\n");

        printf("Executing path: WWAA at %d pps...\r\n", MOVE_RUNPATH_DEFAULT_SPEED);
       // MoveMode_RunPath("WWAA", MOVE_RUNPATH_DEFAULT_SPEED);
        printf("Path done!\r\n");

       // system_delay_ms(1000);

        printf("\r\nExecuting path: WWDDSSAA (square loop)...\r\n");
        MoveMode_RunPath("AAAAASSSAWWWWAASAWSAAAWWWW", MOVE_RUNPATH_DEFAULT_SPEED);
        printf("Path done!\r\n");

        printf("\r\n=== Stage 7 Complete ===\r\n");
        while (1) { system_delay_ms(1000); }
    }

    // Stage 8: OpenMV UART Communication Debug ===============================
    //
    //  【拆分子测试说明】
    //  子测试8.1: 静默监听 RX - 检测噪声/干扰
    //  子测试8.2: 发送触发+观察 OpenMV 蓝灯 - 确认触发信号到达
    //  子测试8.3: 发送触发+轮询 RX 数据量 - 确认 OpenMV 是否回应
    //  子测试8.4: 发送触发+逐字节 Dump - 观察实际发送内容
    //  子测试8.5: 完整循环测试 - 原始 OMV_TriggerAndReceive
    //
    //  【调试流程】
    //  1. 运行 8.1，确认 RX 线路无噪声
    //  2. 运行 8.2，观察 OpenMV 蓝灯是否闪烁（确认触发到达）
    //  3. 运行 8.3，确认 OpenMV 是否发送数据
    //  4. 运行 8.4，观察实际数据内容
    //  5. 运行 8.5，完整流程测试
    //
#elif DEBUG_STAGE == 8
    {
        uint16 recv_len;
        uint16 i;
        uint8 valid;

        printf("\r\n========================================\r\n");
        printf("   === Stage 8: OMV UART Comm Debug ===\r\n");
        printf("========================================\r\n");
        printf("   UART: %d baud, TX=%d, RX=%d\r\n",
               OMV_UART_BAUD, OMV_UART_TX_PIN, OMV_UART_RX_PIN);
        printf("   Expected: %d bytes/map\r\n", OMV_MAP_TOTAL);
        printf("========================================\r\n\r\n");

        printf("[0/5] Initializing OMV UART...\r\n");
        OMV_UART_Init();
        printf("      OMV UART initialized\r\n\r\n");

        // =====================================================================
        //  子测试 8.1: 静默监听 RX（检测噪声/干扰）
        // =====================================================================
        printf("[1/5] Test 8.1: Snoop RX (listen for noise)...\r\n");
        printf("      Listening for 1000ms...\r\n");
        recv_len = OMV_Test_SnoopRx(s_omv_buffer, OMV_MAP_TOTAL, 1000);
        printf("      Result: %u bytes received (expected: 0)\r\n", recv_len);
        if (recv_len > 0) {
            printf("      [WARN] Noise detected! Dump (hex): ");
            for (i = 0; i < recv_len && i < 16; i++) {
                printf("%02X ", s_omv_buffer[i]);
            }
            printf("\r\n");
        } else {
            printf("      [OK] No noise on RX line\r\n");
        }
        printf("\r\n");
        system_delay_ms(500);

        // =====================================================================
        //  子测试 8.2: 发送触发 + 观察 OpenMV 蓝灯
        // =====================================================================
        printf("[2/5] Test 8.2: Send trigger only...\r\n");
        printf("      Sending trigger byte 0x01...\r\n");
        printf("      [ACTION] Check if OpenMV blue LED blinks!\r\n");
        OMV_Test_TriggerOnly();
        printf("      Trigger sent. Waiting 500ms for OpenMV response...\r\n");
        system_delay_ms(500);
        printf("      [MANUAL] Did blue LED blink? (Y=trigger received, N=not received)\r\n");
        printf("\r\n");
        system_delay_ms(1000);

        // =====================================================================
        //  子测试 8.3: 发送触发 + 接收并打印内容
        // =====================================================================
        printf("[3/5] Test 8.3: Trigger + Receive & Dump...\r\n");
        printf("      Sending trigger and receiving for 2000ms...\r\n");
        recv_len = OMV_Test_TriggerAndSave(s_omv_buffer, OMV_MAP_TOTAL + 10, 2000);
        printf("      Result: %u bytes received (expected %d)\r\n", recv_len, OMV_MAP_TOTAL);

        if (recv_len == 0) {
            printf("      [FAIL] No data from OpenMV!\r\n");
            printf("      Possible causes:\r\n");
            printf("        - OpenMV not powered/running\r\n");
            printf("        - OpenMV UART ID mismatch (check main.py UART_ID)\r\n");
            printf("        - TX/RX wires swapped or disconnected\r\n");
            printf("        - OpenMV main.py not running/crashed\r\n");
        } else if (recv_len < OMV_MAP_TOTAL) {
            printf("      [WARN] Only %u bytes (expected %d)\r\n", recv_len, OMV_MAP_TOTAL);
            printf("      Content (hex): ");
            for (i = 0; i < recv_len; i++) {
                printf("%02X ", s_omv_buffer[i]);
            }
            printf("\r\n");
            printf("      Content (ascii): ");
            for (i = 0; i < recv_len; i++) {
                uint8 ch = s_omv_buffer[i];
                if (ch >= 32 && ch <= 126) {
                    printf("%c", ch);
                } else {
                    printf("[%02X]", ch);
                }
            }
            printf("\r\n");
        } else {
            printf("      [OK] %u bytes received\r\n", recv_len);
        }
        printf("\r\n");
        system_delay_ms(500);

        // =====================================================================
        //  子测试 8.4: 发送触发 + 逐字节 Dump
        // =====================================================================
        printf("[4/5] Test 8.4: Trigger + Byte-by-byte dump...\r\n");
        printf("      Sending trigger and dumping all received bytes...\r\n");
        recv_len = OMV_Test_TriggerAndDump(s_omv_buffer, OMV_MAP_TOTAL + 10, 2000);
        printf("      Result: %u bytes received\r\n", recv_len);

        if (recv_len == 0) {
            printf("      [FAIL] Nothing received!\r\n");
        } else {
            printf("      Hex dump (%u bytes):\r\n      ", recv_len);
            for (i = 0; i < recv_len; i++) {
                printf("%02X ", s_omv_buffer[i]);
                if ((i + 1) % 16 == 0) {
                    printf("\r\n      ");
                }
            }
            printf("\r\n");

            printf("      ASCII dump (%u bytes):\r\n      ", recv_len);
            for (i = 0; i < recv_len; i++) {
                uint8 ch = s_omv_buffer[i];
                if (ch >= 32 && ch <= 126) {
                    printf("%c", ch);
                } else {
                    printf("[%02X]", ch);
                }
            }
            printf("\r\n");

            // 检查是否有 0x0D (CR) 结束符
            uint8 has_cr = 0;
            for (i = 0; i < recv_len; i++) {
                if (s_omv_buffer[i] == 0x0D) {
                    has_cr = 1;
                    printf("      [INFO] Found CR (0x0D) at position %u\r\n", i);
                    break;
                }
            }
            if (!has_cr) {
                printf("      [WARN] No CR (0x0D) found - OpenMV may not send \\r\\n\r\n");
            }
        }
        printf("\r\n");
        system_delay_ms(500);

        // =====================================================================
        //  子测试 8.5: 完整循环测试（原始函数）
        // =====================================================================
        printf("[5/5] Test 8.5: Full cycle test (original function)...\r\n");
        {
            uint16 cycle;
            uint16 success_count = 0;
            uint16 fail_count = 0;

            for (cycle = 0; cycle < DEBUG_OMV_TEST_CYCLES; cycle++)
            {
                printf("  --- Cycle %d/%d ---\r\n", cycle + 1, DEBUG_OMV_TEST_CYCLES);

                recv_len = OMV_TriggerAndReceive(s_omv_buffer, OMV_MAP_TOTAL);

                printf("    Received: %u/%u bytes ", recv_len, OMV_MAP_TOTAL);
                if (recv_len == OMV_MAP_TOTAL) {
                    printf("[OK]\r\n");
                } else {
                    printf("[FAIL]\r\n");
                }

                valid = OMV_MapIsValid(s_omv_buffer, recv_len);
                printf("    Validity: %s\r\n", valid ? "VALID" : "INVALID");

                if (recv_len == OMV_MAP_TOTAL && valid) {
                    success_count++;
                } else {
                    fail_count++;
                }

                if (recv_len > 0) {
                    printf("    Raw (first 16): ");
                    for (i = 0; i < 16 && i < recv_len; i++) {
                        printf("%c", s_omv_buffer[i]);
                    }
                    printf("\r\n");
                }

                printf("\r\n");
                system_delay_ms(200);
            }

            printf("  Summary: Success=%d, Fail=%d, Rate=%d%%\r\n",
                   success_count, fail_count,
                   (success_count * 100) / DEBUG_OMV_TEST_CYCLES);
        }

        printf("========================================\r\n");
        printf("=== Stage 8 Complete ===\r\n");
        printf("[DIAGNOSIS] Check results above to locate the problem:\r\n");
        printf("  8.1 Noise?     -> If >0 bytes, check wiring shielding\r\n");
        printf("  8.2 LED blink? -> If no blink, OpenMV not receiving trigger\r\n");
        printf("  8.3 Data count -> If 0 bytes, OpenMV not responding\r\n");
        printf("  8.4 Hex dump   -> Check if data format matches expected\r\n");
        printf("  8.5 Full test  -> Overall success rate\r\n");
        while (1) { system_delay_ms(1000); }
    }

    // Stage 9: Path Planning Independent Debug ===============================
    //
    //  【工作流覆盖分析】
    //  正式工作流(Normal Mode)的路径规划环节：
    //    while(1) {
    //      [A] OMV_TriggerAndReceive()   从OpenMV接收地图  <-- Stage 9 未覆盖！使用硬编码地图
    //      [B] OMV_MapToAscii()          原始地图转ASCII码  <-- Stage 9 未覆盖！
    //      [C] SokoMap_AsciiToIntArray()  ASCII转Int数组   <-- Stage 9 覆盖
    //      [D] SokoMap_IntArrayToMap()   Int数组转2D地图   <-- Stage 9 覆盖
    //      [E] mark_all_objects()        标记箱子/目标     <-- Stage 9 覆盖
    //      [F] Sokoban_Solve()           求解路径          <-- Stage 9 覆盖
    //      [G] MoveMode_RunPath()        执行路径          <-- Stage 9 覆盖(旁路模式)
    //    }
    //
    //  Stage 9 实际覆盖: [C]~[G]（算法链路完整）
    //  Stage 9 未覆盖: [A][B]（串口接收，在 Stage 8 中测试）
    //  Stage 9 差异点:
    //    - 使用硬编码地图而非从OpenMV接收（避免依赖视觉硬件）
    //    - 电机旁路模式，不实际驱动电机（避免机械运动风险）
    //
    //  【调试建议】
    //  1. Stage 9 用于验证地图解析算法和求解器逻辑的正确性
    //  2. 若 Stage 9 通过但 Normal Mode 失败，问题在串口通讯或OpenMV端
    //  3. 若需测试真实电机响应，设置 DEBUG_MOTOR_BYPASS=0 并确保 Stage <= 7
    //
#elif DEBUG_STAGE == 9
    {
        SokoMapStatus map_status;
        SokobanStatus solve_status;
        uint16 i;

        printf("\r\n========================================\r\n");
        printf("   === Stage 9: Path Planning Debug ===\r\n");
        printf("========================================\r\n");
        printf("   [Coverage] AsciiToInt->IntArrayToMap->Mark->Solve->RunPath\r\n");
        printf("   [Missing]  OMV_TriggerAndReceive, OMV_MapToAscii (Stage 8)\r\n");
        printf("   [Diff]     BUILTIN map (not from OpenMV), MOTOR BYPASS\r\n");
#if DEBUG_MOTOR_BYPASS
        printf("   MOTOR BYPASS: ON (no actual motor movement)\r\n");
#endif
        printf("   Map source: BUILTIN (hardcoded test map)\r\n");
        printf("   Map size: %dx%d\r\n", SOKOMAP_ROW, SOKOMAP_COL);
        printf("========================================\r\n\r\n");

        // [C] 加载硬编码测试地图（替代正式工作流中的[A][B]串口接收环节）
        printf("[1/5] Loading built-in test map...\r\n");
        printf("      [NOTE] Normal Mode uses OMV_TriggerAndReceive() here\r\n");
        for (i = 0; i < SOKOMAP_ARRAY_LEN; i++) {
            s_ascii_map[i] = g_debug_builtin_ascii_map[i];
        }
        printf("      Built-in map loaded (%d bytes)\r\n", SOKOMAP_ARRAY_LEN);

        printf("\r\n  ASCII Map:\r\n");
        for (i = 0; i < SOKOMAP_ROW; i++) {
            uint16 j;
            printf("  Row %2d: ", i);
            for (j = 0; j < SOKOMAP_COL; j++) {
                printf("%c", s_ascii_map[i * SOKOMAP_COL + j]);
            }
            printf("\r\n");
        }

        // [C] ASCII转Int数组（与正式工作流一致）
        printf("\r\n[2/5] Converting ASCII to int array...\r\n");
        map_status = SokoMap_AsciiToIntArray(s_ascii_map, s_num_map);
        printf("      Result: %s\r\n", SokoMap_StatusString(map_status));

        if (map_status != SOKOMAP_OK) {
            printf("ERROR: Map conversion failed, aborting.\r\n");
            while (1) { system_delay_ms(1000); }
        }

        printf("\r\n  Int Array (first 2 rows):\r\n");
        for (i = 0; i < 2; i++) {
            uint16 j;
            printf("  Row %d: ", i);
            for (j = 0; j < SOKOMAP_COL; j++) {
                printf("%d ", s_num_map[i * SOKOMAP_COL + j]);
            }
            printf("\r\n");
        }

        // [D] Int数组转2D地图（与正式工作流一致）
        printf("\r\n[3/5] Converting to 2D game map...\r\n");
        map_status = SokoMap_IntArrayToMap(s_num_map, s_game_map);
        printf("      Result: %s\r\n", SokoMap_StatusString(map_status));

        // [E] 标记箱子和目标（与正式工作流一致）
        printf("\r\n[4/5] Marking boxes and targets...\r\n");
        mark_all_objects(s_game_map);
        printf("      Marking complete\r\n");

#if DEBUG_PATH_PRINT_SOLVED_MAP
        printf("\r\n  Solved Map (with labels):\r\n");
        for (i = 0; i < SOKOMAP_ROW; i++) {
            uint16 j;
            printf("  Row %2d: ", i);
            for (j = 0; j < SOKOMAP_COL; j++) {
                int cell = s_game_map[i][j];
                if (cell >= 'A' && cell <= 'Z') {
                    printf(" %c ", cell);
                } else if (cell >= 'a' && cell <= 'z') {
                    printf(" %c ", cell);
                } else {
                    printf(" %d ", cell);
                }
            }
            printf("\r\n");
        }
#endif

        // [F] 求解路径（与正式工作流一致）
        printf("\r\n[5/5] Solving Sokoban...\r\n");
        solve_status = Sokoban_Solve(s_game_map, s_path, SOKOBAN_MAX_STEP);
        printf("      Solver result: %s\r\n", Sokoban_StatusString(solve_status));

        if (solve_status != SOKOBAN_OK) {
            printf("ERROR: Solver failed.\r\n");
            while (1) { system_delay_ms(1000); }
        }

        printf("\r\n========================================\r\n");
        printf("  Path found: %s\r\n", s_path);
        printf("  Path length: %u steps\r\n", (uint16)strlen(s_path));
        printf("========================================\r\n");

        // [G] 执行路径（正式工作流真实驱动电机，Stage 9 旁路模式只打印）
#if DEBUG_MOTOR_BYPASS
        printf("\r\n[MOTOR BYPASS] Executing path (simulated, no motor movement):\r\n");
        printf("[NOTE] Set DEBUG_MOTOR_BYPASS=0 to test real motor movement\r\n");
#else
        printf("\r\nExecuting path...\r\n");
#endif
        MoveMode_RunPath(s_path, MOVE_RUNPATH_DEFAULT_SPEED);

        printf("\r\n=== Stage 9 Complete ===\r\n");
        printf("[NOTE] Algorithm chain verified. Use Normal Mode for full integration.\r\n");
        while (1) { system_delay_ms(1000); }
    }

#endif

    // ========================================================================
    //  Normal Mode (MAIN_DEBUG=0)
    // ========================================================================
#else

    printf("[2/4] Initializing motors...\r\n");
    MotionPID_Motor_Init();
    MoveMode_Init();
    printf("      4 motors initialized\r\n");

    printf("[3/4] Initializing OMV UART...\r\n");
    OMV_UART_Init();
    printf("      OMV UART initialized (UART4, 115200bps, 14x10 map)\r\n");

    printf("[4/4] Enabling interrupts...\r\n");
    interrupt_global_enable(0);
    printf("      Interrupts enabled\r\n");

    printf("   System ready\r\n");
    printf("   ===== Waiting for map data =====\r\n");

    while (1)
    {
        uint16 recv_len;
        SokoMapStatus map_status;
        SokobanStatus solve_status;

        printf("\r\n--- Requesting map from OpenMV ---\r\n");

        recv_len = OMV_TriggerAndReceive(s_omv_buffer, OMV_MAP_TOTAL);

        if (recv_len != OMV_MAP_TOTAL) {
            printf("WARNING: Received %u/%u bytes, retrying...\r\n",
                   recv_len, OMV_MAP_TOTAL);
            system_delay_ms(500);
            continue;
        }

        printf("Map received: %u bytes\r\n", recv_len);

        if (!OMV_MapIsValid(s_omv_buffer, OMV_MAP_TOTAL)) {
            printf("WARNING: Invalid map data, retrying...\r\n");
            system_delay_ms(500);
            continue;
        }

        OMV_MapToAscii(s_omv_buffer, s_ascii_map, OMV_MAP_TOTAL);

        map_status = SokoMap_AsciiToIntArray(s_ascii_map, s_num_map);
        if (map_status != SOKOMAP_OK) {
            printf("WARNING: ASCII to int conversion: %s\r\n",
                   SokoMap_StatusString(map_status));
        }

        map_status = SokoMap_IntArrayToMap(s_num_map, s_game_map);
        if (map_status != SOKOMAP_OK) {
            printf("ERROR: IntArray to 2D map failed: %s\r\n",
                   SokoMap_StatusString(map_status));
            continue;
        }

        printf("Marking boxes and targets...\r\n");
        mark_all_objects(s_game_map);

        printf("Solving path...\r\n");
        solve_status = Sokoban_Solve(s_game_map, s_path, SOKOBAN_MAX_STEP);

        if (solve_status != SOKOBAN_OK) {
            printf("Solver failed: %s\r\n", Sokoban_StatusString(solve_status));
            system_delay_ms(500);
            continue;
        }

        printf("Path found: %s\r\n", s_path);
        printf("Executing path (%u steps)...\r\n", (uint16)strlen(s_path));

        MoveMode_RunPath(s_path, MOVE_RUNPATH_DEFAULT_SPEED);

        printf("Path execution completed!\r\n");
        system_delay_ms(1000);
    }

#endif
}
