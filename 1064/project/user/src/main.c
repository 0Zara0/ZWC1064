// ============================================================================
//  运行模式选择
// ============================================================================
//  MAIN_DEBUG=1   → 调试模式
//    DEBUG_STAGE=1: 传感器检查 (编码器+IMU，电机不转)
//    DEBUG_STAGE=2: 单电机开环测试 (逐个电机 ±20% PWM, 2s)
//    DEBUG_STAGE=3: 四轮开环运动学测试 (前进/后退/左移/右移)
//    DEBUG_STAGE=4: 单电机速度PID闭环 (逐个电机阶跃响应)
//    DEBUG_STAGE=5: 联合运动+航向保持 (前进+平移, 观察Yaw纠偏)
//    DEBUG_STAGE=6: 距离控制 (定距前进/平移, 加减速验证)
//    DEBUG_STAGE=7: 路径执行端到端 (硬编码测试路径)
//  MAIN_DEBUG=0   → 完整工作模式 (推箱子求解 + 路径执行)
// ============================================================================
#define MAIN_DEBUG           1
#define DEBUG_STAGE          4

// ============================================================================
//  推箱子地图参数（仅在正常模式下生效）
// ============================================================================
#define SOKOMAP_ROW         12u
#define SOKOMAP_COL         16u
#define SOKOBAN_MAP_ROW     12
#define SOKOBAN_MAP_COL     16

// ============================================================================
//  系统级公共头文件
// ============================================================================
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_device_oled.h"
#include "zf_common_interrupt.h"

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
#if !MAIN_DEBUG
#include "soko_map_tools.h"
#include "sokoban_solver.h"
#include "omv_uart.h"
#endif

// ============================================================================
//  推箱子全局缓冲区（条件编译）
// ============================================================================
#if !MAIN_DEBUG
static uint8  s_omv_buffer[OMV_MAP_TOTAL];
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
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    system_delay_ms(50);

#if MAIN_DEBUG
    printf("   Motion Control Test System - Stage %d\r\n", DEBUG_STAGE);
#else
    printf("   Motion Control Test System\r\n");
#endif

    printf("[1] Initializing sensors...\r\n");
    MotionPID_Sensor_Init();
    printf("    IMU + Encoder initialized\r\n");

    encoder_clear_count(QTIMER1_ENCODER1);
    encoder_clear_count(QTIMER1_ENCODER2);
    encoder_clear_count(QTIMER2_ENCODER1);
    encoder_clear_count(QTIMER2_ENCODER2);
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
        printf("   [Encoder] Turn each wheel manually, observe count + dir\r\n");
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
                int32 e0 = -encoder_get_count(QTIMER1_ENCODER1);
                int32 e1 =  encoder_get_count(QTIMER1_ENCODER2);
                int32 e2 = -encoder_get_count(QTIMER2_ENCODER1);
                int32 e3 =  encoder_get_count(QTIMER2_ENCODER2);
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
            QTIMER1_ENCODER1, QTIMER1_ENCODER2,
            QTIMER2_ENCODER1, QTIMER2_ENCODER2
        };
        const int32 enc_sign[4] = { -1, 1, -1, 1 };  // ENC0/2 reversed vs motor forward
        const char *mot_name[4] = { "RF", "LF", "RR", "LR" };

        printf("\r\n========================================\r\n");
        printf("   === Stage 2: Single Motor Open-Loop ===\r\n");
        printf("========================================\r\n");
        printf("   Each motor: +20%% PWM 2s -> stop 1s -> -20%% 2s -> stop 1s\r\n");
        printf("   Expect: +PWM forward, encoder count increases\r\n");
        printf("   Display: raw ENC + signed ENC (ENC0/2 reversed)\r\n");
        printf("========================================\r\n");

        for (int m = 0; m < 4; m++)
        {
            printf("\r\n-------- Motor %d (%s) +20%% PWM --------\r\n", m, mot_name[m]);
            printf("  Time    ENC_raw  ENC_signed\r\n");
            printf("  ----    -------  ----------\r\n");
            encoder_clear_count(enc_ch[m]);
            DCMotor_SetSpeed(&g_motor_controller.motor[m], 20.0f);
            for (int t = 0; t < 10; t++)
            {
                system_delay_ms(200);
                int32 raw = encoder_get_count(enc_ch[m]);
                printf("  %4.1fs   %5d     %5d\r\n",
                       (t + 1) * 0.2f, raw, raw * enc_sign[m]);
            }
            DCMotor_SetSpeed(&g_motor_controller.motor[m], 0.0f);
            system_delay_ms(1000);

            printf("\r\n-------- Motor %d (%s) -20%% PWM --------\r\n", m, mot_name[m]);
            printf("  Time    ENC_raw  ENC_signed\r\n");
            printf("  ----    -------  ----------\r\n");
            encoder_clear_count(enc_ch[m]);
            DCMotor_SetSpeed(&g_motor_controller.motor[m], -20.0f);
            for (int t = 0; t < 10; t++)
            {
                system_delay_ms(200);
                int32 raw = encoder_get_count(enc_ch[m]);
                printf("  %4.1fs   %5d     %5d\r\n",
                       (t + 1) * 0.2f, raw, raw * enc_sign[m]);
            }
            DCMotor_SetSpeed(&g_motor_controller.motor[m], 0.0f);
            system_delay_ms(1000);
        }

        printf("\r\n=== Stage 2 Complete ===\r\n");
        printf("Verify: +PWM -> ENC_signed positive for all 4 motors\r\n");
        while (1) { system_delay_ms(1000); }
    }

    // Stage 3: Four-Wheel Open-Loop Kinematics ===============================
#elif DEBUG_STAGE == 3
    {
        const encoder_index_enum enc_ch[4] = {
            QTIMER1_ENCODER1, QTIMER1_ENCODER2,
            QTIMER2_ENCODER1, QTIMER2_ENCODER2
        };
        const int32 enc_sign[4] = { -1, 1, -1, 1 };
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

                printf("  %.1fs  ENC0:%5d(%+d)  ENC1:%5d(%+d)  ENC2:%5d(%+d)  ENC3:%5d(%+d)  GyroZ:%+5.1f\r\n",
                       (t + 1) * 0.2f,
                       e0, e0 * enc_sign[0],
                       e1, e1 * enc_sign[1],
                       e2, e2 * enc_sign[2],
                       e3, e3 * enc_sign[3],
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
                float actual = EncoderSpeedCalc_GetFilteredSpeed(m);
                if (g_motor_controller.motor[m].dir_inverted)
                {
                    actual = -actual;
                }

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

        for (int cycle = 0; cycle < 4; cycle++)
        {
            printf("\r\n--- Cycle %d: Forward 1 cell ---\r\n", cycle + 1);
            printf("  Time     ENC_avg  Speed_RF  Speed_LF  Speed_RR  Speed_LR\r\n");
            printf("  ----     -------  --------  --------  --------  --------\r\n");

            MoveMode_ForwardDistance(1, MOVE_DEFAULT_SPEED);
            uint32 t = 0;
            while (!MoveMode_IsFinished())
            {
                system_delay_ms(100);
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

            printf("\r\n--- Cycle %d: StrafeLeft 1 cell ---\r\n", cycle + 1);
            printf("  Time     ENC_avg  Speed_RF  Speed_LF  Speed_RR  Speed_LR\r\n");
            printf("  ----     -------  --------  --------  --------  --------\r\n");

            MoveMode_StrafeLeftDistance(1, MOVE_DEFAULT_SPEED);
            t = 0;
            while (!MoveMode_IsFinished())
            {
                system_delay_ms(100);
                MoveMode_DistanceUpdate();
                t++;
                raw = EncoderSpeedCalc_GetFilteredSpeed(0); s0 = raw < 0 ? -raw : raw;
                raw = EncoderSpeedCalc_GetFilteredSpeed(1); s1 = raw < 0 ? -raw : raw;
                raw = EncoderSpeedCalc_GetFilteredSpeed(2); s2 = raw < 0 ? -raw : raw;
                raw = EncoderSpeedCalc_GetFilteredSpeed(3); s3 = raw < 0 ? -raw : raw;
                avg_enc = (s0 + s1 + s2 + s3) / 4.0f;
                printf("  %4.1fs    %5.0f    %+6.0f     %+6.0f     %+6.0f     %+6.0f\r\n",
                       t * 0.1f, avg_enc,
                       EncoderSpeedCalc_GetFilteredSpeed(0),
                       EncoderSpeedCalc_GetFilteredSpeed(1),
                       EncoderSpeedCalc_GetFilteredSpeed(2),
                       EncoderSpeedCalc_GetFilteredSpeed(3));
            }
            printf("  --- StrafeLeft done ---\r\n");
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
        MoveMode_RunPath("WWAA", MOVE_RUNPATH_DEFAULT_SPEED);
        printf("Path done!\r\n");

        system_delay_ms(1000);

        printf("\r\nExecuting path: WWDDSSAA (square loop)...\r\n");
        MoveMode_RunPath("WWDDSSAA", MOVE_RUNPATH_DEFAULT_SPEED);
        printf("Path done!\r\n");

        printf("\r\n=== Stage 7 Complete ===\r\n");
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
    printf("      OMV UART initialized (UART2, 19200bps)\r\n");

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
