#include "Imu.h"
#include "main.h"
#include "usart.h"
#include "Motor.h"
#include "PID.h"
#include "OLED.h"
#include <math.h>
#include <string.h>

/* ==================================================================
 * 参数集中在这里，标注"可调"的是实车调试时可能要动的。
 * ================================================================== */

/* ---- 串口 / 协议 ---- */
#define IMU_FRAME_LEN         11U      /* 0x55 + TYPE + 8 数据 + SUM */
#define IMU_TIMEOUT_MS        300U     /* 超过该时间无有效帧判为离线 */

/* ---- 上电与标定 ---- */
#define IMU_BOOT_WAIT_MS      1200U    /* 模块启动时间 1000ms + 余量 */
#define IMU_CALIB_MS          900U     /* 静止采样时长 */
#define IMU_CALIB_MAX         120U     /* 静止采样最大帧数 */
#define IMU_STILL_STD_DEG     0.5f     /* 判静止的 yaw 标准差门限 */

/* ---- 滤波 ---- */
#define IMU_YAW_LP_ALPHA      0.25f    /* 显示用一阶低通，控制不用 */
#define IMU_GZ_LP_ALPHA       0.35f

/* ---- 定角度控制（可调） ---- */
#define IMU_TURN_TOL_DEG      1.5f     /* 到位死区，可调 */
#define IMU_TURN_RATE_TOL     20.0f    /* 到位时角速度门限 °/s */
#define IMU_TURN_OMEGA_MAX    180.0f   /* 最高转向角速度 °/s，可调 */
#define IMU_TURN_OMEGA_MIN    18.0f    /* 最小转向角速度，克服静摩擦，可调 */
#define IMU_TURN_KP           2.5f     /* 比例系数 1/s，可调 */
#define IMU_TURN_A_DECEL      300.0f   /* 制动减速度 °/s^2，可调 */
#define IMU_TURN_TIMEOUT_MS   3000U    /* 单遍超时（90° 约 0.6s，3s 很宽裕） */
#define IMU_TURN_SETTLE_MS    150U     /* 停车后静置复测时间 */
#define IMU_TURN_PASSES       3U       /* 最多修正几遍 */
#define IMU_TURN_STABLE_CNT   3U       /* 连续几拍满足死区才算到位 */

/* ==================================================================
 * 模块内部状态
 * ================================================================== */

/*
 * 串口字节在中断里直接解析，不再经过环形缓冲。
 * 100Hz x 3 帧 = 3300 B/s，每字节解析只有几十条指令，
 * 占用远小于 1%。这样主循环再慢（比如刷 OLED）也不会丢帧。
 */
static volatile uint8_t  imu_rx_byte = 0U;

/* 最简自检用：累计收到的原始字节数（不管帧对不对） */
static volatile uint32_t imu_raw_bytes = 0U;

static uint8_t imu_frame[IMU_FRAME_LEN];
static uint8_t imu_frame_index   = 0U;
static uint8_t imu_frame_syncing = 1U;

static float   imu_yaw_last_raw = 0.0f;
static float   imu_yaw_cont     = 0.0f;
static uint8_t imu_yaw_inited   = 0U;
static float   imu_gz_filtered  = 0.0f;

/*
 * 自己积分 GZ 得到的航向（未乘符号、未减零点的原始累计值）。
 * 模块倒装时欧拉角解算不可靠，这条路径只依赖角速度，响应是线性的。
 */
static float    imu_yaw_int      = 0.0f;
static uint32_t imu_last_gz_tick = 0U;

/*
 * 软件零点：置零时记录"当时的车辆航向"，之后都减掉它。
 * 不依赖模块自己的 CALSW 是否生效，模块丢配置也不会让零点漂掉。
 */
static float   imu_zero_ref = 0.0f;

/* 航向保持：暂停截止时刻（HAL_GetTick 时基） */
static uint32_t imu_hold_pause_until = 0U;

/* 失控保护：记下窗口起点的误差，看它在窗口内有没有变小（仅供完整版使用） */
uint32_t imu_hold_fault_tick = 0U;
float    imu_hold_fault_err  = 0.0f;

/*
 * 上电自动布防标志。
 * 1 = 还没拿到第一帧，主循环一旦发现 online 就立刻"定 0° + 开始保持"。
 * 这样模块比 MCU 晚上电、或者第一次没等到数据，也能自己接上。
 */
static uint8_t imu_boot_arm = 1U;

static volatile uint32_t imu_angle_frames = 0U;

Imu_Data_t  imu_data;
Imu_Calib_t imu_calib;
Imu_State_t imu_state;

/* ==================================================================
 * 内部小工具
 * ================================================================== */

/* 把角度折到 -180~+180，用 fmodf 保证常数时间 */
static float Imu_Wrap180(float angle)
{
    angle = fmodf(angle + 180.0f, 360.0f);
    if (angle < 0.0f)
    {
        angle += 360.0f;
    }
    return angle - 180.0f;
}

/* 急停：目标速度和当前 PWM 一起清零，和 Function.c 的 Scene_StopCar 一致 */
static void Imu_StopCar(void)
{
    Set_Target_Speed(1U, 0.0f);
    Set_Target_Speed(2U, 0.0f);
    Set_Target_Speed(3U, 0.0f);
    Set_Target_Speed(4U, 0.0f);
    Set_Speed(1, 0);
    Set_Speed(2, 0);
    Set_Speed(3, 0);
    Set_Speed(4, 0);
    imu_state.last_omega = 0.0f;
}

/*
 * 拼 16 位有符号数：模块低字节先发。
 * 必须先合成 uint16 再转 int16，直接对高位做算术右移会把负数算错。
 */
static int16_t Imu_GetInt16(const uint8_t *low_first)
{
    return (int16_t)((uint16_t)low_first[0] | ((uint16_t)low_first[1] << 8));
}

/*
 * 航向展开：模块只给 -180~+180，转两圈就跳变。
 * 每帧增量限制在 +-180 内累加，得到可以一直增大的连续角。
 */
static void Imu_UpdateYaw(float raw_yaw)
{
    float delta;

    if (imu_yaw_inited == 0U)
    {
        imu_yaw_cont     = raw_yaw;
        imu_yaw_last_raw = raw_yaw;
        imu_yaw_inited     = 1U;
    }
    else
    {
        delta = raw_yaw - imu_yaw_last_raw;
        if (delta > 180.0f)
        {
            delta -= 360.0f;
        }
        else if (delta < -180.0f)
        {
            delta += 360.0f;
        }

        imu_yaw_last_raw = raw_yaw;
        imu_yaw_cont    += delta;
    }

#if (IMU_YAW_FROM_GYRO == 0)
    /* 用模块欧拉角做航向时，这里才刷新对外航向；用陀螺积分时在 0x52 里刷新 */
    imu_state.yaw = ((float)imu_state.yaw_sign * imu_yaw_cont) - imu_zero_ref;
    imu_state.yaw_disp += IMU_YAW_LP_ALPHA * (imu_state.yaw - imu_state.yaw_disp);
    imu_state.rate = (float)imu_state.yaw_sign * (imu_gz_filtered - imu_calib.gz_bias);
#endif
}

static void Imu_HandleFrame(const uint8_t *frame)
{
    uint8_t sum = 0U;
    uint8_t i;

    /* SUM = (0x55 + TYPE + 8 个数据字节) 的低 8 位 */
    for (i = 0U; i < (IMU_FRAME_LEN - 1U); i++)
    {
        sum = (uint8_t)(sum + frame[i]);
    }
    if (sum != frame[IMU_FRAME_LEN - 1U])
    {
        imu_data.sum_errors++;
        return;
    }

    switch (frame[1])
    {
    case 0x51U: /* 加速度 + 温度 */
        imu_data.ax   = (float)Imu_GetInt16(&frame[2]) / 32768.0f * 16.0f;
        imu_data.ay   = (float)Imu_GetInt16(&frame[4]) / 32768.0f * 16.0f;
        imu_data.az   = (float)Imu_GetInt16(&frame[6]) / 32768.0f * 16.0f;
        imu_data.temp = (float)Imu_GetInt16(&frame[8]) / 100.0f;
        break;

    case 0x52U: /* 角速度 */
        imu_data.gx = (float)Imu_GetInt16(&frame[2]) / 32768.0f * 2000.0f;
        imu_data.gy = (float)Imu_GetInt16(&frame[4]) / 32768.0f * 2000.0f;
        imu_data.gz = (float)Imu_GetInt16(&frame[6]) / 32768.0f * 2000.0f;
        imu_gz_filtered += IMU_GZ_LP_ALPHA * (imu_data.gz - imu_gz_filtered);

#if (IMU_YAW_FROM_GYRO != 0)
        /*
         * 自己积分 GZ 得到航向，完全不用模块的欧拉角。
         * 模块倒装时它的姿态解算本身就不对（平放却报 Roll=-173/Pitch=36），
         * 欧拉 yaw 会在某一段角度里不敏感；角速度不会。
         */
        {
            uint32_t tk = HAL_GetTick();
            float dt = 0.0f;

            if (imu_last_gz_tick != 0U)
            {
                dt = (float)(uint32_t)(tk - imu_last_gz_tick) / 1000.0f;
            }
            imu_last_gz_tick = tk;

            /* 间隔异常（比如中断被长时间屏蔽）就丢掉这一段，避免一次性积飞 */
            if (dt > 0.05f)
            {
                dt = 0.05f;
            }
            if (dt > 0.0f)
            {
                imu_yaw_int += imu_data.gz * dt;
            }

            imu_state.yaw = ((float)imu_state.yaw_sign * imu_yaw_int) - imu_zero_ref;
            imu_state.yaw_disp += IMU_YAW_LP_ALPHA * (imu_state.yaw - imu_state.yaw_disp);
            imu_state.rate = (float)imu_state.yaw_sign * (imu_gz_filtered - imu_calib.gz_bias);
        }
#endif
        break;

    case 0x53U: /* 角度 */
        imu_data.roll  = (float)Imu_GetInt16(&frame[2]) / 32768.0f * 180.0f;
        imu_data.pitch = (float)Imu_GetInt16(&frame[4]) / 32768.0f * 180.0f;
        imu_data.yaw   = (float)Imu_GetInt16(&frame[6]) / 32768.0f * 180.0f;
        Imu_UpdateYaw(imu_data.yaw);
        imu_angle_frames++;
        break;

    default:
        return;
    }

    imu_data.frames++;
    imu_data.last_tick = HAL_GetTick();
}

static void Imu_ParseByte(uint8_t byte)
{
    if (imu_frame_syncing != 0U)
    {
        if (byte == 0x55U)
        {
            imu_frame[0]      = 0x55U;
            imu_frame_index   = 1U;
            imu_frame_syncing = 0U;
        }
        return;
    }

    /*
     * 第二个字节必须是合法 TYPE（0x50~0x5A）。
     * 数据区里也可能出现 0x55，靠这一条挡掉大部分假帧头。
     */
    if ((imu_frame_index == 1U) && ((byte < 0x50U) || (byte > 0x5AU)))
    {
        imu_frame_index   = 0U;
        imu_frame_syncing = 1U;
        return;
    }

    imu_frame[imu_frame_index++] = byte;

    if (imu_frame_index >= IMU_FRAME_LEN)
    {
        imu_frame_index   = 0U;
        imu_frame_syncing = 1U;
        Imu_HandleFrame(imu_frame);
    }
}

/* ==================================================================
 * 串口中断回调（由 main.c 按 huart->Instance 分发进来）
 * ================================================================== */

void Imu_RxCpltCallback(void)
{
    imu_raw_bytes++;        /* 只要中断进来了就计数，不看内容 */
    Imu_ParseByte(imu_rx_byte);   /* 在中断里直接解析，主循环再慢也不丢帧 */

    (void)HAL_UART_Receive_IT(&huart3, (uint8_t *)&imu_rx_byte, 1U);
}

void Imu_RxErrorCallback(void)
{
    /* ORE/FE 等错误会让 HAL 结束当前接收事务，必须重新挂上 */
    (void)HAL_UART_Receive_IT(&huart3, (uint8_t *)&imu_rx_byte, 1U);
}

/* ==================================================================
 * 对外接口
 * ================================================================== */

void Imu_Update(void)
{
    uint32_t now;

    now = HAL_GetTick();
    imu_data.online  = ((uint32_t)(now - imu_data.last_tick) > IMU_TIMEOUT_MS) ? 0U : 1U;
    imu_state.online = imu_data.online;

    /*
     * 接收看门狗：
     * 如果 500ms 一个有效帧都没有，就重新挂一次单字节接收。
     * HAL 在 ORE 等错误后不一定会自动恢复，这一句能把它救回来；
     * 接收本来就正常时，这个调用只会返回 HAL_BUSY，无副作用。
     */
    if ((uint32_t)(now - imu_data.last_tick) > 500U)
    {
        (void)HAL_UART_Receive_IT(&huart3, (uint8_t *)&imu_rx_byte, 1U);
    }
}

void Imu_MinimalTest(void)
{
    uint32_t start;
    uint32_t af0;
    uint32_t bytes;
    uint32_t aframes;
    float    yaw0;
    float    dyaw;

    af0 = imu_angle_frames;
    imu_raw_bytes = 0U;

    /* 1) 先观察 2 秒，什么都不做 */
    start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start) < 2000U)
    {
        Imu_Update();
        HAL_Delay(1);
    }

    bytes   = imu_raw_bytes;
    aframes = imu_angle_frames - af0;

    /* 2) 一个字节都没收到 —— 完全不动 */
    if (bytes < 100U)
    {
        Car_Move(0.0f, 0.0f, 0.0f);
        return;
    }

    /* 3) 有字节但没有角度帧（0x53）—— 顺时针转 1.5s */
    if (aframes < 20U)
    {
        Car_Move(0.0f, 0.0f, -60.0f);
        start = HAL_GetTick();
        while ((uint32_t)(HAL_GetTick() - start) < 1500U)
        {
            Imu_Update();
            HAL_Delay(2);
        }
        Car_Move(0.0f, 0.0f, 0.0f);
        HAL_Delay(300U);
        return;
    }

    /* 4) 有角度帧：逆时针转 1.5s，同时看 yaw 有没有跟着变 */
    yaw0 = imu_state.yaw;
    Car_Move(0.0f, 0.0f, 60.0f);
    start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start) < 1500U)
    {
        Imu_Update();
        HAL_Delay(2);
    }
    Car_Move(0.0f, 0.0f, 0.0f);
    HAL_Delay(300U);

    dyaw = imu_state.yaw - yaw0;

    if (fabsf(dyaw) > 30.0f)
    {
        /* A：yaw 跟着转 —— 全通。反向转回去，回到起始朝向 */
        Car_Move(0.0f, 0.0f, -60.0f);
        start = HAL_GetTick();
        while ((uint32_t)(HAL_GetTick() - start) < 1500U)
        {
            Imu_Update();
            HAL_Delay(2);
        }
        Car_Move(0.0f, 0.0f, 0.0f);
        HAL_Delay(300U);
    }
    else
    {
        /* C：角度帧有，但 yaw 不动 —— 前进后退点两下 */
        Car_Move(20.0f, 0.0f, 0.0f);
        HAL_Delay(300U);
        Car_Move(-20.0f, 0.0f, 0.0f);
        HAL_Delay(300U);
        Car_Move(20.0f, 0.0f, 0.0f);
        HAL_Delay(300U);
        Car_Move(-20.0f, 0.0f, 0.0f);
        HAL_Delay(300U);
        Car_Move(0.0f, 0.0f, 0.0f);
    }
}

void Imu_Init(void)
{
    uint32_t start;

    memset(&imu_data, 0, sizeof(imu_data));
    memset(&imu_calib, 0, sizeof(imu_calib));
    memset(&imu_state, 0, sizeof(imu_state));

    imu_frame_index   = 0U;
    imu_frame_syncing = 1U;
    imu_yaw_last_raw  = 0.0f;
    imu_yaw_cont      = 0.0f;
    imu_yaw_inited    = 0U;
    imu_gz_filtered   = 0.0f;
    imu_yaw_int       = 0.0f;
    imu_last_gz_tick  = 0U;
    imu_angle_frames  = 0U;
    imu_zero_ref      = 0.0f;
    imu_hold_pause_until = 0U;
    imu_hold_fault_tick  = 0U;

    imu_state.yaw_sign    = (int8_t)IMU_YAW_SIGN_INIT;
    imu_state.motor_sign  = (int8_t)IMU_MOTOR_SIGN_INIT;
    imu_state.hold_target = 0.0f;
    imu_state.hold_on     = 0U;
    imu_state.hold_active = 0U;
    imu_state.hold_fault  = 0U;

    /* 先布防：即使现在没数据，后面主循环收到第一帧也会自动接上 */
    imu_boot_arm = 1U;

    /* USART3 已由 CubeMX 初始化好，这里只挂单字节中断接收 */
    (void)HAL_UART_Receive_IT(&huart3, (uint8_t *)&imu_rx_byte, 1U);

    /*
     * 模块启动时间 1000ms，这段时间输出不可信。
     * 等够时间再交给上层，顺便把开机噪声帧排掉。
     */
    start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start) < IMU_BOOT_WAIT_MS)
    {
        Imu_Update();
        HAL_Delay(1);
    }

#if (IMU_BOOT_TEST != 0)
    /*
     * 最简自检：只用车的动作报告串口状态，不需要调试器和屏幕。
     *   A 逆时针转   = 帧正常
     *   B 顺时针转   = 有字节但凑不成帧（波特率/干扰）
     *   C 前后点一下 = 一个字节都没收到（接线/共地/接错引脚组）
     * 自检结束后继续走正常的 0° 保持流程。
     */
    Imu_MinimalTest();
#endif

    /*
     * 等到第一帧有效角度再置零，否则零点会取在还没收到数据的时刻。
     * 最多再等 1s，模块没接也不会卡死在启动阶段。
     */
    start = HAL_GetTick();
    while ((Imu_IsOnline() == 0U) &&
           ((uint32_t)(HAL_GetTick() - start) < 1000U))
    {
        Imu_Update();
        HAL_Delay(1);
    }

    if (Imu_IsOnline() != 0U)
    {
        /* 上电时的车头方向 = 0°，并立刻开始保持这个方向 */
        Imu_ZeroYaw();
        Imu_HoldStart(0.0f);
        imu_boot_arm = 0U;
    }
    /* 没等到数据就保持布防状态，交给主循环里的 Imu_HoldStep 补上 */
}

void Imu_StaticCalib(void)
{
    float sum = 0.0f;
    float sum_sq = 0.0f;
    float sum_gz = 0.0f;
    float mean;
    float var;
    uint32_t start;
    uint32_t last_frames;
    uint16_t count = 0U;

    imu_calib.still   = 0U;
    imu_calib.samples = 0U;

    start       = HAL_GetTick();
    last_frames = imu_angle_frames;

    while (((uint32_t)(HAL_GetTick() - start) < IMU_CALIB_MS) &&
           (count < IMU_CALIB_MAX))
    {
        Imu_Update();

        if (imu_angle_frames != last_frames)
        {
            last_frames = imu_angle_frames;
            sum        += imu_data.yaw;
            sum_sq     += imu_data.yaw * imu_data.yaw;
            sum_gz     += imu_data.gz;
            count++;
        }

        HAL_Delay(1);
    }

    imu_calib.samples = count;

    if (count >= 10U)
    {
        mean = sum / (float)count;
        var  = (sum_sq / (float)count) - (mean * mean);
        if (var < 0.0f)
        {
            var = 0.0f;
        }

        imu_calib.yaw_zero = mean;
        imu_calib.yaw_std  = sqrtf(var);
        imu_calib.gz_bias  = sum_gz / (float)count;
        imu_calib.still    = (imu_calib.yaw_std < IMU_STILL_STD_DEG) ? 1U : 0U;
    }

}

void Imu_ZeroYaw(void)
{
    /*
     * 只做软件零点，不再给模块发 CALSW。
     * 模块那边清零既多余，也曾被认为是"发完就断流"的可疑变量。
     */
#if (IMU_YAW_FROM_GYRO != 0)
    imu_zero_ref = (float)imu_state.yaw_sign * imu_yaw_int;
#else
    imu_zero_ref = (float)imu_state.yaw_sign * imu_yaw_cont;
#endif

    imu_state.yaw      = 0.0f;
    imu_state.yaw_disp = 0.0f;
    imu_state.rate     = 0.0f;
}

void Imu_HoldStart(float target_deg)
{
    imu_state.hold_target = target_deg;
    imu_state.hold_on     = 1U;
    imu_state.hold_fault  = 0U;
    imu_state.hold_active = 0U;
    imu_hold_fault_tick   = 0U;
}

void Imu_HoldPause(uint32_t ms)
{
    imu_hold_pause_until = HAL_GetTick() + ms;
}

void Imu_HoldStop(void)
{
    imu_boot_arm         = 0U;   /* 手动停了就不再自动重启 */
    imu_state.hold_on     = 0U;
    imu_state.hold_active = 0U;
    imu_state.hold_fault  = 0U;
    imu_hold_fault_tick   = 0U;
    Car_Move(0.0f, 0.0f, 0.0f);
}

#if (IMU_HOLD_SIMPLE != 0)

/*
 * 最简闭环：只有一条比例项，没有死区、没有滤波、没有任何状态。
 * 专门用来回答一个问题：yaw 变了，电机到底会不会动。
 *   yaw 有任何变化 -> 立刻输出角速度
 *   yaw 恒为 0     -> 永远不动
 */
static void Imu_HoldSimpleStep(void)
{
    float err;
    float w;

    /* 手动动作刚下发，先让它生效一会儿 */
    if ((int32_t)(HAL_GetTick() - imu_hold_pause_until) < 0)
    {
        return;
    }

    if (imu_state.online == 0U)
    {
        Car_Move(0.0f, 0.0f, 0.0f);
        return;
    }

    err = Imu_Wrap180(imu_state.hold_target - imu_state.yaw);

    /*
     * 注意：这里原来有一句 "|err| > 150° 就停车"，是早期防"符号反了自转"用的。
     * 它会在离目标 150°~180° 的地方形成一个 60° 宽的死区（顺时针 170~210 度
     * 推过去完全没反应），是纯 bug，已经删掉。
     * 现在 Imu_Wrap180 保证走最短路，负反馈本身是稳定的，不需要这个刹车。
     */

    /* 1° 死区：比这个小的偏差当作噪声，不动 */
    if (fabsf(err) < IMU_HOLD_ENTER_DEG)
    {
        w = 0.0f;
    }
    else
    {
        /* 比例输出：偏差越小转得越慢，避免一触发就冲过头 */
        w = 4.0f * err;

        if (w > 80.0f)
        {
            w = 80.0f;
        }
        else if (w < -80.0f)
        {
            w = -80.0f;
        }

        /*
         * 下限：算出来的速度太小电机会被静摩擦卡住，
         * 给一个 20°/s 的最低有效转速（车轮约 91°/s）。
         */
        if (fabsf(w) < 20.0f)
        {
            w = (err > 0.0f) ? 20.0f : -20.0f;
        }
    }

    Car_Move(0.0f, 0.0f, (float)imu_state.motor_sign * w);
    imu_state.last_omega = w;
}

#else

/* 完整版：死区迟滞 + 平方根制动 + 失速保护 */
static void Imu_HoldFullStep(void)
{
    float err;
    float ae;
    float w;
    float wmax;
    uint32_t now;

    now = HAL_GetTick();

    /* 手动动作刚下发，先让它生效一会儿 */
    if ((int32_t)(now - imu_hold_pause_until) < 0)
    {
        return;
    }

    if (imu_state.online == 0U)
    {
        Car_Move(0.0f, 0.0f, 0.0f);
        return;
    }

    err = Imu_Wrap180(imu_state.hold_target - imu_state.yaw);
    ae  = fabsf(err);

    /*
     * 进入/退出死区做成迟滞：
     * 误差 < ENTER 就停，超过 EXIT 才重新动，避免在门限附近来回抖。
     */
    if (ae < IMU_HOLD_ENTER_DEG)
    {
        imu_state.hold_active = 0U;
    }
    else if (ae > IMU_HOLD_EXIT_DEG)
    {
        imu_state.hold_active = 1U;
    }

    if (imu_state.hold_active == 0U)
    {
        imu_hold_fault_tick = 0U;
        Car_Move(0.0f, 0.0f, 0.0f);
        return;
    }

    /*
     * 失控保护：正常纠偏时 |err| 一定在变小。
     * 连续 IMU_HOLD_STALL_MS 都没把误差压下去 2° 以上，只可能是两种：
     *   1. yaw 符号反了 —— 正反馈，越推越远
     *   2. 车被卡住 / 轮子悬空 —— 电机转但车不转
     * 两种情况都必须马上停车，否则会一直转下去。
     */
    if (imu_hold_fault_tick == 0U)
    {
        imu_hold_fault_tick = now;
        imu_hold_fault_err  = ae;
    }
    else if ((uint32_t)(now - imu_hold_fault_tick) > IMU_HOLD_STALL_MS)
    {
        if (ae > (imu_hold_fault_err - 2.0f))
        {
            imu_state.hold_on     = 0U;
            imu_state.hold_active = 0U;
            imu_state.hold_fault  = 1U;
            Imu_StopCar();
            return;
        }

        /* 还在收敛，滑动窗口重新计时 */
        imu_hold_fault_tick = now;
        imu_hold_fault_err  = ae;
    }

    /* 比例 + 平方根制动曲线 */
    wmax = sqrtf(2.0f * IMU_HOLD_A_DECEL * ae);
    if (wmax > IMU_HOLD_OMEGA_MAX)
    {
        wmax = IMU_HOLD_OMEGA_MAX;
    }

    w = IMU_HOLD_KP * err;
    if (w > wmax)
    {
        w = wmax;
    }
    else if (w < -wmax)
    {
        w = -wmax;
    }

    if (fabsf(w) < IMU_HOLD_OMEGA_MIN)
    {
        w = (err > 0.0f) ? IMU_HOLD_OMEGA_MIN : -IMU_HOLD_OMEGA_MIN;
    }

    Car_Move(0.0f, 0.0f, (float)imu_state.motor_sign * w);
    imu_state.last_omega = w;
}

#endif

void Imu_HoldStep(void)
{
    /*
     * 上电自动布防：模块可能比 MCU 晚上电、或者第一次没等到数据。
     * 只要哪一轮发现"已经收到有效帧"，就立刻把当前方向定成 0° 并开始保持。
     */
    if (imu_state.hold_on == 0U)
    {
        if ((imu_boot_arm != 0U) && (imu_state.online != 0U))
        {
            Imu_ZeroYaw();
            Imu_HoldStart(0.0f);
            imu_boot_arm = 0U;
        }
        else
        {
            return;
        }
    }

#if (IMU_HOLD_SIMPLE != 0)
    Imu_HoldSimpleStep();
#else
    Imu_HoldFullStep();
#endif
}

float Imu_GetYaw(void)
{
    return imu_state.yaw;
}

float Imu_GetRate(void)
{
    return imu_state.rate;
}

uint8_t Imu_IsOnline(void)
{
    return imu_state.online;
}

float Imu_HeadingOmega(float target_deg, float *err_out)
{
    float err = Imu_Wrap180(target_deg - imu_state.yaw);
    float ae  = fabsf(err);
    float wmax;
    float w;

    /* 平方根制动曲线：接近目标时自动把允许角速度压下来 */
    wmax = sqrtf(2.0f * IMU_TURN_A_DECEL * ae);
    if (wmax > IMU_TURN_OMEGA_MAX)
    {
        wmax = IMU_TURN_OMEGA_MAX;
    }

    w = IMU_TURN_KP * err;
    if (w > wmax)
    {
        w = wmax;
    }
    else if (w < -wmax)
    {
        w = -wmax;
    }

    /* 误差还大但算出来的量太小，电机可能根本不动，给一个下限 */
    if ((fabsf(w) < IMU_TURN_OMEGA_MIN) && (ae > IMU_TURN_TOL_DEG))
    {
        w = (err > 0.0f) ? IMU_TURN_OMEGA_MIN : -IMU_TURN_OMEGA_MIN;
    }

    if (err_out != NULL)
    {
        *err_out = err;
    }

    return w;
}

int Imu_TurnTo(float target_deg)
{
    uint8_t pass;
    uint8_t stable;
    float err;
    float w;
    float omax;
    uint32_t start;

    if (Imu_IsOnline() == 0U)
    {
        return -1;
    }

    imu_state.moving = 1U;

    /*
     * 转完就停在这个角度上继续定住，
     * 所以目标直接交给航向保持，转到位后由 HoldStep 接力维持。
     */
    imu_state.hold_target = target_deg;
    imu_state.hold_fault  = 0U;
    imu_hold_fault_tick   = 0U;

    for (pass = 0U; pass < IMU_TURN_PASSES; pass++)
    {
        /* 第二、三遍降速，专门用来啃掉残余误差 */
        omax = IMU_TURN_OMEGA_MAX;
        if (pass == 1U)
        {
            omax = IMU_TURN_OMEGA_MAX * 0.5f;
        }
        else if (pass >= 2U)
        {
            omax = IMU_TURN_OMEGA_MAX / 3.0f;
        }

        start  = HAL_GetTick();
        stable = 0U;

        while ((uint32_t)(HAL_GetTick() - start) < IMU_TURN_TIMEOUT_MS)
        {
            Imu_Update();

            if (Imu_IsOnline() == 0U)
            {
                Imu_StopCar();
                imu_state.moving = 0U;
                return -1;
            }

            w = Imu_HeadingOmega(target_deg, &err);
            if (w > omax)
            {
                w = omax;
            }
            else if (w < -omax)
            {
                w = -omax;
            }

            if (fabsf(err) < IMU_TURN_TOL_DEG)
            {
                if (stable < 250U)
                {
                    stable++;
                }
            }
            else
            {
                stable = 0U;
            }

            if ((stable >= IMU_TURN_STABLE_CNT) &&
                (fabsf(imu_state.rate) < IMU_TURN_RATE_TOL))
            {
                break;
            }

            Car_Move(0.0f, 0.0f, (float)imu_state.motor_sign * w);
            imu_state.last_omega = w;
            HAL_Delay(5);
        }

        Imu_StopCar();
        HAL_Delay(IMU_TURN_SETTLE_MS);
        Imu_Update();

        imu_state.last_err = Imu_Wrap180(target_deg - imu_state.yaw);
        if (fabsf(imu_state.last_err) < IMU_TURN_TOL_DEG)
        {
            imu_state.moving = 0U;
            return 0;
        }
    }

    imu_state.moving = 0U;
    return -2;
}

int Imu_TurnBy(float delta_deg)
{
    /* 在当前航向上再转 delta_deg：0x46 每按一次 +90° */
    return Imu_TurnTo(imu_state.yaw + delta_deg);
}

void Imu_SelfTest(void)
{
#if (IMU_TEST_MODE == 0)

    /* 比赛模式：什么都不做 */
    return;

#elif (IMU_TEST_MODE == 1)

    /* 只显示 yaw，电机完全不动：只做静止统计 + 置零，不探测方向 */
    OLED_ShowString(1, 1, "IMU M1");
    Imu_StaticCalib();
    Imu_ZeroYaw();
    return;

#elif (IMU_TEST_MODE == 2)

    /* 只置零并开始保持，之后停在显示状态 */
    OLED_ShowString(1, 1, "IMU M2");
    Imu_ZeroYaw();
    Imu_HoldStart(0.0f);
    OLED_ShowString(2, 1, "HOLD 0");
    HAL_Delay(1500U);
    return;

#else

    /* 定角度自测：+90 -> 0 -> -90 -> 0 无限循环 */
    uint8_t i = 0U;
    static const float seq[4] = { 90.0f, 0.0f, -90.0f, 0.0f };

    OLED_ShowString(1, 1, "IMU M3");
    Imu_ZeroYaw();
    Imu_HoldStart(0.0f);
    OLED_ShowString(2, 1, "TURN TEST");
    HAL_Delay(1500U);

    while (1)
    {
        (void)Imu_TurnTo(seq[i]);
        Imu_DebugToOled();
        HAL_Delay(800U);

        i = (uint8_t)((i + 1U) % 4U);
    }

#endif
}

void Imu_DebugToOled(void)
{
#if (IMU_DEBUG_OLED != 0)
    static uint32_t last_tick = 0U;
    uint32_t now = HAL_GetTick();

    /* OLED 是软件 I2C，刷新太快会拖慢主循环，限到 10Hz */
    if ((uint32_t)(now - last_tick) < 100U)
    {
        return;
    }
    last_tick = now;

    /*
     * 屏幕分四行，左右两列：
     *   L1  F=角度帧计数(0x53)        E=校验错误数
     *   L2  R=模块原始 yaw            Y=车辆航向（展开-零点后）
     *   L3  E=当前误差                W=输出的车体角速度
     *   L4  H=保持开关 A=正在纠偏 M=电机符号  最后一位是心跳
     */
    OLED_ShowString(1, 1, "F");
    OLED_ShowNum(1, 2, imu_angle_frames, 5);
    OLED_ShowString(1, 7, "E");
    OLED_ShowNum(1, 8, imu_data.sum_errors, 3);

    OLED_ShowString(2, 1, "R");
    OLED_ShowSignedNum(2, 2, (int32_t)(imu_data.yaw * 10.0f), 4);
    OLED_ShowString(2, 7, "Y");
    OLED_ShowSignedNum(2, 8, (int32_t)(imu_state.yaw * 10.0f), 4);

    OLED_ShowString(3, 1, "E");
    OLED_ShowSignedNum(3, 2,
                       (int32_t)(Imu_Wrap180(imu_state.hold_target - imu_state.yaw) * 10.0f), 4);
    OLED_ShowString(3, 7, "W");
    OLED_ShowSignedNum(3, 8, (int32_t)imu_state.last_omega, 3);

    OLED_ShowString(4, 1, "H");
    OLED_ShowNum(4, 2, imu_state.hold_on, 1);
    OLED_ShowString(4, 4, "A");
    OLED_ShowNum(4, 5, imu_state.hold_active, 1);
    OLED_ShowString(4, 7, "M");
    OLED_ShowSignedNum(4, 8, (int32_t)imu_state.motor_sign, 1);

    /*
     * L = 主循环调用 Imu_Update 的次数（取低 3 位）。
     * 它一直在变 = 主循环活着；冻住 = 循环卡死或停了。
     */
    OLED_ShowString(4, 11, "L");
    OLED_ShowNum(4, 12, imu_loops % 1000U, 3);
#else
    /* 编译期关掉后不留任何代码 */
#endif
}
