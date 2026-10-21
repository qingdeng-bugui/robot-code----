#include "Function.h"
#include "adc.h"
#include "LineFast.h"
#include "Motor.h"
#include "Nav.h"
#include "PID.h"
#include "Serial.h"
#include "Servo.h"
#include "ir_nec.h"
#include "track.h"
#include "usart.h"
#include "Photoresistor.h"
#include <math.h>
#include <string.h>

/*
 * 视觉串口一行数据的最大长度（包含结尾的 '\0'）。
 * 当前协议最长的正常帧是“整数,0/1,red,整数”，同时还要容纳负号；
 * 预留余量可以让解析器在收到异常长帧时主动丢弃，而不是写越界。
 */
#define VISUAL_LINE_LENGTH 32U

/*
 * USART1 中断接收使用的环形字节缓存大小。
 * 中断只负责把字节写入缓存，主循环稍后再组装完整行，
 * 因此可以吸收树莓派短时间内连续发送的数据突发。
 */
#define VISUAL_RX_BUFFER_SIZE 512U

/*
 * 视觉目标帧短暂中断时允许继续运动的最长时间，单位为毫秒。
 * 超过该时间仍未收到新目标，就停车等待，避免使用过期的 dx 继续行驶。
 */
#define VISUAL_TARGET_TIMEOUT_MS 500U

/* 红球只在确认会进入车体通行区域时短暂接管底盘。 */
#define VISUAL_RED_FRONT_DANGER_DEG 65.0f
#define VISUAL_RED_CENTER_BEARING_DEG 12.0f
#define VISUAL_RED_LANE_HALF_DEG 22.0f
#define VISUAL_RED_MOTION_CONFLICT_DEG 28.0f
#define VISUAL_RED_CONFLICT_FRAMES 3U
#define VISUAL_RED_CLEAR_FRAMES 5U
#define VISUAL_RED_OBSERVE_TIME_MS 300U
#define VISUAL_RED_TURN_AWAY_OMEGA 70.0f
#define VISUAL_RED_TURN_AWAY_TIME_MS 700U
#define VISUAL_RED_TURN_BACK_MIN_MS 250U
#define VISUAL_RED_TURN_BACK_MAX_MS 1200U
#define VISUAL_RED_TURN_BACK_MIN_OMEGA 45.0f
#define VISUAL_RED_TURN_BACK_MAX_OMEGA 70.0f
#define VISUAL_RED_TURN_BACK_GAIN 2.0f
#define VISUAL_RED_TURN_BACK_CENTER_HOLD_MS 120U
#define VISUAL_RED_FRAME_TIMEOUT_MS 500U

/*
 * 舵机2的逻辑角度定义：168°对应车体正前方。
 * 这里使用 Servo.h 的逻辑角度，不是定时器脉宽；角度增大表示云台向左转。
 */
#define VISUAL_CENTER_ANGLE 168.0f
#define VISUAL_SERVO_MIN_ANGLE 80.0f
#define VISUAL_SERVO_MAX_ANGLE 258.0f

/*
 * 普通跟踪时每收到一帧视觉数据的云台修正参数。
 * dx 是图像横向像素偏差，不是角度；因此必须乘以比例系数并限制单次步长。
 */
#define VISUAL_SERVO_TRACK_GAIN 0.02f

/*
 * 收到 can_hit=1 之后的“出针 → 推进 → 反复戳”参数。
 *
 * 视觉端的 can_hit=1 只表示“已经进入可戳窗口”：此时云台/摄影轴线大致对准气球，
 * 但钢针即使在 135° 全伸状态下，针尖离气球表面仍有约 3cm。
 * 所以下位机收到 1 以后不能再做任何“针还没出就往前挪”的小补近，否则车身会先顶到气球
 * （规则：非钢针部分碰破气球每个加时 30s）。现在的策略：
 *   1. 立即停车，只用云台在最长时间 150ms（或 |dx|<=30px）内把针轴收正；
 *   2. 马上出针（舵机1 = 135°），等钢针走完行程；
 *   3. 先沿云台指向（针轴）以 6cm/s 直推约 2.4cm，把针尖送到气球表面；
 *   4. 之后一边继续以 3cm/s 前进，一边让舵机1 在 87°↔135° 之间按 380ms 节拍反复出收，
 *      用连续的扎击代替一次性的硬顶（球是软的、又被双面胶固定，单次直推容易只顶不破）；
 *   5. 目标从画面消失（视觉判定已扎破）或到达 2.0s 上限就停车收针，再后退脱离。
 * 行程预算：6cm/s×0.4s + 3cm/s×1.6s ≈ 7.2cm，针管口在针尖后方 6cm，仍留约 1.8cm 余量，
 * 不会演变成“车身去顶气球”。
 */
#define VISUAL_HIT_ALIGN_GAIN 0.1f     /* 对准阶段云台修正增益，比普通跟踪更积极。 */
#define VISUAL_HIT_ALIGN_DEADBAND 6    /* 对准阶段的小死区：死区取大了收敛会明显变慢。 */
#define VISUAL_HIT_ALIGN_MAX_STEP 6.0f /* 对准阶段单帧最大修正角度，避免过冲把针甩过目标。 */
#define VISUAL_HIT_ALIGN_TOLERANCE 30  /* |dx| 进入该范围就认为针轴已经对准，可以出针。 */
#define VISUAL_HIT_ALIGN_MAX_MS 150U   /* 对准总时限：到点后即使 dx 偏大也照常出针，避免卡死。 */
#define VISUAL_HIT_LOST_ABORT_MS 500U  /* 对准期间目标连续消失这么久，就放弃本次出针，避免对着空气伸针。 */
#define VISUAL_HIT_NEEDLE_OUT_MS 150U  /* 出针到 135° 后等舵机走完行程再开始推进。 */
#define VISUAL_HIT_PUSH_SPEED 6.0f     /* 反复戳之前：先直推到位，把针尖送到球面。 */
#define VISUAL_HIT_JAB_LEAD_MS 400U    /* 直推时长：6cm/s × 0.4s ≈ 2.4cm。 */
#define VISUAL_HIT_JAB_SPEED 3.0f      /* 反复戳期间的前进速度；比 6cm/s 慢，避免把球顶飞/车身接触。 */
#define VISUAL_HIT_JAB_IN_MS 180U      /* 每个节拍先收针 87° 保持的时间（回抽）。 */
#define VISUAL_HIT_JAB_OUT_MS 200U     /* 再出针 135° 保持的时间（扎击）。 */
#define VISUAL_HIT_PUSH_MAX_MS 2000U   /* 推进+反复戳总时限：总行程约 7cm，防止目标异常时一直顶撞。 */
#define VISUAL_HIT_PUSH_TRIM_GAIN 0.05f     /* 推进中云台微调增益，只做小幅纠正。 */
#define VISUAL_HIT_PUSH_TRIM_DEADBAND 20    /* 推进中微调死区，避免针尖来回摆。 */
#define VISUAL_HIT_PUSH_TRIM_MAX_STEP 2.0f  /* 推进中单帧最大修正角度。 */
#define VISUAL_HIT_RETRACT_MS 200U        /* 收针后短暂停稳，再开始后退。 */
#define VISUAL_HIT_BACKOFF_SPEED 6.0f
#define VISUAL_HIT_BACKOFF_TIME_MS 550U

/*
 * dx 死区：目标已经基本居中时不再调整云台。
 * 这样可以减少视觉噪声造成的来回抖动，也能避免针尖在目标附近反复扫动。
 */
#define VISUAL_SERVO_DEADBAND 12

/* 场内接近速度，单位为 cm/s；收到 can_hit 之前全速接近，收到后由戳击状态机降速。 */
#define VISUAL_APPROACH_SPEED 15.0f

/*
 * 云台搜索范围和扫描节拍。搜索采用往返方式，
 * 到达任一机械端点后反向，从而覆盖车体前方约 180° 的空间。
 */
#define VISUAL_SEARCH_MIN_ANGLE 80
#define VISUAL_SEARCH_MAX_ANGLE 258
#define VISUAL_SEARCH_STEP 3
#define VISUAL_SEARCH_INTERVAL_MS 60U
#define VISUAL_GIMBAL_LIMIT_DX 24
#define VISUAL_GIMBAL_LIMIT_FRAMES 6U
#define VISUAL_GIMBAL_LIMIT_ANGLE_MARGIN 2.0f
#define VISUAL_GIMBAL_BACKOFF_SPEED 10.0f
#define VISUAL_GIMBAL_BACKOFF_TIME_MS 1300U

/* 横移略作放大，避免目标长期位于画面边缘时小车只向前走。 */
#define VISUAL_LATERAL_GAIN 1.25f
#define VISUAL_MAX_LATERAL_SPEED 12.0f

/*
 * 无目标时底盘沿当前车头方向慢速扫场；检测到前方边界后后退、掉头，
 * 再覆盖另一条方向。这样视觉搜索不仅转动云台，也会改变车辆所在位置。
 */
#define VISUAL_SEARCH_SPEED 10.0f
#define VISUAL_BACKOFF_SPEED 5.0f
#define VISUAL_BACKOFF_TIME_MS 600U
#define VISUAL_TURN_OMEGA 120.0f
#define VISUAL_TURN_SCAN_START_MS 750U
#define VISUAL_TURN_SCAN_OMEGA 60.0f
#define VISUAL_TURN_TIME_MS 2250U
#define VISUAL_TURN_SETTLE_MS 120U

/* 视觉解析结果的内部命令类型，避免在主状态机中直接比较字符串。 */
#define VISUAL_COMMAND_INVALID 0U
#define VISUAL_COMMAND_TARGET 1U
#define VISUAL_COMMAND_NONE 2U
#define VISUAL_COMMAND_OK 3U
#define VISUAL_RED_AVOID_IDLE 0U
#define VISUAL_RED_AVOID_OBSERVE 1U
#define VISUAL_RED_AVOID_TURN_AWAY 2U
#define VISUAL_RED_AVOID_TURN_BACK 3U
#define VISUAL_RED_AVOID_WAIT_CLEAR 4U

/*
 * 戳球状态机的内部编码，避免在主循环里直接使用魔法数字：
 *   HIT_IDLE    普通跟踪，按 can_hit 决定是否开始一次戳击
 *   HIT_ALIGN   收针状态下极短的停车对准（≤150ms 或 |dx|≤30px）
 *   HIT_EXTEND  钢针伸至 135° 并等待舵机走完行程
 *   HIT_PUSH    保持出针，沿云台（针）轴向目标低速推进
 *   HIT_RETRACT 停车收针并等待机构稳定
 *   HIT_BACKOFF 后退脱离气球，然后回到跟踪/搜索
 */
#define HIT_IDLE 0U
#define HIT_ALIGN 1U
#define HIT_EXTEND 2U
#define HIT_PUSH 3U
#define HIT_RETRACT 4U
#define HIT_BACKOFF 5U
static void Scene_StopCar(void);
static uint8_t Visual_FrontBoundaryDetected(void);
static void Visual_AdjustServo(float *servo_angle, int dx, float gain, int deadband, float max_step);

/*
 * 下面四个变量由 USART1 接收中断和主循环共同访问，因此声明为 volatile。
 * head 指向下一个写入位置，tail 指向下一个待读取字节；两者相等表示为空。
 */
static volatile uint8_t visual_rx_buffer[VISUAL_RX_BUFFER_SIZE];
static volatile uint16_t visual_rx_head = 0U;
static volatile uint16_t visual_rx_tail = 0U;
static volatile uint8_t visual_rx_byte = 0U;
static volatile uint8_t visual_rx_overflow = 0U;

/* 主循环从环形缓存取字节后，在这里逐行拼接视觉协议帧。 */
static char visual_line_buffer[VISUAL_LINE_LENGTH];
static uint8_t visual_line_length = 0U;
static uint8_t visual_discard_line = 0U;

void Keep_In(void) // 防止出界
{
    /*
     * 场内保护函数由运动循环反复调用，不负责独立导航，只负责在检测到
     * 黑色边界线时把车体推回场内。所有脱困动作都采用时间状态机，
     * 因而本函数不会长时间阻塞调用它的循迹或视觉控制循环。
     */
    static uint8_t avoid_direction = 0U; /* 1:向右纠偏，2:向左纠偏 */
    static uint8_t clear_count = 0U;
    static uint32_t avoid_until = 0U;
    static uint8_t edge_escape_state = 0U;    /* 0空闲，1角点后退，2角点转向，4正面后退。 */
    static uint32_t edge_escape_tick = 0U;    /* 当前脱困子状态开始的时间戳。 */
    static uint32_t edge_cooldown_until = 0U; /* 脱困后短暂禁止重复触发的截止时间。 */
    uint8_t left_edge = 0;
    uint8_t center_edge = 0;
    uint8_t right_edge = 0;
    uint32_t now;

    const float keep_forward_speed = 12.0f;
    const float keep_lateral_speed = 12.0f;
    const float edge_reverse_speed = 18.0f;
    const float edge_turn_speed = 180.0f;
    const uint32_t keep_hold_ms = 150U;
    const uint32_t corner_reverse_ms = 180U;
    const uint32_t corner_turn_ms = 300U;
    const uint32_t front_reverse_ms = 220U;
    const uint32_t edge_settle_ms = 100U;
    const uint32_t edge_cooldown_ms = 350U;

    /* 开启红外模块；重复调用不会重复初始化。 */
    Track_Control(1);
    now = HAL_GetTick();

    /*
     * 将八路传感器按空间位置合并成左、中、右三个区域：
     * 左右区域用于判断侧边纠偏或角点，中间区域用于判断正面触边。
     */
    left_edge = (track_values[0] > 1300 ||
                 track_values[1] > 1300 ||
                 track_values[2] > 1300);
    center_edge = (track_values[3] > 1300 ||
                   track_values[4] > 1300);
    right_edge = (track_values[5] > 1300 ||
                  track_values[6] > 1300 ||
                  track_values[7] > 1300);

    /*
     * 两侧黑线、中间白线通常表示车身斜压在方形角点上。
     * 使用非阻塞的后退-转向动作，避免在同一个角点反复停车和延时。
     */
    if (edge_escape_state == 1U)
    {
        /* 角点第一阶段：先沿车体反方向退出黑线交汇区域。 */
        if ((uint32_t)(now - edge_escape_tick) < corner_reverse_ms)
        {
            Car_Move(-edge_reverse_speed, 0.0f, 0.0f);
            return;
        }

        edge_escape_state = 2U;
        edge_escape_tick = now;
    }

    if (edge_escape_state == 2U)
    {
        /* 角点第二阶段：原地转向，使车头离开当前角点方向。 */
        if ((uint32_t)(now - edge_escape_tick) < corner_turn_ms)
        {
            Car_Move(0.0f, 0.0f, edge_turn_speed);
            return;
        }

        edge_escape_state = 3U;
        edge_escape_tick = now;
        edge_cooldown_until = now + edge_cooldown_ms;
    }

    if (edge_escape_state == 4U)
    {
        /* 正面触边只需要后退，不额外旋转车身。 */
        if ((uint32_t)(now - edge_escape_tick) < front_reverse_ms)
        {
            Car_Move(-edge_reverse_speed, 0.0f, 0.0f);
            return;
        }

        edge_escape_state = 5U;
        edge_escape_tick = now;
    }

    if (edge_escape_state == 3U || edge_escape_state == 5U)
    {
        /* 后退/转向完成后短暂停车，让机械振动和传感器读数稳定。 */
        if ((uint32_t)(now - edge_escape_tick) < edge_settle_ms)
        {
            Scene_StopCar();
            return;
        }

        edge_escape_state = 0U;
    }

    if ((int32_t)(now - edge_cooldown_until) < 0)
    {
        /* 冷却期内不重新启动脱困状态，避免在边界附近反复触发。 */
        if (center_edge)
        {
            Car_Move(-edge_reverse_speed, 0.0f, 0.0f);
        }
        else
        {
            Scene_StopCar();
        }
        return;
    }

    /* 两侧同时触边时先脱离角点；不要在这里使用阻塞延时。 */
    if (left_edge && right_edge)
    {
        /* 左右同时触边通常意味着斜压角点，优先采用完整的后退-转向流程。 */
        edge_escape_state = 1U;
        edge_escape_tick = now;
        avoid_direction = 0U;
        clear_count = 0U;
        Car_Move(-edge_reverse_speed, 0.0f, 0.0f);
        return;
    }

    /* 正前方触边时后退一小段，避免每次调用都停在边线上。 */
    if (center_edge)
    {
        /* 中间触边表示车头靠近边界，先后退到安全距离。 */
        edge_escape_state = 4U;
        edge_escape_tick = now;
        avoid_direction = 0U;
        clear_count = 0U;
        Car_Move(-edge_reverse_speed, 0.0f, 0.0f);
        return;
    }
    else if (left_edge)
    {
        /* 左侧触边后向右纠偏，但保留前进速度，避免压线时几乎原地不动。 */
        avoid_direction = 1U;
        clear_count = 0U;
        avoid_until = now + keep_hold_ms;
        Car_Move(keep_forward_speed, -keep_lateral_speed, 0.0f);
    }
    else if (right_edge)
    {
        /* 右侧触边后向左纠偏，同样保留前进速度。 */
        avoid_direction = 2U;
        clear_count = 0U;
        avoid_until = now + keep_hold_ms;
        Car_Move(keep_forward_speed, keep_lateral_speed, 0.0f);
    }
    else if (avoid_direction != 0U)
    {
        /*
         * 边线暂时消失时继续沿原方向纠偏，直到保持时间结束且连续多次清线。
         * 这样不会在边线阈值附近左右来回切换。
         */
        if ((int32_t)(now - avoid_until) < 0)
        {
            clear_count = 0U;
        }
        else if (clear_count < 4U)
        {
            clear_count++;
        }
        else
        {
            avoid_direction = 0U;
        }

        if (avoid_direction == 1U)
        {
            Car_Move(keep_forward_speed, -keep_lateral_speed, 0.0f);
        }
        else if (avoid_direction == 2U)
        {
            Car_Move(keep_forward_speed, keep_lateral_speed, 0.0f);
        }
    }
}

static uint8_t Visual_FrontBoundaryDetected(void)
{
    /* 视觉搜索期间只检查中间两路传感器，用于判断车头是否碰到前方边界。 */
    Track_Control(1);

    return (track_values[3] > 1300 ||
            track_values[4] > 1300);
}

/*
 * USART1 使用单字节中断接收，主循环从环形缓冲区提取最新完整视觉帧。
 */
static uint16_t Visual_RxNext(uint16_t index)
{
    /* 环形缓冲区到达末尾后回到 0，避免访问数组范围之外。 */
    index++;
    if (index >= VISUAL_RX_BUFFER_SIZE)
    {
        index = 0U;
    }
    return index;
}

void Visual_Rx_Init(void)
{
    /*
     * 初始化前暂时关中断，确保主循环清零 head/tail 时不会与接收中断竞争。
     * 清零完成后立即挂起单字节中断接收，供后续 START 帧使用。
     */
    __disable_irq();
    visual_rx_head = 0U;
    visual_rx_tail = 0U;
    visual_rx_overflow = 0U;
    visual_line_length = 0U;
    visual_discard_line = 0U;
    __enable_irq();

    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&visual_rx_byte, 1U);
}

void Visual_Rx_Clear(void)
{
    /* 丢弃当前缓存中的旧数据，只从当前 head 位置开始接收新的视觉命令。 */
    __disable_irq();
    visual_rx_tail = visual_rx_head;
    visual_rx_overflow = 0U;
    __enable_irq();

    visual_line_length = 0U;
    visual_discard_line = 0U;
}

void Visual_RxCpltCallback(void)
{
    /*
     * 每收到一个字节就进入一次回调：先写入当前 head，再推进 head。
     * 若推进后追上 tail，说明缓存已满，丢掉最旧字节以保留最新数据。
     */
    uint16_t next = Visual_RxNext(visual_rx_head);

    visual_rx_buffer[visual_rx_head] = visual_rx_byte;
    if (next == visual_rx_tail)
    {
        /* 缓冲区满时丢弃最旧字节，保证控制循环拿到最新视觉数据。 */
        visual_rx_tail = Visual_RxNext(visual_rx_tail);
        visual_rx_overflow = 1U;
    }
    visual_rx_head = next;

    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&visual_rx_byte, 1U);
}

void Visual_RxErrorCallback(void)
{
    /*
     * 串口溢出等错误可能使 HAL 停止当前接收事务；记录错误并重新挂接
     * 单字节接收，让后续视觉帧仍能恢复进入环形缓存。
     */
    visual_rx_overflow = 1U;

    /*
     * ORE会让HAL结束当前接收事务。错误标志由HAL_UART_IRQHandler清除，
     * 这里负责立即重新挂上单字节接收，避免后续帧无人接收。
     */
    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&visual_rx_byte, 1U);
}

static uint8_t Visual_RxPop(uint8_t *byte)
{
    /* 从 tail 取出一个字节；返回 0 表示空，返回 1 表示成功取出。 */
    uint16_t tail = visual_rx_tail;

    if (tail == visual_rx_head)
    {
        return 0U;
    }

    *byte = visual_rx_buffer[tail];
    visual_rx_tail = Visual_RxNext(tail);
    return 1U;
}

/*
 * 取出当前已经收到的所有完整行，只保留最后一行。
 * 视觉帧可能比控制周期更快，不能把旧dx逐条排队执行。
 */
uint8_t Visual_ReadLatestLine(char *line)
{
    /*
     * 将缓存中的字节按换行符组帧。一次调用可能读到多行，
     * 通过反复覆盖 line 只保留最后一行，避免控制循环执行过期 dx。
     */
    uint8_t byte;
    uint8_t has_line = 0U;

    if (visual_rx_overflow != 0U)
    {
        /* 发生覆盖后放弃当前半行，直到下一个换行符再恢复正常组帧。 */
        visual_rx_overflow = 0U;
        visual_line_length = 0U;
        visual_discard_line = 1U;
    }

    while (Visual_RxPop(&byte) != 0U)
    {
        if (byte == '\r')
        {
            continue;
        }

        if (byte == '\n')
        {
            if (visual_discard_line == 0U &&
                visual_line_length > 0U)
            {
                visual_line_buffer[visual_line_length] = '\0';
                strcpy(line, visual_line_buffer);
                has_line = 1U;
            }

            visual_line_length = 0U;
            visual_discard_line = 0U;
            continue;
        }

        if (visual_discard_line != 0U)
        {
            continue;
        }

        if (visual_line_length < VISUAL_LINE_LENGTH - 1U)
        {
            visual_line_buffer[visual_line_length++] = (char)byte;
        }
        else
        {
            /* 超过协议允许长度的帧直接丢弃，防止字符串缓冲区越界。 */
            visual_line_length = 0U;
            visual_discard_line = 1U;
        }
    }

    return has_line;
}

static uint8_t Visual_ParseSignedInt(const char *line, uint8_t *index, int *value)
{
    int sign = 1;
    int number = 0;
    uint8_t parsed = 0U;

    if (line[*index] == '-')
    {
        sign = -1;
        (*index)++;
    }
    else if (line[*index] == '+')
    {
        (*index)++;
    }

    while (line[*index] >= '0' && line[*index] <= '9')
    {
        parsed = 1U;
        number = number * 10 + line[*index] - '0';
        (*index)++;
    }

    if (parsed == 0U)
    {
        return 0U;
    }

    *value = sign * number;
    return 1U;
}

uint8_t Visual_ParseLine(const char *line, int *dx, int *can_hit,
                                uint8_t *red_present, int *red_dx)
{
    /*
     * 解析树莓派发送的单行协议：
     *   none       表示暂时没有目标；none,red,red_dx 表示只有红球；
     *   ok         表示视觉端已准备好或流程确认；
     *   dx,0/1     表示横向误差以及当前是否允许戳击；
     *   dx,0/1,red,red_dx 表示同时检测到红球。
     * 返回值只描述命令类型，具体数据通过输出参数提供。
     */
    uint8_t index = 0U;
    int target_dx = 0;
    int target_can_hit = 0;
    int detected_red_dx = 0;

    if (strcmp(line, "none") == 0)
    {
        *red_present = 0U;
        return VISUAL_COMMAND_NONE;
    }

    if (strncmp(line, "none,red,", 9U) == 0)
    {
        index = 9U;
        if (Visual_ParseSignedInt(line, &index, &detected_red_dx) == 0U ||
            line[index] != '\0')
        {
            return VISUAL_COMMAND_INVALID;
        }

        *red_present = 1U;
        *red_dx = detected_red_dx;
        return VISUAL_COMMAND_NONE;
    }

    if (strcmp(line, "ok") == 0)
    {
        *red_present = 0U;
        return VISUAL_COMMAND_OK;
    }

    if (Visual_ParseSignedInt(line, &index, &target_dx) == 0U ||
        line[index++] != ',')
    {
        return VISUAL_COMMAND_INVALID;
    }

    if (line[index] == '0')
    {
        target_can_hit = 0;
    }
    else if (line[index] == '1')
    {
        target_can_hit = 1;
    }
    else
    {
        return VISUAL_COMMAND_INVALID;
    }

    index++;
    if (line[index] == '\0')
    {
        *dx = target_dx;
        *can_hit = target_can_hit;
        *red_present = 0U;
        return VISUAL_COMMAND_TARGET;
    }

    if (line[index++] != ',' ||
        line[index++] != 'r' ||
        line[index++] != 'e' ||
        line[index++] != 'd' ||
        line[index++] != ',')
    {
        return VISUAL_COMMAND_INVALID;
    }

    if (Visual_ParseSignedInt(line, &index, &detected_red_dx) == 0U ||
        line[index] != '\0')
    {
        return VISUAL_COMMAND_INVALID;
    }

    *dx = target_dx;
    *can_hit = target_can_hit;
    *red_present = 1U;
    *red_dx = detected_red_dx;
    return VISUAL_COMMAND_TARGET;
}

/* 将视觉横向误差转换成云台角度修正，正误差表示目标在画面右侧。 */
static void Visual_AdjustServo(float *servo_angle, int dx, float gain, int deadband, float max_step)
{
    /*
     * 将图像横向误差映射到云台逻辑角度：先扣除死区，再乘增益，
     * 最后限制单帧最大变化量和机械角度范围，避免视觉噪声造成剧烈转动。
     */
    float correction = 0.0f;

    if (dx > deadband)
    {
        /* 目标在图像右侧，修正量为正，随后会减小舵机逻辑角度。 */
        correction = (float)(dx - deadband) * gain;
    }
    else if (dx < -deadband)
    {
        /* 目标在图像左侧，修正量为负，随后会增大舵机逻辑角度。 */
        correction = (float)(dx + deadband) * gain;
    }
    else
    {
        return;
    }

    if (correction > max_step)
    {
        correction = max_step;
    }
    else if (correction < -max_step)
    {
        correction = -max_step;
    }

    /* Servo.h规定舵机角度增大为向左，目标在右侧时应减小舵机角度。 */
    *servo_angle -= correction;

    if (*servo_angle < VISUAL_SERVO_MIN_ANGLE)
    {
        *servo_angle = VISUAL_SERVO_MIN_ANGLE;
    }
    else if (*servo_angle > VISUAL_SERVO_MAX_ANGLE)
    {
        *servo_angle = VISUAL_SERVO_MAX_ANGLE;
    }

    Servo_SetAngle(2, *servo_angle);
}

/* 快速停车：目标速度和当前PWM同时清零，避免刺针时车轮仍有残余转速。 */
static void Scene_StopCar(void)
{
    /*
     * 场内动作的统一急停接口：既清零速度环目标，也清零当前 PWM。
     * 只清目标速度可能仍会因控制器残留输出继续滑行，因此两者都要清零。
     */
    Set_Target_Speed(1, 0);
    Set_Target_Speed(2, 0);
    Set_Target_Speed(3, 0);
    Set_Target_Speed(4, 0);
    Set_Speed(1, 0);
    Set_Speed(2, 0);
    Set_Speed(3, 0);
    Set_Speed(4, 0);
}

/*
 * 按云台当前指向解算底盘速度。
 * 舵机角度是逻辑角度，不是TIM3的实际半角脉宽角度，直接使用Servo.h规定的范围。
 */
static void Visual_MoveTowardTarget(float servo_angle, float speed)
{
    /*
     * 根据云台相对车头的方位，把前进速度分解为底盘前进 vx 和横移 vy。
     * 这里不旋转车身，车辆通过全向底盘平移接近目标；执行后调用 Keep_In 保证不越界。
     */
    float bearing_deg = (float)servo_angle - VISUAL_CENTER_ANGLE;
    float bearing_rad;
    float vx;
    float vy;

    if (bearing_deg > 90.0f)
    {
        /* 限制在前方半平面，防止异常云台角度产生倒车分量。 */
        bearing_deg = 90.0f;
    }
    else if (bearing_deg < -90.0f)
    {
        bearing_deg = -90.0f;
    }

    bearing_rad = bearing_deg * PI / 180.0f;
    vx = speed * cosf(bearing_rad);
    vy = speed * sinf(bearing_rad) * VISUAL_LATERAL_GAIN;

    if (vy > VISUAL_MAX_LATERAL_SPEED)
    {
        vy = VISUAL_MAX_LATERAL_SPEED;
    }
    else if (vy < -VISUAL_MAX_LATERAL_SPEED)
    {
        vy = -VISUAL_MAX_LATERAL_SPEED;
    }

    Car_Move(vx, vy, 0.0f);
    Keep_In();
}

/* 返回 Visual_MoveTowardTarget 实际产生的底盘运动方向，左侧为正。 */
static float Visual_GetMotionBearing(float servo_angle)
{
    float bearing_deg = (float)servo_angle - VISUAL_CENTER_ANGLE;
    float bearing_rad;
    float vx;
    float vy;

    if (bearing_deg > 90.0f)
    {
        bearing_deg = 90.0f;
    }
    else if (bearing_deg < -90.0f)
    {
        bearing_deg = -90.0f;
    }

    bearing_rad = bearing_deg * PI / 180.0f;
    vx = cosf(bearing_rad);
    vy = sinf(bearing_rad) * VISUAL_LATERAL_GAIN;

    if (vy > VISUAL_MAX_LATERAL_SPEED)
    {
        vy = VISUAL_MAX_LATERAL_SPEED;
    }
    else if (vy < -VISUAL_MAX_LATERAL_SPEED)
    {
        vy = -VISUAL_MAX_LATERAL_SPEED;
    }

    return atan2f(vy, vx) * 180.0f / PI;
}

/*
 * 出针后的推进：严格沿云台当前指向（即钢针轴线）平移。
 * 与 Visual_MoveTowardTarget 的区别是不放大横向分量——
 * 接近阶段放大横移是为了更好的画面跟随，而戳击阶段方向必须和针轴一致，
 * 否则针尖会随推进横向走偏，最后变成车身去顶气球。
 */
static void Visual_PushAlongNeedle(float servo_angle, float speed)
{
    float bearing_deg = (float)servo_angle - VISUAL_CENTER_ANGLE;
    float bearing_rad;

    if (bearing_deg > 90.0f)
    {
        /* 限制在前方半平面，防止异常云台角度产生倒车分量。 */
        bearing_deg = 90.0f;
    }
    else if (bearing_deg < -90.0f)
    {
        bearing_deg = -90.0f;
    }

    bearing_rad = bearing_deg * PI / 180.0f;
    Car_Move(speed * cosf(bearing_rad), speed * sinf(bearing_rad), 0.0f);
    Keep_In();
}

/*
 * 场内戳气球逻辑：
 * 1. 发送 START，等待树莓派持续发送视觉数据；
 * 2. 根据 dx 调整云台2，并让小车无旋转平移靠近目标；
 * 3. can_hit 为1表示“已进入可戳窗口，但出针后仍差约3cm”：
 *    立即停车 → 用最多150ms把针轴收正 → 出针 → 沿针轴以6cm/s推进，
 *    目标消失（已扎破）或到达1.2s上限后收针、后退；
 * 4. 收到 none 时云台完整扫描，同时让小车低速扫过当前方向；
 * 5. 前方触边后退开并掉头，反向覆盖场地。
 */
void Hit_Balloons(void)
{
    /*
     * 场内戳球主状态机。主循环每 5 ms 读取最新视觉帧并更新动作：
     * 跟踪阶段允许底盘平移，戳击阶段保持底盘静止，搜索阶段扫描云台和场地。
     */
    char line[VISUAL_LINE_LENGTH];
    int dx = 0;
    int can_hit = 0;
    uint8_t red_present = 0U;
    int red_dx = 0;
    float servo_angle = VISUAL_CENTER_ANGLE;
    int search_angle = (int)VISUAL_CENTER_ANGLE;
    int search_step = VISUAL_SEARCH_STEP;
    int target_valid = 0;
    int searching = 0;
    uint8_t command;
    /* hit_state：HIT_IDLE跟踪，HIT_ALIGN停车快速对准，HIT_EXTEND出针等待，
     * HIT_PUSH保持出针推进，HIT_RETRACT收针稳定，HIT_BACKOFF后退脱离。 */
    uint8_t hit_state = HIT_IDLE;
    uint8_t hit_align_count = 0U; /* 对准达标标志：1 表示本帧 |dx| 已进入容差。 */
    uint8_t jab_needle_out = 1U;  /* 反复戳的当前相位：1=针在135°，0=已回抽到87°。 */
    uint8_t gimbal_limit_count = 0U;
    uint8_t at_gimbal_limit = 0U; /* 当前视觉目标是否位于云台机械限位外侧。 */
    uint8_t forced_forward_search = 0U;
    uint8_t gimbal_escape_state = 0U;
    /* coverage_state：0正常搜索，1前方后退，2掉头，3掉头稳定。 */
    uint8_t coverage_state = 0U;
    uint8_t red_avoid_state = VISUAL_RED_AVOID_IDLE;
    int8_t red_avoid_direction = 1; /* -1:向右，1:向左；首次默认向左。 */
    uint32_t red_avoid_tick = 0U;
    uint32_t red_last_seen_tick = 0U;
    uint32_t red_center_tick = 0U;
    uint8_t red_conflict_count = 0U;
    uint8_t red_clear_count = 0U;
    float red_bearing = 0.0f;
    float target_bearing = 0.0f;
    float motion_bearing = 0.0f;
    float bearing_delta = 0.0f;
    uint8_t red_conflict = 0U;
    uint32_t state_tick = HAL_GetTick();    /* 当前状态开始时间，用于超时和动作保持。 */
    uint32_t target_tick = HAL_GetTick();   /* 最近一次有效目标帧时间。 */
    uint32_t search_tick = HAL_GetTick();   /* 最近一次云台搜索步进时间。 */
    uint32_t coverage_tick = HAL_GetTick(); /* 场地覆盖子状态开始时间。 */
    uint32_t now;

    red_last_seen_tick = HAL_GetTick();

    /* 初始化云台和针机构位置，确保每次进入场内流程都有一致起始姿态。 */
    Servo_SetAngle(2, (int)VISUAL_CENTER_ANGLE);
    Servo_SetAngle(1, 87);
    Track_Control(1);
    /* USART1必须先挂上单字节中断接收，再通知树莓派开始发送视觉帧。 */
    Visual_Rx_Init();
    Visual_Rx_Clear();
    HAL_Delay(30);
    Serial_SendString("START");

    while (1)
    {
        now = HAL_GetTick();

        if (Visual_ReadLatestLine(line) != 0U)
        {
            /* 一次只处理缓存中最新完整帧，避免旧视觉数据阻塞实时控制。 */
            command = Visual_ParseLine(line, &dx, &can_hit,
                                       &red_present, &red_dx);

            if (command != VISUAL_COMMAND_INVALID)
            {
                uint8_t motion_active = 0U;

                /* 当前底盘实际会沿目标方向接近，或在无目标时直行搜索。 */
                if (hit_state == HIT_IDLE && coverage_state == 0U &&
                    gimbal_escape_state == 0U)
                {
                    if (command == VISUAL_COMMAND_TARGET && can_hit == 0)
                    {
                        motion_active = 1U;
                        motion_bearing = Visual_GetMotionBearing(servo_angle);
                        target_bearing = servo_angle - VISUAL_CENTER_ANGLE -
                                         (float)dx * VISUAL_SERVO_TRACK_GAIN;
                    }
                    else if (command == VISUAL_COMMAND_NONE)
                    {
                        motion_active = 1U;
                        motion_bearing = 0.0f;
                    }
                }

                if (target_bearing > 90.0f)
                {
                    target_bearing = 90.0f;
                }
                else if (target_bearing < -90.0f)
                {
                    target_bearing = -90.0f;
                }

                if (red_present != 0U)
                {
                    red_last_seen_tick = now;
                    red_clear_count = 0U;
                    red_bearing = servo_angle - VISUAL_CENTER_ANGLE -
                                  (float)red_dx * VISUAL_SERVO_TRACK_GAIN;
                    if (red_bearing > 90.0f)
                    {
                        red_bearing = 90.0f;
                    }
                    else if (red_bearing < -90.0f)
                    {
                        red_bearing = -90.0f;
                    }

                    red_conflict = 0U;
                    if (motion_active != 0U &&
                        red_bearing >= -VISUAL_RED_FRONT_DANGER_DEG &&
                        red_bearing <= VISUAL_RED_FRONT_DANGER_DEG)
                    {
                        bearing_delta = red_bearing - motion_bearing;
                        if (bearing_delta >= -VISUAL_RED_MOTION_CONFLICT_DEG &&
                            bearing_delta <= VISUAL_RED_MOTION_CONFLICT_DEG)
                        {
                            red_conflict = 1U;
                        }

                        /* 有目标时，红球还必须落在目标通行方向附近。 */
                        if (command == VISUAL_COMMAND_TARGET)
                        {
                            bearing_delta = red_bearing - target_bearing;
                            if (bearing_delta > VISUAL_RED_LANE_HALF_DEG +
                                VISUAL_RED_MOTION_CONFLICT_DEG ||
                                bearing_delta < -(VISUAL_RED_LANE_HALF_DEG +
                                                  VISUAL_RED_MOTION_CONFLICT_DEG))
                            {
                                red_conflict = 0U;
                            }
                        }
                    }

                    if (red_avoid_state != VISUAL_RED_AVOID_WAIT_CLEAR)
                    {
                        if (red_conflict != 0U)
                        {
                            if (red_conflict_count < VISUAL_RED_CONFLICT_FRAMES)
                            {
                                red_conflict_count++;
                            }
                        }
                        else
                        {
                            red_conflict_count = 0U;
                        }
                    }

                    if (red_avoid_state == VISUAL_RED_AVOID_IDLE &&
                        red_conflict_count >= VISUAL_RED_CONFLICT_FRAMES)
                    {
                        /* 连续确认会撞红球后才锁定一次避让。 */
                        red_avoid_state = VISUAL_RED_AVOID_OBSERVE;
                        red_avoid_tick = now;
                        red_center_tick = 0U;
                        if (red_bearing > VISUAL_RED_CENTER_BEARING_DEG)
                        {
                            red_avoid_direction = -1;
                        }
                        else if (red_bearing < -VISUAL_RED_CENTER_BEARING_DEG)
                        {
                            red_avoid_direction = 1;
                        }
                        else if (command == VISUAL_COMMAND_TARGET)
                        {
                            red_avoid_direction = (target_bearing >= 0.0f) ? -1 : 1;
                        }
                        can_hit = 0;
                        searching = 0;
                        red_conflict_count = 0U;
                        Scene_StopCar();
                    }
                }
                else
                {
                    red_conflict_count = 0U;
                    if (red_avoid_state == VISUAL_RED_AVOID_WAIT_CLEAR)
                    {
                        if (red_clear_count < VISUAL_RED_CLEAR_FRAMES)
                        {
                            red_clear_count++;
                        }
                        if (red_clear_count >= VISUAL_RED_CLEAR_FRAMES)
                        {
                            red_avoid_state = VISUAL_RED_AVOID_IDLE;
                            red_clear_count = 0U;
                        }
                    }
                }
            }

            if ((coverage_state == 0U ||
                 (coverage_state == 2U &&
                  (uint32_t)(now - coverage_tick) >= VISUAL_TURN_SCAN_START_MS)) &&
                gimbal_escape_state == 0U &&
                command == VISUAL_COMMAND_TARGET)
            {
                /* 掉头超过90°后发现目标，立即退出掉头，复用正常跟踪流程。 */
                if (coverage_state == 2U)
                {
                    coverage_state = 0U;
                    coverage_tick = now;
                }

                /* 目标帧：先判断云台是否仍在机械限位外侧，再进入跟踪/戳击状态。 */
                at_gimbal_limit =
                    ((servo_angle <= VISUAL_SERVO_MIN_ANGLE +
                                         VISUAL_GIMBAL_LIMIT_ANGLE_MARGIN &&
                      dx > VISUAL_GIMBAL_LIMIT_DX) ||
                     (servo_angle >= VISUAL_SERVO_MAX_ANGLE -
                                         VISUAL_GIMBAL_LIMIT_ANGLE_MARGIN &&
                      dx < -VISUAL_GIMBAL_LIMIT_DX));

                if (forced_forward_search != 0U && at_gimbal_limit != 0U)
                {
                    /* 脱困后的目标仍在限位外侧：继续直行，不重复进入后退状态。 */
                    gimbal_limit_count = 0U;
                    target_valid = 0;
                    searching = 1;
                    can_hit = 0;
                }
                else
                {
                    if (forced_forward_search != 0U)
                    {
                        /* 目标已离开限位外侧，恢复普通跟踪和可戳流程。 */
                        forced_forward_search = 0U;
                        searching = 0;
                    }

                    /* 只在普通跟踪阶段累计限位计数；戳击动作进行中不打断。 */
                    if (hit_state == HIT_IDLE && at_gimbal_limit != 0)
                    {
                        if (gimbal_limit_count < VISUAL_GIMBAL_LIMIT_FRAMES)
                        {
                            gimbal_limit_count++;
                        }
                    }
                    else
                    {
                        gimbal_limit_count = 0U;
                    }

                    if (gimbal_limit_count >= VISUAL_GIMBAL_LIMIT_FRAMES)
                    {
                        /* 云台无法继续向目标修正，先后退再沿场地前进到边线掉头。 */
                        gimbal_escape_state = 1U;
                        state_tick = now;
                        target_valid = 0;
                        searching = 0;
                        can_hit = 0;
                        hit_state = HIT_IDLE;
                        hit_align_count = 0U;
                        gimbal_limit_count = 0U;
                        Scene_StopCar();
                    }
                    else if (hit_state == HIT_IDLE)
                    {
                        /* 普通跟踪状态：调整云台、记录目标时间，并按 can_hit 决定是否开始戳击流程。 */
                        Visual_AdjustServo(&servo_angle, dx, VISUAL_SERVO_TRACK_GAIN,
                                           VISUAL_SERVO_DEADBAND, 4.0f);

                        target_bearing = servo_angle - VISUAL_CENTER_ANGLE -
                                         (float)dx * VISUAL_SERVO_TRACK_GAIN;
                        if (target_bearing > 90.0f)
                        {
                            target_bearing = 90.0f;
                        }
                        else if (target_bearing < -90.0f)
                        {
                            target_bearing = -90.0f;
                        }

                        target_valid = 1;
                        searching = 0;
                        target_tick = now;

                        if (can_hit != 0 &&
                            red_avoid_state == VISUAL_RED_AVOID_IDLE)
                        {
                            /*
                             * 视觉判定进入可戳窗口：立刻停车开始极短对准。
                             * 这里不再向目标行驶，否则车身会先一步顶到气球。
                             */
                            Scene_StopCar();
                            hit_align_count = 0U;
                            hit_state = HIT_ALIGN;
                            state_tick = now;
                        }
                    }
                    else if (hit_state == HIT_ALIGN)
                    {
                        /*
                         * 快速对准：车体完全不动，只用云台把针轴收正。
                         * can_hit 只负责启动本次流程，后续单帧变为0不取消出针，
                         * 只在目标持续丢失时由定时段放弃出针。
                         */
                        Scene_StopCar();
                        Visual_AdjustServo(&servo_angle, dx, VISUAL_HIT_ALIGN_GAIN,
                                           VISUAL_HIT_ALIGN_DEADBAND, VISUAL_HIT_ALIGN_MAX_STEP);
                        target_valid = 1;
                        searching = 0;
                        target_tick = now;

                        if (dx >= -VISUAL_HIT_ALIGN_TOLERANCE &&
                            dx <= VISUAL_HIT_ALIGN_TOLERANCE)
                        {
                            /* 本帧已进入容差，定时段会立即结束对准并出针。 */
                            hit_align_count = 1U;
                        }
                    }
                    else if (hit_state == HIT_EXTEND || hit_state == HIT_PUSH)
                    {
                        /*
                         * 钢针已经伸出：推进阶段继续用小增益把针尖收向目标中心，
                         * 出针行程阶段只刷新目标时间，不再转动云台。
                         */
                        if (hit_state == HIT_PUSH)
                        {
                            Visual_AdjustServo(&servo_angle, dx,
                                               VISUAL_HIT_PUSH_TRIM_GAIN,
                                               VISUAL_HIT_PUSH_TRIM_DEADBAND,
                                               VISUAL_HIT_PUSH_TRIM_MAX_STEP);
                        }

                        target_valid = 1;
                        searching = 0;
                        target_tick = now;
                    }
                }
            }
            else if ((coverage_state == 0U ||
                      (coverage_state == 2U &&
                       (uint32_t)(now - coverage_tick) >= VISUAL_TURN_SCAN_START_MS)) &&
                     gimbal_escape_state == 0U &&
                     command == VISUAL_COMMAND_NONE)
            {
                /* none 不是错误：普通跟踪时启动云台扫描，戳击流程中按当前阶段处理。 */
                if (hit_state == HIT_ALIGN)
                {
                    /*
                     * 对准期间目标消失：先停车；若超过放弃时限，由定时段退出本次戳击，
                     * 避免对着空气伸针。
                     */
                    target_valid = 0;
                    searching = 0;
                    hit_align_count = 0U;
                    Scene_StopCar();
                }
                else if (hit_state == HIT_EXTEND || hit_state == HIT_PUSH)
                {
                    /*
                     * 钢针已经伸出时目标从画面消失，视觉端的判定就是“气球已被扎破”：
                     * 立即停车收针，再由定时段完成后退脱离。
                     */
                    target_valid = 0;
                    searching = 0;
                    Scene_StopCar();
                    Servo_SetAngle(1, 87);
                    hit_state = HIT_RETRACT;
                    state_tick = now;
                }
                else if (hit_state == HIT_IDLE)
                {
                    if (searching == 0)
                    {
                        search_angle = (int)servo_angle;
                        search_step = (search_angle >= (int)VISUAL_CENTER_ANGLE) ? -VISUAL_SEARCH_STEP : VISUAL_SEARCH_STEP;
                        search_tick = now;
                    }

                    target_valid = 0;
                    can_hit = 0;
                    searching = 1;
                    Scene_StopCar();
                }
            }
            else if ((coverage_state == 0U ||
                      (coverage_state == 2U &&
                       (uint32_t)(now - coverage_tick) >= VISUAL_TURN_SCAN_START_MS)) &&
                     gimbal_escape_state == 0U &&
                     command == VISUAL_COMMAND_OK)
            {
                /* ok 只在尚未进入戳击状态时清理搜索标志，避免打断正在进行的动作。 */
                if (hit_state == HIT_IDLE)
                {
                    target_valid = 0;
                    can_hit = 0;
                    /* 强制前进阶段收到ok仍需继续向前搜索，不能停在原地。 */
                    searching = (forced_forward_search != 0U) ? 1 : 0;
                }
            }
        }

        if (gimbal_escape_state == 1U &&
            (uint32_t)(now - state_tick) >= VISUAL_GIMBAL_BACKOFF_TIME_MS)
        {
            /* 云台脱困后转入强制前进搜索，直到前方边界触发场地覆盖掉头。 */
            /* 后退完成后改为向前扫到边线，再利用场地掉头覆盖另一侧。 */
            gimbal_escape_state = 0U;
            forced_forward_search = 1U;
            target_valid = 0;
            searching = 1;
            search_angle = (int)servo_angle;
            search_step = (search_angle >= (int)VISUAL_CENTER_ANGLE)
                              ? -VISUAL_SEARCH_STEP
                              : VISUAL_SEARCH_STEP;
            search_tick = now;
        }

        if (red_avoid_state == VISUAL_RED_AVOID_WAIT_CLEAR &&
            (uint32_t)(now - red_last_seen_tick) >= VISUAL_RED_FRAME_TIMEOUT_MS)
        {
            /* 视觉流中断时也必须释放红球锁，不能永久停在等待状态。 */
            red_avoid_state = VISUAL_RED_AVOID_IDLE;
            red_clear_count = 0U;
        }

        if (red_avoid_state == VISUAL_RED_AVOID_OBSERVE &&
            (uint32_t)(now - red_avoid_tick) >= VISUAL_RED_OBSERVE_TIME_MS)
        {
            if (red_conflict_count >= VISUAL_RED_CONFLICT_FRAMES)
            {
                red_avoid_state = VISUAL_RED_AVOID_TURN_AWAY;
                red_avoid_tick = now;
                red_center_tick = 0U;
            }
            else
            {
                red_avoid_state = (red_present != 0U)
                                      ? VISUAL_RED_AVOID_WAIT_CLEAR
                                      : VISUAL_RED_AVOID_IDLE;
                red_clear_count = 0U;
            }
        }
        else if (red_avoid_state == VISUAL_RED_AVOID_TURN_AWAY &&
                 (uint32_t)(now - red_avoid_tick) >= VISUAL_RED_TURN_AWAY_TIME_MS)
        {
            red_avoid_state = VISUAL_RED_AVOID_TURN_BACK;
            red_avoid_tick = now;
            red_center_tick = 0U;
        }
        else if (red_avoid_state == VISUAL_RED_AVOID_TURN_BACK)
        {
            uint32_t turn_back_elapsed = (uint32_t)(now - red_avoid_tick);

            if (turn_back_elapsed >= VISUAL_RED_TURN_BACK_MAX_MS)
            {
                red_avoid_state = (red_present != 0U)
                                      ? VISUAL_RED_AVOID_WAIT_CLEAR
                                      : VISUAL_RED_AVOID_IDLE;
                red_center_tick = 0U;
            }
            else if (turn_back_elapsed >= VISUAL_RED_TURN_BACK_MIN_MS &&
                     target_valid != 0 &&
                     (uint32_t)(now - target_tick) < VISUAL_TARGET_TIMEOUT_MS)
            {
                if (target_bearing >= -VISUAL_RED_CENTER_BEARING_DEG &&
                    target_bearing <= VISUAL_RED_CENTER_BEARING_DEG)
                {
                    if (red_center_tick == 0U)
                    {
                        red_center_tick = now;
                    }
                    else if ((uint32_t)(now - red_center_tick) >=
                             VISUAL_RED_TURN_BACK_CENTER_HOLD_MS)
                    {
                        red_avoid_state = (red_present != 0U)
                                              ? VISUAL_RED_AVOID_WAIT_CLEAR
                                              : VISUAL_RED_AVOID_IDLE;
                        red_center_tick = 0U;
                    }
                }
                else
                {
                    red_center_tick = 0U;
                }
            }
            else if (turn_back_elapsed >= VISUAL_RED_TURN_BACK_MIN_MS)
            {
                red_avoid_state = (red_present != 0U)
                                      ? VISUAL_RED_AVOID_WAIT_CLEAR
                                      : VISUAL_RED_AVOID_IDLE;
                red_center_tick = 0U;
            }
        }

        if (hit_state == HIT_ALIGN &&
            (hit_align_count != 0U ||
             (uint32_t)(now - state_tick) >= VISUAL_HIT_ALIGN_MAX_MS))
        {
            /*
             * 对准达标（|dx| 已进入容差）或到达 150ms 时限：
             * 先确认目标仍在有效期内，再驱动一号舵机出针。
             */
            hit_align_count = 0U;

            if ((uint32_t)(now - target_tick) >= VISUAL_HIT_LOST_ABORT_MS)
            {
                /* 目标在对准期间已经消失，放弃本次出针，回到搜索。 */
                hit_state = HIT_IDLE;
                target_valid = 0;
                searching = 1;
                search_angle = (int)servo_angle;
                search_step = (search_angle >= (int)VISUAL_CENTER_ANGLE)
                                  ? -VISUAL_SEARCH_STEP
                                  : VISUAL_SEARCH_STEP;
                search_tick = now;
            }
            else
            {
                Servo_SetAngle(1, 135);
                hit_state = HIT_EXTEND;
                state_tick = now;
            }
        }
        else if (hit_state == HIT_EXTEND &&
                 (uint32_t)(now - state_tick) >= VISUAL_HIT_NEEDLE_OUT_MS)
        {
            /* 钢针行程走完，开始沿针轴推进；此时针就在 135°，相位从“伸出”开始。 */
            hit_state = HIT_PUSH;
            jab_needle_out = 1U;
            state_tick = now;
        }
        else if (hit_state == HIT_PUSH &&
                 (uint32_t)(now - state_tick) >= VISUAL_HIT_PUSH_MAX_MS)
        {
            /* 推进到上限仍未收到目标消失（没扎破）：停车收针，交给后退段重新接近。 */
            Scene_StopCar();
            Servo_SetAngle(1, 87);
            hit_state = HIT_RETRACT;
            state_tick = now;
        }
        else if (hit_state == HIT_RETRACT &&
                 (uint32_t)(now - state_tick) >= VISUAL_HIT_RETRACT_MS)
        {
            /* 收针稳定结束，后退拉开距离后再恢复跟踪/搜索。 */
            target_valid = 0;
            can_hit = 0;
            searching = 0;
            hit_state = HIT_BACKOFF;
            state_tick = now;
        }

        /*
         * 推进过程中的反复戳节拍：针到位后先回抽（87°）再扎出（135°），
         * 每 JAB_IN_MS + JAB_OUT_MS 一个周期。相位由 state_tick 直接推算，
         * 只在相位翻转时写一次舵机，避免每个 5ms 周期都刷 CCR。
         */
        if (hit_state == HIT_PUSH)
        {
            uint32_t push_elapsed = (uint32_t)(now - state_tick);

            if (push_elapsed >= VISUAL_HIT_JAB_LEAD_MS)
            {
                uint32_t jab_phase =
                    (push_elapsed - VISUAL_HIT_JAB_LEAD_MS) %
                    (VISUAL_HIT_JAB_IN_MS + VISUAL_HIT_JAB_OUT_MS);
                uint8_t needle_out = (jab_phase >= VISUAL_HIT_JAB_IN_MS) ? 1U : 0U;

                if (needle_out != jab_needle_out)
                {
                    jab_needle_out = needle_out;
                    Servo_SetAngle(1, (needle_out != 0U) ? 135 : 87);
                }
            }
        }

        if (hit_state == HIT_IDLE && coverage_state == 0U &&
            Visual_FrontBoundaryDetected() != 0U)
        {
            /* 正常搜索时检测到车头边界，启动后退-掉头覆盖流程。 */
            coverage_state = 1U;
            coverage_tick = now;
            target_valid = 0;
            searching = 0;
            can_hit = 0;
            forced_forward_search = 0U;
            gimbal_escape_state = 0U;
            Scene_StopCar();
        }

        if (gimbal_escape_state == 1U)
        {
            /* 云台限位脱困第一阶段：只后退，不接受新的视觉目标。 */
            Car_Move(-VISUAL_GIMBAL_BACKOFF_SPEED, 0.0f, 0.0f);
        }
        else if (coverage_state == 0U &&
                 red_avoid_state == VISUAL_RED_AVOID_OBSERVE &&
                 hit_state == HIT_IDLE)
        {
            /* 观察期短暂停车，过滤红球误检。 */
            Scene_StopCar();
        }
        else if (coverage_state == 0U &&
                 red_avoid_state == VISUAL_RED_AVOID_TURN_AWAY &&
                 hit_state == HIT_IDLE)
        {
            /* 先向红球反方向原地转开，给车身留出余量。 */
            Car_Move(0.0f, 0.0f,
                     (float)red_avoid_direction * VISUAL_RED_TURN_AWAY_OMEGA);
            Keep_In();
        }
        else if (coverage_state == 0U &&
                 red_avoid_state == VISUAL_RED_AVOID_TURN_BACK &&
                 hit_state == HIT_IDLE)
        {
            float turn_back_omega = 0.0f;
            uint32_t turn_back_elapsed = (uint32_t)(now - red_avoid_tick);

            if (turn_back_elapsed < VISUAL_RED_TURN_BACK_MIN_MS)
            {
                /* 至少反向转回一段，确保车辆确实开始回正。 */
                turn_back_omega = (float)(-red_avoid_direction) *
                                  VISUAL_RED_TURN_BACK_MIN_OMEGA;
            }
            else if (target_valid != 0 &&
                     (uint32_t)(now - target_tick) < VISUAL_TARGET_TIMEOUT_MS)
            {
                turn_back_omega = target_bearing * VISUAL_RED_TURN_BACK_GAIN;
                if (turn_back_omega > VISUAL_RED_TURN_BACK_MAX_OMEGA)
                {
                    turn_back_omega = VISUAL_RED_TURN_BACK_MAX_OMEGA;
                }
                else if (turn_back_omega < -VISUAL_RED_TURN_BACK_MAX_OMEGA)
                {
                    turn_back_omega = -VISUAL_RED_TURN_BACK_MAX_OMEGA;
                }
            }

            Car_Move(0.0f, 0.0f, turn_back_omega);
            Keep_In();
        }
        else if (hit_state == HIT_ALIGN || hit_state == HIT_EXTEND ||
                 hit_state == HIT_RETRACT)
        {
            /* 对准、出针行程、收针稳定三个阶段车体都保持静止。 */
            Scene_StopCar();
        }
        else if (hit_state == HIT_PUSH)
        {
            /*
             * 钢针伸出后沿针轴推进：前 LEAD 段用 6cm/s 把针尖送到球面，
             * 之后放慢到 3cm/s，配合上面的出/收节拍一边走一边反复扎。
             * 目标帧超时（视觉断流）时立即停车，避免用过期方向继续顶。
             */
            if (target_valid != 0 &&
                (uint32_t)(now - target_tick) < VISUAL_TARGET_TIMEOUT_MS)
            {
                float push_speed =
                    ((uint32_t)(now - state_tick) >= VISUAL_HIT_JAB_LEAD_MS)
                        ? VISUAL_HIT_JAB_SPEED
                        : VISUAL_HIT_PUSH_SPEED;

                Visual_PushAlongNeedle(servo_angle, push_speed);
            }
            else
            {
                Scene_StopCar();
            }
        }
        else if (coverage_state == 1U)
        {
            /* 场地覆盖第一阶段：直线后退，离开前方边界。 */
            if ((uint32_t)(now - coverage_tick) < VISUAL_BACKOFF_TIME_MS)
            {
                Car_Move(-VISUAL_BACKOFF_SPEED, 0.0f, 0.0f);
            }
            else
            {
                Scene_StopCar();
                coverage_state = 2U;
                coverage_tick = now;
            }
        }
        else if (coverage_state == 2U)
        {
            /* 场地覆盖第二阶段：先快速掉头90°，再减速并同步扫描云台。 */
            if ((uint32_t)(now - coverage_tick) < VISUAL_TURN_TIME_MS)
            {
                uint32_t turn_elapsed = (uint32_t)(now - coverage_tick);

                if (turn_elapsed < VISUAL_TURN_SCAN_START_MS)
                {
                    Car_Move(0.0f, 0.0f, VISUAL_TURN_OMEGA);
                }
                else
                {
                    /* 后半段保持原地低速转向，同时恢复云台扫描。 */
                    Car_Move(0.0f, 0.0f, VISUAL_TURN_SCAN_OMEGA);

                    if (searching == 0U)
                    {
                        target_valid = 0;
                        search_angle = (int)servo_angle;
                        search_step = (search_angle >= (int)VISUAL_CENTER_ANGLE)
                                          ? -VISUAL_SEARCH_STEP
                                          : VISUAL_SEARCH_STEP;
                        search_tick = now;
                        searching = 1;
                    }

                    if ((uint32_t)(now - search_tick) >= VISUAL_SEARCH_INTERVAL_MS)
                    {
                        search_angle += search_step;
                        if (search_angle >= VISUAL_SEARCH_MAX_ANGLE)
                        {
                            search_angle = VISUAL_SEARCH_MAX_ANGLE;
                            search_step = -VISUAL_SEARCH_STEP;
                        }
                        else if (search_angle <= VISUAL_SEARCH_MIN_ANGLE)
                        {
                            search_angle = VISUAL_SEARCH_MIN_ANGLE;
                            search_step = VISUAL_SEARCH_STEP;
                        }

                        servo_angle = (float)search_angle;
                        Servo_SetAngle(2, search_angle);
                        search_tick = now;
                    }
                }
            }
            else
            {
                Scene_StopCar();
                coverage_state = 3U;
                coverage_tick = now;
            }
        }
        else if (coverage_state == 3U)
        {
            /* 掉头后短暂停车稳定姿态，再恢复视觉搜索。 */
            Scene_StopCar();
            if ((uint32_t)(now - coverage_tick) >= VISUAL_TURN_SETTLE_MS)
            {
                coverage_state = 0U;
                target_valid = 0;
                searching = 1;
                forced_forward_search = 0U;
                gimbal_escape_state = 0U;
                gimbal_limit_count = 0U;
                search_angle = VISUAL_SEARCH_MIN_ANGLE;
                search_step = VISUAL_SEARCH_STEP;
                servo_angle = (float)search_angle;
                Servo_SetAngle(2, search_angle);
                search_tick = now;
            }
        }
        else if (hit_state == HIT_BACKOFF)
        {
            /* 戳击后的固定后退距离，用于让针尖和车体脱离目标附近。 */
            if ((uint32_t)(now - state_tick) < VISUAL_HIT_BACKOFF_TIME_MS)
            {
                Car_Move(-VISUAL_HIT_BACKOFF_SPEED, 0.0f, 0.0f);
                Keep_In();
            }
            else
            {
                Scene_StopCar();
                hit_state = HIT_IDLE;
                searching = 1;
                search_angle = (int)servo_angle;
                search_step = (search_angle >= (int)VISUAL_CENTER_ANGLE)
                                  ? -VISUAL_SEARCH_STEP
                                  : VISUAL_SEARCH_STEP;
                search_tick = now;
            }
        }
        else if (hit_state != HIT_IDLE)
        {
            Scene_StopCar();
        }
        else if (searching != 0)
        {
            /* 无目标时底盘低速前进，同时按固定周期步进云台角度。 */
            Car_Move(VISUAL_SEARCH_SPEED, 0.0f, 0.0f);
            Keep_In();
            if ((uint32_t)(now - search_tick) >= VISUAL_SEARCH_INTERVAL_MS)
            {
                search_angle += search_step;
                if (search_angle >= VISUAL_SEARCH_MAX_ANGLE)
                {
                    search_angle = VISUAL_SEARCH_MAX_ANGLE;
                    search_step = -VISUAL_SEARCH_STEP;
                }
                else if (search_angle <= VISUAL_SEARCH_MIN_ANGLE)
                {
                    search_angle = VISUAL_SEARCH_MIN_ANGLE;
                    search_step = VISUAL_SEARCH_STEP;
                }

                servo_angle = (float)search_angle;
                Servo_SetAngle(2, search_angle);
                search_tick = now;
            }
        }
        else if (target_valid != 0 &&
                 (uint32_t)(now - target_tick) < VISUAL_TARGET_TIMEOUT_MS)
        {
            /* 有效目标仍在超时窗口内：can_hit 时停车，否则按云台方位接近。 */
            if (can_hit != 0 &&
                red_avoid_state == VISUAL_RED_AVOID_IDLE)
            {
                Scene_StopCar();
            }
            else
            {
                Visual_MoveTowardTarget(servo_angle, VISUAL_APPROACH_SPEED);
            }
        }
        else
        {
            /* 目标帧过期或从未有效，清除目标状态并停车等待下一帧。 */
            target_valid = 0;
            Scene_StopCar();
        }

        HAL_Delay(5);
    }
}
void All_functions(void)
{
    /*
     * 比赛主流程入口：先完成赛道循迹，再进入场内。
     * 两个函数都是阻塞式任务，只有前一个流程返回后才会开始后一个流程。
     */
    /* 陀螺仪辅助的高速循迹 */
    LineFast_Run();

    /* 第一步：从循迹结束的位置走到 7 号点位，到位就停 */
    Nav_GotoOnce();
}

/* *
 * @brief 处理光敏电阻逻辑
 * @note 该函数在主循环中被调用，用于根据光敏电阻的状态决定是否执行其他功能。
 */
void Photoresistor(void)
{
    // 处理光敏电阻的逻辑
    if (car_start_flag==1)  // 处理启动的情况
    {
        All_functions();
    }
    else    // 处理未启动的情况
    {
        if((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_REG_BUSY) == 0U) //如果ADC不在转换中，则启动下一次转换
        {
            HAL_ADC_Start_IT(&hadc1);
        }
        HAL_Delay(10);
    }
}

