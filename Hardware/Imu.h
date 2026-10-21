#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>

/* ==================================================================
 * 10 轴 IMU（维特智能 WT/JY 系列）驱动 + 定角度控制
 *
 * 硬件：USART3，PC10=TX / PC11=RX，115200 8N1，模块配置为
 *       六轴算法 / 水平安装 / 只输出 加速度+角速度+角度 / 100Hz
 * 安装：模块倒装在车底，元件面朝地面，所以 yaw 需要取反才是
 *       "车辆逆时针为正"，见 IMU_YAW_SIGN_INIT。
 *
 * 注意：本模块只使用 Yaw。模块倒装后 Roll/Pitch/加速度都是反的，
 *       且模块内部卡尔曼滤波假设 Z 朝上，这些量不要直接使用。
 * ================================================================== */

/* ---------------- 调试 / 测试开关（改这里） ---------------- */

/*
 * IMU_TEST_MODE：
 *   0 = 关闭，走正常比赛流程（Photoresistor + IR_work）
 *   1 = 只显示 yaw，电机不动（验证数据流用）
 *   2 = 方向自检：标定 + 自动探测电机符号，然后停在显示状态
 *   3 = 定角度自测：+90 -> 0 -> -90 -> 0 无限循环
 * 比赛前必须改回 0。
 */
#define IMU_TEST_MODE        0

/* 1 = 在 OLED 上显示调试信息（会占用主循环约 10ms/次，调参时先关） */
#define IMU_DEBUG_OLED       0

/*
 * 航向来源：
 *   1 = 自己积分模块输出的角速度 GZ（推荐）
 *   0 = 用模块输出的欧拉角 Yaw
 *
 * 模块倒装在车底时，它自己的姿态解算是不对的（水平放置却报 Roll=-173°、
 * Pitch=36°），欧拉 Yaw 会在某一段角度里不敏感，表现为"推过去没反应"。
 * 角速度是原始量，不受这个问题影响，所以默认用积分。
 * 代价：会有缓慢零漂，模块的陀螺自动校准已经把零偏压得比较小。
 */
#define IMU_YAW_FROM_GYRO    1

/*
 * 1 = 上电后跑一次"最简自检"（约 6 秒，车会自己转 90° 再转回来）。
 * 0 = 跳过，直接进入 0° 保持。
 * 现在改用 OLED 看数据，自检关掉，省得每次上电车都动。
 */
#define IMU_BOOT_TEST        0

/*
 * 1 = 主循环只跑 IMU（Update / HoldStep / 显示），跳过光敏启动和红外。
 *     排查用：排除光敏误触发 All_functions() 把主循环卡死的可能。
 * 0 = 正常比赛流程（光敏启动 + 红外遥控）。
 */
#define IMU_LOOP_IMU_ONLY    0

/*
 * 航向符号。
 * 模块倒装（元件面朝地，+Z 指向地面），这里只决定"往哪边转算正"。
 * 判断方法：上电后车会自己定住 0°，用手推它一下——
 *   - 它转回来并稳住  -> 符号对了
 *   - 它越转越猛/自己打圈 -> 符号反了，把 (-1) 改成 (+1)
 * 另外若误差长期大于 90°，固件 3 秒后会自动停车并置 hold_fault，不会失控。
 */
#define IMU_YAW_SIGN_INIT    (-1)

/*
 * 电机方向符号，正常情况下永远是 +1。
 * 转向方向不对不要去动它，去改上面的 yaw 符号。
 */
#define IMU_MOTOR_SIGN_INIT  (+1)

/* ---------------- 航向保持参数（可调） ---------------- */

#define IMU_HOLD_ENTER_DEG    1.0f    /* 误差小于这个值就认为到位，停车 */
#define IMU_HOLD_EXIT_DEG     3.0f    /* 误差超过这个值才重新开始纠偏（迟滞） */
#define IMU_HOLD_OMEGA_MAX   100.0f   /* 保持时的最大角速度，别太大，容易被推着抖 */
#define IMU_HOLD_OMEGA_MIN    15.0f   /* 最小角速度，克服静摩擦 */
#define IMU_HOLD_KP           3.0f    /* 比例系数 */
#define IMU_HOLD_A_DECEL     250.0f   /* 制动减速度 */
#define IMU_HOLD_STALL_MS     800U    /* 连续这么久误差没压下去就判失控并停车 */

/*
 * 1 = 航向保持走"最简版"：只有一条比例项，没有死区/滤波/保护。
 *     用来验证 "yaw 变了电机到底会不会动"。
 * 0 = 走完整版：死区迟滞 + 平方根制动 + 失速保护。
 * 定位阶段先保持 1，确认闭环能跑之后改回 0。
 */
#define IMU_HOLD_SIMPLE       1

/* ---------------- 数据结构 ---------------- */

/* 模块最近一帧的原始数据，单位已换算好，便于调试观察 */
typedef struct
{
    float    roll;         /* 横滚角，度（倒装后为反值，勿用） */
    float    pitch;        /* 俯仰角，度（倒装后为反值，勿用） */
    float    yaw;          /* 模块原始航向角，-180~+180 */
    float    gx;           /* 角速度 X，度/秒 */
    float    gy;           /* 角速度 Y，度/秒 */
    float    gz;           /* 角速度 Z，度/秒 */
    float    ax;           /* 加速度 X，g */
    float    ay;           /* 加速度 Y，g */
    float    az;           /* 加速度 Z，g */
    float    temp;         /* 温度，摄氏度 */
    uint32_t frames;       /* 通过校验的帧数 */
    uint32_t sum_errors;   /* 校验失败次数 */
    uint32_t last_tick;    /* 最近一帧时间戳 */
    uint8_t  online;       /* 1 = 300ms 内收到过数据 */
} Imu_Data_t;

/* 上电静止标定的结果 */
typedef struct
{
    float    yaw_zero;     /* 静止时原始 yaw 均值 */
    float    yaw_std;      /* 静止时 yaw 标准差，用于判断是否真静止 */
    float    gz_bias;      /* 静止时 Z 轴角速度均值 */
    uint16_t samples;      /* 参与统计的样本数 */
    uint8_t  still;        /* 1 = 判定为静止 */
} Imu_Calib_t;

/* 车辆航向状态 */
typedef struct
{
    float   yaw;           /* 车辆航向，连续角，逆时针为正 */
    float   yaw_disp;      /* 低通后的显示值 */
    float   rate;          /* 车辆 Z 轴角速度，度/秒 */
    float   last_err;      /* 最近一次转到位的残余误差 */
    float   last_omega;    /* 最近一次输出的车体角速度指令 */
    int8_t  yaw_sign;      /* +1 / -1 */
    int8_t  motor_sign;    /* +1 / -1 / 0=未标定 */
    uint8_t online;        /* 1 = 模块在线 */
    uint8_t moving;        /* 1 = 定角度控制循环正在运行 */
    float   hold_target;   /* 当前要定住的角度 */
    uint8_t hold_on;       /* 1 = 航向保持已开启 */
    uint8_t hold_active;   /* 1 = 正在纠偏（在死区外） */
    uint8_t hold_fault;    /* 1 = 检测到失控，已自动停车 */
} Imu_State_t;

extern Imu_Data_t  imu_data;
extern Imu_Calib_t imu_calib;
extern Imu_State_t imu_state;

/* ---------------- 对外接口 ---------------- */

/* 初始化 USART3 接收，并等待模块上电启动（阻塞约 1.2s） */
void Imu_Init(void);

/*
 * 最简自检（阻塞约 4s），只用"车怎么动"来报告串口状态：
 *   A. 原地逆时针转 1.5s       -> 帧正常，陀螺仪和解析都没问题
 *   B. 原地顺时针转 1.5s       -> 收到字节但凑不成帧（波特率/干扰）
 *   C. 前进 0.3s 再后退 0.3s   -> 一个字节都没收到（接线/共地/引脚组错误）
 */
void Imu_MinimalTest(void);

/* 主循环周期调用：把中断收到的字节解析成数据 */
void Imu_Update(void);

/* 由 main.c 的 HAL_UART_RxCpltCallback 按 Instance 分发进来 */
void Imu_RxCpltCallback(void);
void Imu_RxErrorCallback(void);

/*
 * 只做静止统计，不转动、不置零。
 * 上电流程里已经不调用它（模块自带陀螺零偏校准），
 * 只有 IMU_TEST_MODE=1 的自测会用到。
 */
void Imu_StaticCalib(void);

/* 把当前车头方向置为 0°，并同步把模块自己的航向也清零 */
void Imu_ZeroYaw(void);

/* ---------------- 航向保持（持续闭环） ---------------- */

/*
 * 开启航向保持，target_deg 是要定住的角度（0 = 上电时的车头方向）。
 * 只在主循环里周期调用 Imu_HoldStep()，不阻塞。
 */
void Imu_HoldStart(float target_deg);

/* 主循环里每个周期调用一次；越快越好，10ms 以内最佳 */
void Imu_HoldStep(void);

/*
 * 暂停保持一段时间（毫秒）。按了前进/横移/手动旋转这类键时调用，
 * 让手动动作能生效，时间到后自动恢复保持。
 */
void Imu_HoldPause(uint32_t ms);

/* 关闭保持（停车），比如比赛流程开始前 */
void Imu_HoldStop(void);

/* 车辆航向（连续角，逆时针为正），控制用 */
float Imu_GetYaw(void);

/* 车辆 Z 轴角速度，度/秒 */
float Imu_GetRate(void);

uint8_t Imu_IsOnline(void);

/*
 * 纯计算：根据目标航向算出本周期应输出的车体角速度（逆时针为正）。
 * 不操作电机，供定角度控制和以后的导航模块复用。
 * err_out 可为 NULL；返回值为车体角速度指令。
 */
float Imu_HeadingOmega(float target_deg, float *err_out);

/*
 * 阻塞式转到目标航向并停住。
 * 目标按连续角处理，可以超过 ±180，内部走最短路径。
 * 返回：0 成功 / -1 陀螺离线 / -2 超时未收敛
 */
int Imu_TurnTo(float target_deg);

/*
 * 阻塞式相对转弯：在当前航向上再转 delta_deg。
 * 正数左转（逆时针），负数右转。返回码同 Imu_TurnTo。
 */
int Imu_TurnBy(float delta_deg);

/* 按 IMU_TEST_MODE 执行测试流程 */
void Imu_SelfTest(void);

/* 把调试信息显示到 OLED（10Hz 限流） */
void Imu_DebugToOled(void);

#endif
