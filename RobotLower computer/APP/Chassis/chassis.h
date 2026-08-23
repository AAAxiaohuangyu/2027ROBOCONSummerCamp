#ifndef CHASSIS_H
#define CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "M3508.h"
#include "StrategyAlogrithm.h"
#include "chassis_mecanum.h"
#include "yis512.h"

/*
底盘车体坐标与速度模型(底盘平面几何中心为坐标原点,单位:m/rad):

1. 车体系定义为 +X 向前、+Y 向左、+Wz 从车顶看逆时针,四轮数组顺序始终为
   FL(左前)、FR(右前)、RL(左后)、RR(右后),与 chassis_mecanum.h 保持一致。

2. 原始速度接口(ChassisSetVelocity)直接下发车体速度 Vx/Vy/Wz,方向由符号
   表达、大小由数值表达,不经过 S 曲线规划,信任调用者给出的目标。

3. 位移接口拆分为平移、偏航两个独立入口:ChassisSetTranslation 下发世界系
   绝对目标坐标 x/y,ChassisSetYaw 下发相对当前朝向的目标偏航 dyaw、内部
   换算成绝对目标 yaw_target_rad = 调用时刻的 pose.yaw_rad + dyaw;两者
   分别触发各自的重新规划,互不影响。平移拆分为 x、y 两条完全独立的七段
   S 曲线规划器,偏航单独一条,三者参数一致的部分互不耦合、跟踪方式完全
   一致:规划器与跟踪器都直接以 pose 的绝对值(x_m/y_m/yaw_rad)作为实际
   反馈,目标同为绝对量。

4. 本模块不做平移里程计积分,pose.x_m/y_m 由外部里程计/位姿融合模块按
   周期通过 ChassisSetPosition 写入;pose.yaw_rad 由 ChassisUpdate 每次
   调用时从传入的 YIS512 陀螺仪直接写入(角度转弧度),本模块不做滤波/
   融合。每次 ChassisSetTranslation/ChassisSetYaw 都会令对应规划器重新
   规划一段新位移;ChassisUpdate 中用当前 pose 直接作为跟踪器的实际反馈,
   对规划速度做闭环修正。跟踪器(PositionTrack)不论处于 init/idle/phaseN
   哪个规划阶段都持续生效,到位后仍锁住目标位置、抵抗扰动;平移 x、y、
   偏航使用三套完全独立的跟踪器。

   ChassisSetTranslation 的 x/y 是世界系绝对目标,而非相对当前点的增量:
   若只想改变其中一个轴,调用方必须把另一个轴原样传回上一次下发的目标
   值,而不能传当前实际位置——否则该轴会被重新规划到"当前位置"而在原地
   被打断停住。传回原目标值时,若该轴尚未到位,会被 SpeedPlanUpdate 的
   打断(同向/反向)逻辑接管、平滑续走,不会有速度突变。
*/

/* chassis->velocity_mode 取值。 */
#define CHASSIS_MODE_POSITION (0U) /* 位移 S 曲线,x/y/yaw 三轴均由规划器+跟踪器给出速度。 */
#define CHASSIS_MODE_VELOCITY (1U) /* ChassisSetVelocity 原始速度,三轴均直接下发、不重新计算。 */
#define CHASSIS_MODE_RAMP (2U)     /* 上坡模式:x、偏航仍走位移 S 曲线跟踪,y 轴固定速度
                                       ramp_vy_mps,不经过 y 方向的规划与跟踪,见
                                       ChassisSetRampVelocity。 */

/* 驱动硬件与麦轮运动学:电机组、轮序到电调 ID 的映射、逆解参数。 */
typedef struct
{
    M3508Group_TypeDef motor_group; /* 四台 C620/M3508,驱动 FL/FR/RL/RR。 */
    uint8_t motor_id[CHASSIS_MECANUM_WHEEL_COUNT]; /* 各轮对应的电调 ID,顺序 FL/FR/RL/RR。 */
    ChassisMecanum_t mecanum;       /* 麦轮运动学参数与逆解。 */
} Chassis_Drive_t;

/* 位移接口(ChassisSetTranslation/ChassisSetYaw)下的 S 曲线规划状态。 */
typedef struct
{
    SpeedPlan_TypeDef translation_x; /* 平移 x 方向独立 S 曲线,含独立跟踪器。 */
    SpeedPlan_TypeDef translation_y; /* 平移 y 方向独立 S 曲线,含独立跟踪器。 */
    SpeedPlan_TypeDef yaw;           /* 偏航 S 曲线。 */
    float translation_target_x_m;    /* 本次平移的世界系绝对目标 x。 */
    float translation_target_y_m;    /* 本次平移的世界系绝对目标 y。 */
    float yaw_target_rad;          /* 本次偏航的世界系绝对目标,ChassisSetYaw 调用时刻的
                                       pose.yaw_rad + dyaw_rad,与平移目标同为绝对量。 */
} Chassis_DisplacementPlan_t;

/* 底盘当前位姿:x_m/y_m 由外部里程计/位姿融合模块(如编码器积分)按周期
   通过 ChassisSetPosition 写入;yaw_rad 由 ChassisUpdate 每次调用时从传入
   的 YIS512 陀螺仪读数直接换算写入,本文件不做滤波/融合。初始化时清零。 */
typedef struct
{
    float x_m;     /* 底盘当前位置 x,m。 */
    float y_m;     /* 底盘当前位置 y,m。 */
    float yaw_rad; /* 底盘当前朝向,rad。 */
} Chassis_Pose_t;

typedef struct
{
    Chassis_Drive_t drive;                        /* 电机组、轮序映射、麦轮运动学。 */
    Chassis_DisplacementPlan_t displacement_plan;  /* 位移模式下的 S 曲线规划状态。 */

    ChassisMecanum_BodyVelocity_t velocity; /* 当前下发的车体速度 Vx/Vy/Wz。 */
    uint8_t velocity_mode; /* CHASSIS_MODE_POSITION:位移 S 曲线三轴均生效;
                               CHASSIS_MODE_VELOCITY:ChassisSetVelocity 原始速度三轴均生效;
                               CHASSIS_MODE_RAMP:x、偏航仍走位移 S 曲线跟踪,y 轴改为固定
                               速度 ramp_vy_mps,见 ChassisSetRampVelocity。 */
    float ramp_vy_mps; /* CHASSIS_MODE_RAMP 下 y 轴下发的固定车体速度,由
                           ChassisSetRampVelocity 写入,该模式下不经过 y 方向的
                           S 曲线规划与跟踪。 */

    ChassisMecanum_BodyVelocity_t actual_velocity; /* 麦轮正解算得到的当前实际车体速度 Vx/Vy/Wz。 */

    Chassis_Pose_t pose; /* 底盘当前位姿,预留字段,说明见上。 */
} Chassis_TypeDef;

/* 初始化底盘:初始化四台 M3508、麦轮运动学参数和两条 S 曲线规划器;
   初始为原始速度模式且速度为零,不会主动发起运动 */
void ChassisInit(Chassis_TypeDef *chassis, FDCAN_HandleTypeDef *can_handle, uint16_t ctrl_id);

/* 原始速度接口:直接下发车体速度(方向由符号表达,大小由数值表达),
   立即生效,不经过 S 曲线规划 */
void ChassisSetVelocity(Chassis_TypeDef *chassis, float vx_mps, float vy_mps, float wz_radps);

/* 平移接口:下发世界系绝对目标坐标 x_target_m/y_target_m,触发 x、y 两条
   独立 S 曲线重新规划,不影响偏航目标;运动过程速度由 ChassisUpdate 中的
   规划器结合两套独立跟踪器给出。若只想改变一个轴,另一个轴必须原样传入
   上一次下发的目标值(而非当前实际位置),否则该轴会被打断并重新规划到
   "当前位置",导致运动被截停 */
void ChassisSetTranslation(Chassis_TypeDef *chassis, float x_target_m, float y_target_m);

/* 偏航接口:下发相对当前朝向的目标偏航 dyaw,内部换算为绝对目标
   yaw_target_rad = 调用时刻的 pose.yaw_rad + dyaw,触发偏航 S 曲线重新
   规划,不影响平移目标;运动过程速度由 ChassisUpdate 中的规划器结合
   跟踪器给出,跟踪方式与平移 x/y 一致(直接以 pose.yaw_rad 为实际反馈) */
void ChassisSetYaw(Chassis_TypeDef *chassis, float dyaw_rad);

/* 上坡接口:切到 CHASSIS_MODE_RAMP,y 轴固定车体速度 vy_mps 立即生效、不经过
   y 方向 S 曲线规划与跟踪;x、偏航沿用调用前 ChassisSetTranslation/ChassisSetYaw
   已下发的目标,继续走各自的 S 曲线跟踪(闭环纠偏),不会被本接口打断或重新规划。
   用于上斜坡等 y 轴需匀速冲坡、x/yaw 仍需保持的场景 */
void ChassisSetRampVelocity(Chassis_TypeDef *chassis, float vy_mps);

/* 位姿写入接口:直接覆盖 pose.x_m/y_m(不改 yaw_rad),供外部里程计/位姿融合
   模块(如编码器积分)按周期写入当前位置,本文件不做任何计算或校验 */
void ChassisSetPosition(Chassis_TypeDef *chassis, float x_m, float y_m);

/* 停止接口:切回原始速度模式并强制车体速度三分量清零,立即生效,
   不经过 S 曲线减速 */
void ChassisStop(Chassis_TypeDef *chassis);

/* 周期调用:先由四台电机反馈转速做麦轮正解算,得到实际车体速度写入
   actual_velocity;将 yis512 的欧拉角 yaw 读数换算为弧度直接写入
   pose.yaw_rad;位移模式下再推进三条 S 曲线得到车体速度,随后统一做
   麦轮逆解并刷新四台电机目标、发送控制帧;调用前需已通过
   M3508GroupParseFeedback 更新反馈,yis512 需已通过 Yis512ParseEulerFrame
   更新读数 */
void ChassisUpdate(Chassis_TypeDef *chassis, const Yis512_TypeDef *yis512);

uint8_t ChassisTranslationReached(Chassis_TypeDef *chassis, float tolerance_m);
uint8_t ChassisYawReached(Chassis_TypeDef *chassis, float tolerance_rad);

#ifdef __cplusplus
}
#endif

#endif
