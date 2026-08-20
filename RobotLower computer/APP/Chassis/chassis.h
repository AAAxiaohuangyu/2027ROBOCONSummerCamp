#ifndef CHASSIS_H
#define CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "M3508.h"
#include "StrategyAlogrithm.h"
#include "chassis_mecanum.h"

/*
底盘车体坐标与速度模型(底盘平面几何中心为坐标原点,单位:m/rad):

1. 车体系定义为 +X 向前、+Y 向左、+Wz 从车顶看逆时针,四轮数组顺序始终为
   FL(左前)、FR(右前)、RL(左后)、RR(右后),与 chassis_mecanum.h 保持一致。

2. 原始速度接口(ChassisSetVelocity)直接下发车体速度 Vx/Vy/Wz,方向由符号
   表达、大小由数值表达,不经过 S 曲线规划,信任调用者给出的目标。

3. 位移接口拆分为平移、偏航两个独立入口:ChassisSetTranslation 下发世界系
   绝对目标坐标 x/y,ChassisSetYaw 下发相对当前朝向的目标偏航 dyaw;两者
   分别触发各自的重新规划,互不影响。平移进一步拆分为 x、y 两条完全独立
   的七段 S 曲线规划器,二者参数一致、互不耦合;偏航按 dyaw 的符号原地
   转动。

4. 本模块不做里程计/位姿积分,pose 由外部里程计/位姿融合模块按周期写入
   (x_m/y_m 已接入,yaw_rad 待后续融合模块补齐,目前恒为 0)。每次
   ChassisSetTranslation/ChassisSetYaw 都会令对应规划器重新规划一段新
   位移;ChassisUpdate 中用当前 pose 直接作为跟踪器的实际反馈,对规划
   速度做闭环修正。跟踪器(PositionTrack)不论处于 init/idle/phaseN 哪个
   规划阶段都持续生效,到位后仍锁住目标位置、抵抗扰动;平移 x、y 使用
   两套完全独立的跟踪器。

   ChassisSetTranslation 的 x/y 是世界系绝对目标,而非相对当前点的增量:
   若只想改变其中一个轴,调用方必须把另一个轴原样传回上一次下发的目标
   值,而不能传当前实际位置——否则该轴会被重新规划到"当前位置"而在原地
   被打断停住。传回原目标值时,若该轴尚未到位,会被 SpeedPlanUpdate 的
   打断(同向/反向)逻辑接管、平滑续走,不会有速度突变。
*/

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
    float yaw_target_rad;          /* 本次偏航目标,带符号。 */
    float yaw_start_rad;           /* 本次偏航规划触发时刻的起始朝向,供跟踪器计算实际已转角度。 */
} Chassis_DisplacementPlan_t;

/* 底盘当前位姿,预留字段:本文件不计算、不更新,由外部里程计/位姿
   融合模块负责写入;初始化时清零。 */
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
    uint8_t velocity_mode; /* 1:ChassisSetVelocity 原始速度生效;0:位移 S 曲线生效。 */

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

/* 偏航接口:下发相对当前朝向的目标偏航 dyaw,记录当前 pose.yaw_rad 作为
   跟踪起点,触发偏航 S 曲线重新规划,不影响平移目标;运动过程速度由
   ChassisUpdate 中的规划器结合跟踪器给出 */
void ChassisSetYaw(Chassis_TypeDef *chassis, float dyaw_rad);

/* 位姿写入接口:直接覆盖 pose.x_m/y_m(不改 yaw_rad),供外部里程计/位姿融合
   模块(如编码器积分)按周期写入当前位置,本文件不做任何计算或校验 */
void ChassisSetPosition(Chassis_TypeDef *chassis, float x_m, float y_m);

/* 停止接口:切回原始速度模式并强制车体速度三分量清零,立即生效,
   不经过 S 曲线减速 */
void ChassisStop(Chassis_TypeDef *chassis);

/* 周期调用:先由四台电机反馈转速做麦轮正解算,得到实际车体速度写入
   actual_velocity;位移模式下再推进两条 S 曲线得到车体速度,随后统一做
   麦轮逆解并刷新四台电机目标、发送控制帧;调用前需已通过
   M3508GroupParseFeedback 更新反馈 */
void ChassisUpdate(Chassis_TypeDef *chassis);

uint8_t ChassisTranslationReached(Chassis_TypeDef *chassis, float tolerance_m);
uint8_t ChassisYawReached(Chassis_TypeDef *chassis, float tolerance_rad);

#ifdef __cplusplus
}
#endif

#endif
