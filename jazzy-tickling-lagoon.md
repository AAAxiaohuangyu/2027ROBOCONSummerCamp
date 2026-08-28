# Context

GO-M8010 当前以 RS485 多电机总线组实现，但机械臂仅配置一个前后平移电机（USART3、ID 3）。本次将控制路径收敛为机械臂直接持有的单电机驱动，删除无实际用途的电机数组、下标和轮询仲裁，同时保留 RS485 请求-应答的 DMA 时序、协议校验与异常恢复。

## Implementation

1. 修改 `APP/Common/GO_M8010.h`：
   - 删除 `GOM8010_GROUP_MAX_MOTORS`、`GOM8010BusArbiter_TypeDef`、`GOM8010Group_TypeDef` 及全部 `GOM8010Group*` 接口。
   - 将组/下标式接口替换为直接操作 `GOM8010Motor_TypeDef` 的 `GOM8010MotorInit`、目标位置/速度/前馈设置、周期更新、RX 完成与 RX 错误处理接口。
   - 在 `GOM8010Motor_TypeDef` 内保留单笔事务状态（请求时间与 pending 标志）。保留 20 ms 超时常量，但将其定义为单电机丢失应答恢复，不再描述为总线仲裁。

2. 修改 `APP/Common/GO_M8010.c`：
   - 将组初始化和添加电机逻辑收敛到 `GOM8010MotorInit`；保持现有控制参数、S 曲线规划、FOC 控制模式和输出端/转子侧换算不变。
   - 将目标位置、速度和扭矩前馈设置函数直接作用于传入电机，移除索引和边界逻辑。
   - 将组请求函数替换为私有的单电机请求函数：pending 时等待反馈；超过超时则标记反馈无效、累加 `bad_msg` 并解除 pending；随后维持原有顺序发送 17 字节控制帧并启动固定 16 字节 DMA 接收，接收成功后才标记 pending。
   - 将组更新循环收敛为单电机更新，保持位置/速度模式控制计算和 `SpeedPlanUpdate` 行为不变。
   - 将 RX 完成与错误处理改为直接匹配该电机的 UART 实例；完成、无效帧和错误都解除 pending。保留固定长度 `HAL_UART_Receive_DMA`，不改为空闲线接收。

3. 修改 `APP/Roboticarm/RoboticArm.h` 与 `APP/Roboticarm/RoboticArm.c`：
   - 删除 `ROBOTICARM_GO_FORWARD` 下标枚举。
   - 将 `go_motors` 替换为直接成员 `forward_motor`。
   - 初始化时调用 `GOM8010MotorInit(&arm->forward_motor, forward_huart, forward_id)`。
   - 反馈状态、目标设置和周期更新直接使用 `forward_motor` 的单电机接口。

4. 修改 `APP/Common/bsp_callback.c`：
   - `HAL_UART_RxCpltCallback` 转发至 `GOM8010MotorRxEvent(&Robot.roboticarm.forward_motor, ...)`。
   - `HAL_UART_ErrorCallback` 转发至 `GOM8010MotorRxErrorEvent(&Robot.roboticarm.forward_motor, ...)`。
   - 保持 ZigBee/Vision 的回调分流不变；GO 驱动内部继续按 UART 实例过滤，避免处理其他串口事件。

5. 修改 `APP/Core/Core.c`：
   - 将 UART4 发送的 GO 反馈遥测从 `go_motors.motors[0].feedback.position` 改为 `forward_motor.feedback.position`。

## Verification

1. 全局检查不再存在 `GOM8010Group`、`go_motors`、`ROBOTICARM_GO_FORWARD`、`arbiter`、`motor_count` 或 `GOM8010_GROUP_MAX_MOTORS` 的引用。
2. 构建 Keil 工程，确认未引入头文件、类型或符号错误；`GO_M8010.c` 文件仍在工程中，无需改工程文件。
3. 上电联调 USART3/ID 3：确认每轮为 17 字节 `FE EE` 控制帧和固定 16 字节 `FD EE` 反馈帧，正常帧可更新反馈与软件零点。
4. 验证 3 ms 更新周期在上一笔反馈未完成前不重复启动接收；反馈完成、CRC/帧头错误、UART RX 错误和超过 20 ms 未响应后均能释放 pending 并在后续周期重试。
5. 验证 ZigBee、Vision 的 UART 流量不会改变 GO 电机的反馈或 pending 状态；验证位置、速度、前馈扭矩与末端 `end_x` 更新行为保持原样。
