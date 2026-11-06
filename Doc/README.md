# 文档索引与维护状态

更新：2026-11-05。按键校准改动基于 `main / f75e0ca`；其余源码核对基线：`main / f14be37`。主线已包含原电机里程计开发分支；旧主线 `01c8a68` 保存在 `archive/main-before-odometry`。

软件行为以实际源码及默认配置为准；需求规划不能写成已实现能力，源码也不能证明实物验收。历史测试只对应记录中的版本，不因文档日期更新而成为本次测试结果。外部赛事资料、厂家手册和其他工程未在本次重新核验。

2026-11-04 后续代码变更：光流初始化、采样和显示入口已删除，`User/flow/` 保留且退出 Keil/默认测试编译。首页新增“鼠标位移模块”占位卡片，无内页。此前实机测试确认不自动覆盖这次变更。

2026-11-05：取消 gyro 固定 15.86°/s 补偿，改为静止按 CAL 采集至少 201 个新样本且满 2 s 后更新 RAM 偏置；上电/重初始化后需重校准。三轴单样本及均值绝对值上限统一为 25°/s，极差、方差及有效性检查保留；操作及阈值见[工程 README §4.5](../README.md#45-陀螺仪)。新流程待上板验证。

2026-11-05：删除底盘八段执行代码与专用配置；原 UI 改为前进/后退 30 cm 两个独立按钮。历史八段测试记录仅对应旧版本。用户于 2026-11-05 确认新按钮实测前进/后退方向正确；距离精度及直行后旋转问题仍待验证。本次仅修正方向注释和说明，保持运动参数及控制符号不变。

2026-11-06：新增左右转 90°；陀螺启动 2 s 无有效样本则本次上电锁定轮端估算，新增默认 1.0 的左右轮周长比例。细节见工程 README 末节。

## 继续维护的入口

| 文档 | 用途与范围 |
|---|---|
| [工程 README](../README.md) | 当前能力、上电/操作行为、配置入口、用户确认和待验事项 |
| [底盘电机里程计](CHASSIS_MOTOR_ODOMETRY_2026-10-24.md) | 当前反馈与故障边界；初版资源、测试与 HEX 单独保留为历史 |
| [机构标定记录](MECHANISM_CALIBRATION_TODO_2026-10-25.md) | 当前 5/6/7 轴参数、回零条件、标定待办与分日期实测记录 |
| [UI 说明](UI_REDESIGN_2026-10-24.md) | 当前页面和操作；2026-10-24 截图不能代表后来新增页面 |
| [比赛框架说明](../User/mission/README.md) | 已实现的纯调度、空执行端、计数、超时及后续接入边界 |
| [行为层总计划](BEHAVIOR_MASTER_PLAN_2026-10-22.md) | 保留比赛目标与后续设计；顶部进度表区分已实现基础和未接入部分，旧交接内容不是执行授权 |

日期文件名保留，避免断开已有引用；正文更新日期与历史验证日期分别注明。

## 本次核对的源码依据

| 事实 | 代码入口 |
|---|---|
| 协作式主循环；空 MissionPort；STOP 覆盖七轴与舵机 | [main.c](../User/main.c)、[qr_app.c](../User/app/qr_app.c) |
| 电机位置＋gyro，光流不参与底盘判断 | [chassis.c](../User/chassis/chassis.c)、[chassis_odometry.c](../User/chassis/chassis_odometry.c) |
| 麦轮解算、100 mm/s、30°/s、20 ms 控制与 100 ms 发布 | [chassis_kinematics.c](../User/chassis/chassis_kinematics.c)、[chassis_config.h](../User/chassis/chassis_config.h) |
| 默认 TRIAL；6→5→符合配置/反馈条件的 7；缺少 7 时 HOME 成功不等于三轴 HOMED | [mechanism.c](../User/mechanism/mechanism.c)、[mechanism_config.h](../User/mechanism/mechanism_config.h) |
| 双路 50 Hz PWM；三轴 HOMED 后自动启用；手动 ON 不复核 HOMED | [servo_control.c](../User/servo/servo_control.c)、[servo_pwm.c](../User/servo/servo_pwm.c)、[qr_app.c](../User/app/qr_app.c) |
| CAL 与 ZERO 的电机门控不同；固定偏置不是实物校准证明 | [gyro_service.c](../User/gyro/gyro_service.c)、[gyro_config.h](../User/gyro/gyro_config.h)、[qr_app.c](../User/app/qr_app.c) |
| 两批总计抓取 12 次/放置 12 次；扫码 15 s、其他单动作 90 s；无整场 180 s 时限 | [mission.c](../User/mission/mission.c)、[test_mission.c](../Tests/test_mission.c) |
| 路线配置/验证函数存在但未接入应用；K230、全局导航和抓放执行器未接入 | [mission_config.c](../User/mission/mission_config.c)、[qr_app.c](../User/app/qr_app.c)、[Keil 工程](../Project/YH-F429.uvprojx) |
| 当前页面、舵机脉宽预设、比赛未就绪提示 | [qr_ui.c](../User/app/qr_ui.c)、[servo_control.c](../User/servo/servo_control.c) |
| 构建及主回归脚本面向 Windows Keil/MSVC | [build.ps1](../Scripts/build.ps1)、[test.ps1](../Scripts/test.ps1) |

## 本次新增的实物确认

用户于 2026-11-04 明确范围为：**完成当前屏幕八段底盘验收和机构回零测试**。不包含 K230 按钮启动后自主扫码、两批搬运、码垛及返回。未提供测试实际日期、固件提交号/HEX、各轴覆盖、重复次数或量化误差，因此记录功能测试完成，精度和异常路径仍保留待验。此前“整分支从未实测”的笼统说法不再沿用；历史交付当时的“未烧录”记录保留。

## 已过时，停止按当前代码维护

这些文档保留原始设计、测试证据与截图；每篇开头均有失效原因和当前入口。正文中的“当前”“本轮”“下一对话”等仅代表当时时点，不能用来恢复旧控制方式、重新启用旧菜单或推定实物通过。

| 类别 | 历史文档 |
|---|---|
| 扫码 | [阶段任务](QR_UART_STAGE_TASK.md)、[交付](QR_UART.md) |
| 七电机 | [阶段任务](MULTI_MOTOR_STAGE_TASK.md)、[交付](MULTI_MOTOR.md)、[单机演示](MOTOR_DEMO.md) |
| 光流 | [阶段任务](OPTICAL_FLOW_STAGE_TASK.md)、[开发基线](OPTICAL_FLOW_BASELINE.md)、[交付](OPTICAL_FLOW.md) |
| 陀螺与总线修复 | [陀螺任务](GYRO_STAGE_TASK.md)、[交付](GYRO.md)、[审查计划](GYRO_MOTOR_REVIEW_FIX_PLAN_2026-10-22.md)、[修复报告](GYRO_MOTOR_FIX_REPORT_2026-10-22.md) |
| 旧光流底盘 | [阶段计划](CHASSIS_STAGE_PLAN_2026-10-23.md)、[交付](CHASSIS_IMPLEMENTATION_2026-10-23.md)、[首次试跑修复](CHASSIS_FLOW_FIX_2026-10-24.md) |
| 旧机构方案 | [阶段计划](MECHANISM_STAGE_PLAN_2026-10-24.md)、[初版交付](MECHANISM_IMPLEMENTATION_2026-10-24.md)、[初版试运行](MECHANISM_TRIAL_2026-10-24.md) |
| 旧显示 | [电机页面梳理](MOTOR_UI_DISPLAY_2026-10-23.md)、[原厂画板例程](readme.txt) |

## 历史候选方案与来源记录

[TTL → CAN](TTL_CAN_MIGRATION_PLAN_2026-10-23.md)和[无极 S2 无屏迁移](无极S2无屏迁移假设性计划书_2026-10-22.md)均未实施。它们的基线已过时，保留设计参考；若再采用，须针对当前 main 重做范围、引脚及接口检查，不能将未实施写成已取消。

[光流第三方声明](../User/flow/THIRD_PARTY_NOTICES.md)、[陀螺来源](../User/gyro/UPSTREAM.md)及[Apache-2.0 许可](../User/gyro/LICENSE-Apache-2.0.txt)仍是有效来源/许可记录，不按业务文档过时处理。`Doc/flow_validation/`、旧 UI 图片和原屏幕参数图片作为历史证据保留，本次未重新生成。

2026-11-04 文档整理仅更新文档，未修改源码或重跑测试。2026-11-05 按键校准变更涉及源码、测试和当前说明；不改写以下归档文档中的历史验证记录。
