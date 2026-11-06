# K230 比赛视觉工程

本目录是当前开发主线。根 `main.py` 仍是 SD 卡启动入口，颜色和编号大圆环都使用你的原有 YOLO 模型；无需重训或替换模型。本次按 2027 智能搬运赛题补充了质量检查、局部圆心候选拟合、转盘稳定候选、任务码一致性和部署工具。

**模型含义已按你的说明修正：圆环 YOLO 识别整组大圆环及其中的位置数字。R1/R2/R3 是放置位置编号，不是最内侧的 1 环、2 环、3 环评分。检测框中心只用于粗定位。**

## 从这里开始

1. 把发布包解压后的内容放到 SD 卡根目录；不要把整个 `sdcard` 文件夹嵌套放到卡内。
2. 先运行 `tools/check_install.py`，检查配置、两个模型的大小、SHA256 及类别映射。
3. 运行 `tools/preview_models.py` 看两类 YOLO 框；运行 `tools/preview_rings.py` 看局部圆心候选。这些预览不打开 UART，也不输出 PWM 或电机命令。
4. 转盘样本用 `tools/preview_pick.py` 检查稳定性，颜色在该脚本中选择；可达区域和标定标志在 `transport_vision/vision_config.py` 设置。
5. 正式串口入口运行根 `main.py`。接收 STM32 指令后工作；开机预览只画框，不主动报告目标结果。

串口保持 UART2、115200/8N1：K230 IO11/TX2 → STM32 PA10/RX；K230 IO12/RX2 ← STM32 PA9/TX；GND 共地。IO 编号不是排针顺序。配套 STM32 对端现已接入 USART1、扩展观察包和屏幕联调入口；见 [STM32 接线与烧录说明](../主控板程序/Doc/K230_串口协议与联调.md)。K230 通信格式保持兼容，之前已复制到 SD 卡的版本可继续使用。

## 文件夹职责

```text
k230烧录进去的程序/
  main.py                       唯一正式启动入口
  transport_vision/             当前运行核心、统一配置
    vision_app.py               摄像头、推理、串口和 OSD
    vision_logic.py             目标选择、观察快照、二维码一致性
    vision_quality.py           新帧、时间窗、漂移与尺寸稳定性
    ring_refinement.py          YOLO ROI 内同心圆族中心候选
    vision_tasks.py             任务码校验与同色码垛位置展开
    vision_settings.py          启动配置检查
    vision_protocol.py          原协议和可选扩展
    models/                     原模型和 manifest.json
  tools/                        板上只读安装检查、预览和单张采样
  calibration/                  实测记录模板，不自动写入运行配置
  docs/                         赛题对应关系、协议、标定及验收计划
  host_tests/                   电脑端测试，不放入发布包
  host_tools/                   电脑端打包工具，不放入发布包
  libs/、res/                   原有运行库和字体
  capture/                      采样图片
  archive/                      原 codex_* 实验原样归档，不放入发布包
```

原 `transport_vision/main.py`、`run_integrated.py`、`preview_models.py` 和台架入口保留兼容。原 `codex_yolo`、`codex_legacy_color`、`codex_gripper` 现位于 archive 内，内容未改；它们的旧硬编码 SD 路径未迁移，不能在新位置直接当当前程序运行。夹爪事故保护仍保留，未解锁。

## 本次实际完善

- 非法数值、残缺结果、同色多目标或重复环号不再悄悄选择一个高分框；可用 ROI 限定工位。
- 新帧同时满足时间窗、位置及框尺寸稳定要求，才标记稳定；重复上报不增加稳定次数。
- SCAN(3) 用于转盘抓前稳定候选，PICK_READY 需要实测的 PICK_ROI 和 PICK_CALIBRATED；它不表示抓后持物成功。
- SCAN(4) 用于局部几何精定位候选：在同一 RGBP888 帧的绿色通道内，仅处理目标 YOLO ROI，要求多个独立半径的圆具有一致中心。失败、明显斜视或多圆族歧义时返回未找到，不退回框中心。
- 超过处理时间上限的图像作废；STOP 或推理中改变任务/姿态时，旧结果作废。
- 二维码需要连续 3 个一致的新采样结果，多张不同合法二维码不任取第一张；第二批暂存位按第一批同色位置展开，仅用于检查和日志，不接管 STM32 比赛调度。
- 原指令和原包格式保留；可选 SET_CONTEXT/OBSERVATION 关联请求、相机姿态和采集序号，主控未启用时不发送新观察包。
- 增加模型清单、自检、只读预览、原图采样、主机回归和发布包白名单。

## 必须知道的行为变化

**原 PLACE_CHECK=1 判据被收紧。** 130 像素接近不能证明比赛放置正确。默认 PLACE_CHECK_CALIBRATED=False、RING_REFINEMENT_CALIBRATED=False，所以模式 2 的 PLACE_CHECK 目前始终为 0；坐标仍可用于联调。主控不能把它误解为已完成抓放验收。

模式 2 的旧 RING 包仍发框中心，避免悄悄改变原协议含义；若启用扩展观察包，可从 REFINED 标志区分同一帧的几何中心候选。模式 4 只发拟合中心，未拟合时发 NOTFOUND。两种坐标都未自动换算成毫米。

同心圆拟合候选不等于已识别最内侧评分 1 环，不保证毫米精度，也不证明码垛成功。斜视椭圆、物料遮挡、相机随臂姿态变化仍需真实样本和标定；没有相关模型时，本程序也不具备可靠的障碍检测能力。

## 文档与电脑验证

- [完善内容与赛题对应](docs/COMPETITION_PLAN.md)
- [通信协议与 STM32 对接](docs/PROTOCOL.md)
- [采样、标定与验收](docs/CALIBRATION.md)
- [修改与验证记录](docs/CHANGELOG.md)

在电脑上、以本目录为工作目录：

```text
python -B -m unittest discover -s host_tests -v
python -B tools/check_install.py .
python -B host_tools/build_package.py --out 新的输出目录
```

打包工具只发布当前核心、模型、工具和文档，排除历史实验、夹爪脚本、测试、电脑工具和磁盘系统信息。运行包的自检无需连接相机；采样工具会在 capture 下保存一张无标注 BMP。
