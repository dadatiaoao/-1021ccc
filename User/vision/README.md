# K230 视觉通信模块

本目录实现现有 K230 YOLO 程序的 STM32 对端。完整接线、烧录、协议和屏幕操作见 [K230 串口协议与联调](../../Doc/K230_串口协议与联调.md)。

- `vision_protocol`：最多 16 字节的流解析与组包，纯 C，无 HAL 依赖。
- `vision_service`：命令排队和 ACK、上下文匹配、二维码及坐标保存、失效处理。`TargetFresh` 只表示新鲜的候选。
- `vision_ui`：原触摸界面的视觉页面文本。点击 QR 页左上标题进入。
- 硬件层在 `User/usart/bsp_vision_uart.*`，应用已接入 `QrApp_Init/Process`，不要另建一个重复占用 USART1 的 `main.c`。

模式2/4的目标编号对应整个大圆环上的工位数字1～3。这里不把编号1解释为最中间的小评分环，也不从检测框推断内环精度。
