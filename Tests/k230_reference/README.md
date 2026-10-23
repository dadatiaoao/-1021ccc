# K230 协议测试参考

`vision_protocol.py` 是配套 K230 工程 `transport_vision/vision_protocol.py` 的原样副本，用于在独立克隆本 STM32 仓库后生成标准线协议报文。

这是纯 Python/MicroPython 协议模块，不包含相机、YOLO、模型或电机逻辑。STM32 固件不会编译、运行这个 Python 文件。

`Tests/generate_vision_fixtures.py` 默认使用此副本，也支持 `--protocol <实际 K230 vision_protocol.py 路径>`，用于重新核对对端。对端协议变更时，应同步检查双方编解码和这些测试。
