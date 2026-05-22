# TCP/IP 知识库与双 Host 模拟包

这个包用于给非计算机专业同学解释和测试当前项目的新通信架构，并在没有真实
ESP32/HRT 时模拟完整链路：

```text
SILAB 主机/仿真系统 -> Python 逻辑判定程序
Python 逻辑判定程序 -> 小机器人/Bridge Host
Python 逻辑判定程序 -> HRT Host
Bridge -> ESP32
```

关键点：Bridge 和 HRT 都是 TCP Host。它们负责监听端口、等待连接；逻辑判定程序负责主动连接这两个 Host，并发送同一份判定后的实验数据。

## 文件说明

- `docs/NETWORK_TCP_DATA_KNOWLEDGE_BASE.md`：网络、TCP、数据帧知识库。
- `tools/voice_package.py`：读取语音包里的 common MD 和 manifest，用于模拟小机器人查找语音。
- `tools/logic_dual_host_simulator.py`：同时创建小机器人/Bridge Host 和 HRT Host 的测试脚本。小机器人 Host 会接入语音包，收到触发上升沿时打印将要播放的文本和 WAV 路径，并默认调用本机声卡播放 WAV。
- `tools/logic_decision_simulator.py`：新的逻辑判定程序模拟脚本。它从语音包选择 `SCENE/SEQ`，按固定频率同时发送给小机器人/Bridge Host 和 HRT Host。
- `requirements.txt`：Python 依赖说明。本包脚本只使用标准库，无需额外安装第三方包。

## 安装

Windows 或 Ubuntu 安装 Python 3 后，在包目录运行：

```bash
python -m pip install -r requirements.txt
```

如果输出没有安装任何包，这是正常的，因为脚本不需要第三方依赖。

## 自检

```bash
python tools/logic_dual_host_simulator.py --self-test
```

应该能看到两个 Host 启动，并收到示例数据：

```text
[ROBOT_HOST] RISING trigger=1 ts=1779235200 scene=common seq=left_rear_vehicle_merge speed=1.2
[ROBOT_HOST] robot would_play common/left_rear_vehicle_merge.wav exists=true ...
[AUDIO] playing left_rear_vehicle_merge.wav via pw-play
[HRT_HOST] RISING trigger=1 ts=1779235200 scene=common seq=left_rear_vehicle_merge speed=1.2
```

## 正式模拟小机器人和 HRT

```bash
python tools/logic_dual_host_simulator.py
```

默认端口：

```text
小机器人/Bridge Host: 7777
HRT Host:            9001
```

逻辑判定程序应连接实验电脑的真实局域网 IP，不要连接 `0.0.0.0`。`0.0.0.0` 只表示 Host 监听本机所有网卡。

小机器人/Bridge 数据格式：

```text
TRIGGER;TIMESTAMP;SCENE;SEQ;SPEED<LF>
```

例子：

```text
1;1779235200;boot;1;1.2
```

HRT 数据格式：

```text
trigger,timestamp,scene,seq,speed;
```

例子：

```text
1,1779235200,boot,1,1.2;
```

## 模拟逻辑判定程序发送语音触发

先启动双 Host：

```bash
python tools/logic_dual_host_simulator.py
```

再启动逻辑判定程序模拟器：

```bash
python tools/logic_decision_simulator.py \
  --bridge-host 127.0.0.1 --bridge-port 7777 \
  --hrt-host 127.0.0.1 --hrt-port 9001 \
  --entry common/left_rear_vehicle_merge \
  --pattern 0:0.5,1:0.4,0:0.5 \
  --rate-hz 10
```

小机器人 Host 会打印：

```text
robot would_play common/left_rear_vehicle_merge.wav exists=true ...
text=左后方有车辆汇入
[AUDIO] playing left_rear_vehicle_merge.wav via pw-play
```

这说明逻辑判定程序发出的 `SCENE/SEQ` 已经能在语音包里找到真实 WAV，并且小机器人模拟端已经调用本机声卡播放。Ubuntu 下脚本会自动尝试 `pw-play`、`paplay`、`aplay`、`ffplay`；Windows 下会使用系统自带 `winsound`。

如果只想看日志、不想让电脑发声：

```bash
python tools/logic_dual_host_simulator.py --no-play-audio
```

语音包默认从相邻仓库读取：

```text
../silent_failure_tts/dist/common_voice_test_assets_20260522
```

如果语音包在别的位置，用：

```bash
python tools/logic_decision_simulator.py --voice-package D:\path\common_voice_test_assets_20260522 ...
python tools/logic_dual_host_simulator.py --voice-package D:\path\common_voice_test_assets_20260522
```

## 常见问题

- 连接不上：检查逻辑判定程序填写的是目标电脑真实 IP，不是 `127.0.0.1` 或 `0.0.0.0`。
- 收到 BAD：检查字段数量、分隔符和结束符是否正确。
- 小机器人有数据但 HRT 没数据：检查逻辑判定程序是否同时连接了 HRT Host。
- HRT 有数据但小机器人显示 `voice MISS`：检查发送的 `SCENE/SEQ` 是否来自当前语音包。
- HRT 有数据但真实 ESP32 没声音：这个双 Host 脚本只模拟 Host 接收，不会真的控制 ESP32，需要使用正式 Bridge GUI。
