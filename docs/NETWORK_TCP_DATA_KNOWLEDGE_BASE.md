# 网络/TCP/数据帧知识库

本文用于解释本项目里 SILAB 主机、逻辑判定程序、Windows Bridge GUI、ESP32、HRT 之间的数据通信原理。它不是部署步骤，而是帮助实验员和工程同学理解：为什么要配置 IP，为什么要分清 TCP Host/Device，Python 怎样收发 TCP，以及为什么数据帧要定义格式、精度、拆分和合并规则。

主要参考资料：

- [RFC 791: Internet Protocol](https://www.rfc-editor.org/rfc/rfc791.html)
- [RFC 9293: Transmission Control Protocol](https://www.rfc-editor.org/rfc/rfc9293.html)
- [Python Socket Programming HOWTO](https://docs.python.org/3/howto/sockets.html)
- [Python socket 标准库文档](https://docs.python.org/3/library/socket.html)
- [Python struct 标准库文档](https://docs.python.org/3/library/struct.html)
- [Python JSON 标准库文档](https://docs.python.org/3/library/json.html)
- [Python 浮点数说明](https://docs.python.org/3/tutorial/floatingpoint.html)

## 1. 本项目通信拓扑

当前实验拓扑可以理解为：

```text
SILAB 主机/仿真系统
        |
        | 本机读取系统状态，只和本机 Python 脚本交互
        v
Python 逻辑判定程序
        |\
        | \--TCP/LAN--> HRT Host
        |
        \--TCP/LAN--> Windows Bridge GUI --Direct TCP + MQTT/LAN B--> ESP32
```

角色分工：

| 链路 | 主动方/数据来源 | 接收方 | 项目含义 |
| --- | --- | --- | --- |
| SILAB -> 逻辑判定程序 | SILAB 主机内部程序或本机接口 | Python 逻辑判定程序 | 逻辑程序读取 SILAB 系统状态，不让 Bridge/HRT 直接处理 SILAB 原始数据 |
| 逻辑判定程序 -> Bridge | Python 逻辑判定程序 | Bridge GUI | 发送触发标志、时间戳、场景名、序号、本车速度 |
| 逻辑判定程序 -> HRT | Python 逻辑判定程序 | HRT Host | 发送同一份判定后的实验数据，供 HRT 记录或联动 |
| Bridge -> ESP32 Direct TCP | Bridge GUI | ESP32 | Bridge 优先用低延迟直连 TCP 下发播放命令 |
| Bridge/ESP32 -> MQTT Broker | Bridge GUI、ESP32 | Windows Mosquitto | ESP32 上报状态，Bridge 做状态监控和兜底控制 |

注意：TCP 连接建立后，两端都能收发数据；所谓 Host/Device、主/从、Server/Client，通常是应用层约定。TCP 协议本身更关心连接、字节流、可靠传输和端口。

这个架构的核心变化是：SILAB 不再直接对 Bridge 或 HRT 输出最终控制协议。SILAB 主机上运行一个 Python 逻辑判定程序，由它读取 SILAB 系统状态，判断是否触发、选择播放哪个场景和序号，然后把已经整理好的标准数据发给 Bridge 与 HRT。这样 Bridge、ESP32、HRT 不需要理解 SILAB 内部格式，也不受 SILAB 本身数值口、字符串口或拆分字段限制。

这里最容易漏掉的一点是：**Bridge 和 HRT 都是 TCP Host**。也就是说，它们都先打开一个端口，站在原地等别人连接。逻辑判定程序不是 Host，而是 TCP Client/Device：它判定完一帧数据后，主动连接 Bridge Host 发送播放控制数据，同时主动连接 HRT Host 发送记录/联动数据。

```text
Bridge Host:  bind 0.0.0.0:7777  等逻辑判定程序连接
HRT Host:     bind HRT_IP:9001    等逻辑判定程序连接
逻辑判定程序: connect Bridge_IP:7777 + connect HRT_IP:9001
```

`0.0.0.0` 只适合 Host 监听，表示“监听本机所有网卡”。Client 连接时不能填 `0.0.0.0`，必须填目标电脑在局域网里的真实 IP，例如 `192.168.0.12`。

## 2. IP 协议原理

IP 负责把一个数据包从源主机送到目标主机。RFC 791 将 IP 的核心能力概括为地址、路由、分片与重组。简单说：

- IP 地址回答“发到哪台机器”。
- 路由回答“从哪条路径过去”。
- 分片回答“包太大时怎样拆成多个包”。
- IP 本身不保证可靠到达，不保证顺序，也不重传。

通俗理解：IP 像快递地址系统。每台电脑有一个地址，数据被切成一个个小包，每个包外面写着“从哪里来、到哪里去”。交换机和路由器像分拣站，按地址把包往下一站送。IP 只负责尽力送过去，不保证一定送到，也不保证先发的包一定先到。

更底层一点看，一个 IP 包大致包含两部分：

- 包头：源 IP、目标 IP、长度、分片信息等。
- 负载：真正要交给上层协议的数据，比如 TCP 数据。

如果一个包太大，IP 层可能把它分成多个更小的包传输，到对端再重组。这个能力叫分片与重组。但工程上最好不要依赖 IP 分片解决应用层问题；应用层仍然要自己定义清楚数据帧。

本项目里的典型 IP：

| IP | 所属网络 | 用途 |
| --- | --- | --- |
| SILAB 主机 IP | SILAB 所在网络 | 运行 Python 逻辑判定程序，读取 SILAB 状态 |
| Bridge 主机 IP | Bridge 所在网络 | 填到逻辑判定程序的 Bridge 目标地址 |
| Windows WiFi IP | LAN B | 配给 ESP32 的 MQTT Broker URI |
| ESP32 WiFi IP | LAN B | Bridge 直接 TCP 控制 ESP32，并通过 MQTT 监控状态 |
| HRT Host IP | HRT 所在网络 | 填到逻辑判定程序的 HRT 目标地址 |

常见错误：

- 把 `127.0.0.1` 填给远端设备。`127.0.0.1` 只代表“本机自己”，只能用于同一台电脑内部互连。
- 让 SILAB、Bridge、HRT、ESP32 相互理解对方内部格式。正确做法是让逻辑判定程序输出统一标准帧。
- 不区分网线 IP 和 WiFi IP。Windows 双网卡时，这两个 IP 很可能不同。

## 3. TCP 协议原理

TCP 是建立在 IP 之上的传输层协议。RFC 9293 将 TCP 定义为面向连接、可靠、有序的主机到主机通信协议。

TCP 的关键特点：

- 面向连接：先建立连接，再传数据。
- 可靠传输：TCP 负责确认、重传、排序。
- 字节流：TCP 传的是连续字节，不保留应用层“消息边界”。
- 双向通信：连接建立后，两端都可以 `send` 和 `recv`。

通俗理解：TCP 像先打电话再说话。Client 先拨号，Server/Host 接电话，双方确认连接建立后，才开始连续说话。TCP 会给字节编号，对方收到后会确认；如果中途丢了，TCP 会重传；如果乱序到了，TCP 会重新排好再交给应用层。

更底层一点看，TCP 主要靠这些机制保证可靠：

- 端口：同一台电脑上区分不同程序，例如 Bridge 监听 `7777`，HRT 监听 `9001`。
- 序号：给字节流编号，让接收方知道顺序。
- ACK 确认：接收方告诉发送方“我收到哪里了”。
- 重传：发送方发现迟迟没有确认，就再发一次。
- 窗口：控制一次能发多少，避免把对方或网络塞爆。

所以 TCP 能保证“字节流可靠、有序”，但它不知道应用层一条消息从哪里开始、哪里结束。这就是为什么本项目要规定 `<LF>` 或 `;` 作为帧结束符。

### 主从关系怎么理解

工程里常说的“主机/从机”“Host/Device”“Server/Client”不是 TCP 的绝对主从，而是连接发起方式：

| 名称 | TCP 动作 | 说明 |
| --- | --- | --- |
| Server / Host | `bind` -> `listen` -> `accept` | 先占用本机 IP/端口，等待别人连入 |
| Client / Device | `connect` | 主动连接对方 IP/端口 |

本项目映射：

- Bridge GUI 是逻辑判定程序的 TCP Host，因为它监听 `0.0.0.0:7777`。
- 逻辑判定程序是 Bridge 的 TCP Client，因为它连接 Bridge 主机 IP 的 `7777`。
- HRT 是 TCP Host，因为它监听某个端口。
- 逻辑判定程序是 HRT 的 TCP Device，因为它主动连接 HRT。
- SILAB 主机和逻辑判定程序之间可以是本机 TCP、本机文件、API、共享内存或厂商接口；这部分属于 SILAB 内部采集，不再作为 Bridge/HRT 的公共协议。

这里的“主从”只描述谁等连接、谁主动连接，不代表谁更高级，也不代表数据只能单向流动。连接建立后，Host 和 Client 都可以收发数据。

## 4. TCP 字节流与帧格式

TCP 不知道“这一包就是一条消息”。例如发送端连续发送：

```text
abc\n
def\n
```

接收端可能一次收到：

```text
abc\ndef\n
```

也可能分两次收到：

```text
ab
c\ndef\n
```

所以应用层必须自己定义“帧边界”。本项目采用文本帧，并用换行符作为逻辑判定程序 -> Bridge 的帧结束标记：

```text
TRIGGER;TIMESTAMP;SCENE;SEQ;SPEED<LF>
```

逻辑判定程序 -> HRT 可以沿用 HRT 现有的分号结束格式：

```text
trigger,timestamp,scene,seq,speed;
```

两条链路发送的是同一份语义数据：触发标志、完整 Unix 时间戳、场景名称、场景序号、本车速度。差别只在于接收端历史习惯不同，Bridge 更适合按行读取，HRT 可能已有逗号字段、分号结尾的解析逻辑。

### 为什么 Bridge 输入用 `<LF>`

因为 Bridge 的 TCP Host 会持续读取字节，遇到 `\n` 或 `\r` 才认为一条逻辑判定帧结束。这样即使 TCP 把数据拆开或合并，Bridge 也能正确还原每一帧。

### 为什么 HRT 输出用 `;`

参考 `HRT-button-0.1.py`，HRT 侧已有逗号字段、分号结尾的 Device 文本习惯，例如：

```text
0,1;
```

因此发送给 HRT 时也保持：

```text
1,1779235200,boot,1,1.2;
```

新架构下，HRT 数据可以由逻辑判定程序直接发送，不必再由 Bridge 转发。Bridge 是否继续转发 HRT，可作为兼容旧流程的调试选项，而不是主路径。

## 5. Python 如何用 TCP 收发数据

Python 使用标准库 `socket` 进行 TCP 通信。`socket.AF_INET` 表示 IPv4，`socket.SOCK_STREAM` 表示 TCP。

### TCP Server / Host 示例

```python
import socket

HOST = "0.0.0.0"
PORT = 7777

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(4)

    conn, addr = server.accept()
    with conn:
        buffer = bytearray()
        while True:
            data = conn.recv(1024)
            if not data:
                break
            for b in data:
                if b in (10, 13):
                    if buffer:
                        line = buffer.decode("utf-8", errors="replace")
                        print("frame:", line)
                        buffer.clear()
                else:
                    buffer.append(b)
```

这个模式就是 Bridge GUI 接收逻辑判定程序数据的基本结构。

### TCP Client / Device 示例

```python
import socket

HOST = "192.168.0.10"
PORT = 9001

message = "1,1779235200,boot,1,1.2;"

with socket.create_connection((HOST, PORT), timeout=1.0) as sock:
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.sendall(message.encode("utf-8"))
```

这个模式就是逻辑判定程序作为 TCP Device 主动发送 HRT 数据的基本结构。

### `send` / `recv` 的注意点

- `sendall()` 表示尽量把全部字节发完；普通 `send()` 可能只发出部分字节。
- `recv(1024)` 最多读 1024 字节，不代表一定读到一整帧。
- 发送的是 `bytes`，文本需要 `.encode("utf-8")`。
- 接收后是 `bytes`，文本需要 `.decode("utf-8")`。
- TCP 断开时，`recv()` 返回空字节串 `b""`。

## 6. 本项目数据帧格式

### 逻辑判定程序 -> Bridge

```text
TRIGGER;TIMESTAMP;SCENE;SEQ;SPEED<LF>
```

字段说明：

| 字段 | 类型 | 例子 | 说明 |
| --- | --- | --- | --- |
| `TRIGGER` | 数字 | `0` / `1` | 触发标记。`>= 0.5` 视为有效 |
| `TIMESTAMP` | 整数或浮点数字符串 | `1779235200` | 完整 Unix 时间戳，优先使用整数秒 |
| `SCENE` | 字符串 | `boot` | 场景名称，直接对应 TF 卡目录名 |
| `SEQ` | 整数 | `1` | 序号 ID，映射为 `001.wav` |
| `SPEED` | 数字 | `1.2` | 速度，给实验员观察和判定状态 |

例子：

```text
1;1779235200;boot;1;1.2
```

Bridge 解析结果：

```text
timestamp = 1779235200
trigger   = 1
scene     = boot
seq       = 1
speed     = 1.2
```

由于这条帧由 Python 逻辑判定程序生成，不再受 SILAB 原始数据口限制，所以不需要把时间拆成 `TIME_LOW` 和 `TIME_HIGH`，也不需要用数字场景 ID 再做别名映射。逻辑判定程序应该直接输出完整时间戳和最终场景名称。

### Bridge -> ESP32

逻辑判定程序负责判断 `TRIGGER` 和 `SCENE/SEQ`。Bridge 收到标准帧后，只在 `TRIGGER` 从无效变有效时发送播放命令：

```json
{"type":"play","scene":"boot","seq":1,"message_id":"silab_..."}
```

连续多帧 `TRIGGER=1` 不会重复播放。必须先收到 `TRIGGER=0`，下一次 `TRIGGER=1` 才会再次触发。

### 逻辑判定程序 -> HRT

逻辑判定程序会把每一条判定后的合法帧发送给 HRT。若 HRT 沿用现有逗号字段、分号结尾格式，可使用：

```text
trigger,timestamp,scene,seq,speed;
```

例子：

```text
1,1779235200,boot,1,1.2;
```

HRT 发送不是上升沿触发，而是每条合法帧都发送，方便记录完整状态。如果某个旧版 HRT 脚本仍要求 `timestamp,trigger,scene,seq,speed;`，应在逻辑判定程序里单独做适配，不应让 Bridge 或 ESP32 继续背负旧格式。Bridge 也可以保留 HRT 转发能力用于兼容旧流程，但新主路径应由逻辑判定程序直接发给 HRT。

## 7. 数据精度

### 整数优先

时间戳、序号这类字段应尽量使用整数。整数在文本中传输时不会丢精度，解析后也稳定。

推荐：

```text
TIMESTAMP=1779235200
SCENE=boot
SEQ=1
```

不推荐：

```text
TIMESTAMP=1779235200.0000000001
SEQ=1.00001
```

### 浮点数适合观察量

`SPEED` 是观察量，可以使用浮点数，例如：

```text
SPEED=1.234
```

但 Python 官方文档提醒：二进制浮点不能精确表示多数十进制小数，例如 `0.1`。因此：

- 不要用浮点相等判断，例如 `speed == 1.2`。
- 状态判定应使用阈值，例如 `speed >= 1.0`。
- 显示时可以格式化，例如保留 2 或 3 位小数。

### 文本协议的优点

本项目选择文本协议而不是二进制结构体，是因为：

- 逻辑判定程序、Bridge、HRT 都容易调试。
- 实验员能直接看懂日志。
- 网络抓包和串口日志可读。
- 字段扩展简单，例如追加 `SPEED`。

代价是：

- 文本解析比二进制慢一点。
- 必须严格定义分隔符和结束符。
- 字段里不能随意包含分隔符。

## 8. 数据拆分与合并

### 新架构不再拆时间戳

早期方案里，SILAB 某些模块对数值范围、显示或固定数据口处理不方便，所以曾把 Unix 时间拆成两个数字：

```text
TIME_LOW  = unix_time 的低 6 位
TIME_HIGH = floor(unix_time / 100000)
```

例如：

```text
unix_time = 1779235200
TIME_LOW  = 235200
TIME_HIGH = 17792
```

新架构中，SILAB 主机上增加了 Python 逻辑判定程序。这个程序不受 SILAB 原始数据口格式限制，可以直接使用 Python 的整数、浮点数和字符串。因此公共网络协议应直接发送完整时间戳：

```text
TIMESTAMP = 1779235200
```

这样做的好处：

- Bridge 不需要合并 `TIME_LOW/TIME_HIGH`。
- HRT 记录到的是完整时间，不需要二次还原。
- 场景可以直接发 `boot`、`warning` 这样的名称，不需要再从数字别名推导。
- 协议字段更少，现场调试更直观。

### 旧版时间拆分如何合并

`TIME_HIGH` 已经包含了 100000 秒这一位。`TIME_LOW` 是低 6 位，其中最高的那一位和 `TIME_HIGH` 的边界有重叠。因此 Bridge 采用：

```text
timestamp = TIME_HIGH * 100000 + (TIME_LOW % 100000)
```

例子：

```text
TIME_LOW  = 235200
TIME_HIGH = 17792

TIME_LOW % 100000 = 35200
timestamp = 17792 * 100000 + 35200
          = 1779235200
```

这样即使 `TIME_LOW` 发 6 位，Bridge 也能恢复正确时间。

这个规则只用于解释旧版数据或兼容旧工具。新系统不建议继续把时间戳拆开发给 Bridge/HRT。

### 旧版拆分方案的边界风险

如果 SILAB 在 `TIME_HIGH` 和 `TIME_LOW` 两个数据口取值的瞬间刚好跨过边界，可能出现极小概率的不一致。比如：

```text
TIME_LOW  来自 1779299999
TIME_HIGH 来自 1779300000
```

一般固定频率实验里概率很低。需要更严格时，可以：

- 直接发送完整 Unix 时间。
- 在逻辑判定程序内先生成同一时刻的统一时间变量，再输出。
- Bridge 或 HRT 侧根据前后帧做单调性检查。

## 9. JSON 与二进制结构体

### JSON

JSON 适合 ESP32/MQTT 这类命令：

```json
{"type":"play","scene":"boot","seq":1}
```

优点：

- 字段名清楚。
- 顺序不敏感。
- 方便扩展字段。
- Python `json.dumps()` / `json.loads()` 可直接处理。

缺点：

- 比纯文本帧更长。
- 必须处理 JSONDecodeError。
- 不适合高频极小包场景。

### struct 二进制

Python `struct` 可把数字打包成固定字节格式，例如：

```python
import struct

data = struct.pack(">Iff", 1779235200, 1.0, 1.2)
timestamp, trigger, speed = struct.unpack(">Iff", data)
```

`>` 表示网络字节序/大端。二进制格式适合带宽敏感、协议稳定的场景，但不利于现场调试。本项目现阶段优先文本协议。

## 10. 项目调试建议

### 用双 Host 脚本模拟小机器人/Bridge 和 HRT

如果只想验证逻辑判定程序是否会同时向小机器人/Bridge 和 HRT 发数据，可以先不用真实 ESP32 和 HRT，运行：

```bash
python3 tools/logic_dual_host_simulator.py
```

这个脚本会同时创建两个 TCP Host：

```text
ROBOT_HOST: 0.0.0.0:7777
HRT_HOST:   0.0.0.0:9001
```

逻辑判定程序需要主动连接这两个 Host：

```text
Bridge payload: TRIGGER;TIMESTAMP;SCENE;SEQ;SPEED<LF>
HRT payload:    trigger,timestamp,scene,seq,speed;
```

`ROBOT_HOST` 会通过 `tools/voice_package.py` 接入语音包。收到触发上升沿后，它会根据 `SCENE/SEQ` 查找对应文本和 WAV 路径，例如：

```text
robot would_play common/left_rear_vehicle_merge.wav exists=true
text=左后方有车辆汇入
```

本机自检：

```bash
python3 tools/logic_dual_host_simulator.py --self-test
```

看到 `ROBOT_HOST` 和 `HRT_HOST` 都收到 `RISING` 日志，说明两个 Host 的协议解析正常。

### 用新的逻辑判定模拟器发送语音包条目

`tools/logic_decision_simulator.py` 扮演 SILAB 主机上的逻辑判定程序。它从 common 语音包选择一条或一组语音，按固定频率同时发送给小机器人/Bridge Host 和 HRT Host。

先启动双 Host：

```bash
python3 tools/logic_dual_host_simulator.py
```

再发送一条 common 语音触发：

```bash
python3 tools/logic_decision_simulator.py \
  --bridge-host 127.0.0.1 --bridge-port 7777 \
  --hrt-host 127.0.0.1 --hrt-port 9001 \
  --entry common/left_rear_vehicle_merge \
  --pattern 0:0.5,1:0.4,0:0.5 \
  --rate-hz 10
```

如果语音包不在默认相邻仓库路径，可以显式指定：

```bash
python3 tools/logic_dual_host_simulator.py --voice-package /path/to/common_voice_test_assets_20260522
python3 tools/logic_decision_simulator.py --voice-package /path/to/common_voice_test_assets_20260522
```

### 用 probe 看逻辑判定程序实际发了什么

```bash
python3 tools/silab_tcp_probe.py --host 0.0.0.0 --port 7777
```

期望看到：

```text
format: OK
parsed: timestamp=... trigger=... scene=... seq=... speed=...
hrt_device_packet: ...
```

### 用 simulator 模拟逻辑判定程序

```bash
python3 tools/silab_tcp_simulator.py --host 127.0.0.1 --port 7777 --scene boot --seq 1 --read-ack --verbose
```

默认会发送：

```text
0;1779235199;boot;1;0
1;1779235200;boot;1;1.2
0;1779235201;boot;1;0
```

### GUI 日志应该看什么

正常链路里，GUI 日志应能看到：

```text
[silab] #... play ts=... trigger=1 scene=boot seq=1 speed=1.2 hrt=ok
[direct] play boot/001 sent ...
[esp32] screen speaking ...
[audio] played /sdcard/audio/boot/001.wav ...
```

如果没有播放，优先检查：

- 逻辑判定程序是否连到了 Bridge 主机 IP 和端口。
- GUI 是否已经 `Start TCP Host`。
- `TRIGGER` 是否真的从 0 变 1。
- `SCENE` 和 `SEQ` 是否能映射到 TF 卡文件，例如 `/sdcard/audio/boot/001.wav`。
- ESP32 是否在线，TF 卡是否 mounted。

如果 HRT 没数据，优先检查：

- HRT Host IP/Port 是否正确。
- 逻辑判定程序是否启用了 HRT 发送。
- 逻辑判定程序是否已经连接 HRT。
- HRT 是否真的在监听 TCP，而不是 UDP 或串口。

## 11. 一句话总结

IP 负责找到机器，TCP 负责可靠地传字节，应用层帧格式负责把字节解释成实验数据。本项目的新关键边界是 Python 逻辑判定程序：它读取 SILAB 状态，生成统一的触发标志、时间戳、场景、序号和速度，再分别发送给 Bridge 与 HRT。Bridge 专注于控制 ESP32 播放和监控状态，不再直接理解 SILAB 原始数据格式。
