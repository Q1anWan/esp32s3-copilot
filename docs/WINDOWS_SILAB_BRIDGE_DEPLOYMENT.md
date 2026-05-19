# Windows 实验主机部署说明：SILAB Bridge + ESP32 音频播放

这份文档给非计算机专业同学使用。目标是把 Windows 实验主机配置成
SILAB 与 ESP32 之间的桥接主机，并能完成播放、停止、模拟 SILAB、检查
SILAB 数据包。

## 最终交付的 5 个文件

| 序号 | 类型 | 文件 | 用途 |
| --- | --- | --- | --- |
| 1 | 程序 | `tools/silab_mqtt_bridge_gui.py` | Bridge 可视化程序。接收 SILAB TCP 数据，控制 ESP32 播放/停止，显示 ESP32 状态 |
| 2 | 程序 | `tools/silab_tcp_simulator.py` | 模拟 SILAB 脚本。按固定频率发送 `trigger scene seq` 数据包 |
| 3 | 程序 | `tools/silab_tcp_probe.py` | SILAB 数据包测试脚本。接收真实 SILAB 数据并打印解析结果 |
| 4 | INC | `tools/silab/NOMIRobotStart_ESP32.inc` | SILAB 参考设计。说明 SILAB 如何向 Bridge 发送场景和序号 |
| 5 | 文档 | `docs/WINDOWS_SILAB_BRIDGE_DEPLOYMENT.md` | 本中文部署文档 |

注意：`silab_mqtt_bridge_gui.py` 和 `silab_tcp_probe.py` 都会监听 TCP
`7777` 端口，因此二者不能同时运行。正式实验用 GUI；调试 SILAB 数据格式
时用 probe。

## 系统拓扑

```text
SILAB --网线局域网 A / TCP 7777--> Windows 实验主机
Windows 实验主机 --WiFi 局域网 B / MQTT 1883 + TCP 7777--> ESP32
```

Windows 实验主机有两个重要 IP：

| 网络 | 给谁使用 | 例子 |
| --- | --- | --- |
| 网线局域网 A 的 IPv4 | 填到 SILAB 的 `Destination_IP` | `192.168.100.10` |
| WiFi 局域网 B 的 IPv4 | 配给 ESP32 的 MQTT Broker URI | `mqtt://192.168.0.10:1883` |

不要把 `127.0.0.1` 配给 ESP32。`127.0.0.1` 只代表 Windows 电脑自己。

## 第一步：安装 Python

1. 打开 Python 官网下载安装包：

   ```text
   https://www.python.org/downloads/windows/
   ```

2. 安装时如果看到 `Add python.exe to PATH`，请勾选。

3. 打开 PowerShell，输入：

   ```powershell
   py -3 --version
   ```

   如果能看到 Python 版本号，说明安装成功。

4. 进入项目文件夹，双击：

   ```text
   tools\windows\install_python_deps.bat
   ```

   这个脚本会安装：

   ```text
   paho-mqtt
   pyserial
   ```

   如果窗口里没有红色报错，依赖安装完成。

## 第二步：安装 Mosquitto MQTT Broker

Bridge GUI 里的 `Start Dev Broker` 只适合开发临时测试，不适合正式实验。
正式实验必须在 Windows 上安装 Mosquitto Broker 服务。

官方页面：

- 下载页：`https://mosquitto.org/download/`
- Mosquitto 2.x 局域网监听说明：`https://mosquitto.org/documentation/migrating-to-2-0/`

### 安装 Mosquitto

1. 打开 Mosquitto 下载页。
2. 下载 Windows x64 installer。
3. 安装到默认目录：

   ```text
   C:\Program Files\mosquitto
   ```

4. 如果安装程序询问是否安装服务，请选择安装服务。

### 配置允许 ESP32 连接

Mosquitto 2.x 默认直接运行时通常只允许本机连接。ESP32 是外部设备，
所以必须写配置文件。

1. 用管理员权限打开 Notepad。
2. 打开：

   ```text
   C:\Program Files\mosquitto\mosquitto.conf
   ```

3. 在文件末尾加入下面内容，或直接把文件改成下面内容：

   ```conf
   listener 1883 0.0.0.0
   allow_anonymous true
   persistence false
   ```

仓库里也提供了同样的模板：

```text
tools\windows\mosquitto-copilot.conf
```

安全提醒：`allow_anonymous true` 只适合封闭实验局域网，不要暴露到公网。

### 打开 Windows 防火墙端口

用管理员权限打开 PowerShell，执行：

```powershell
New-NetFirewallRule -DisplayName "Copilot MQTT 1883" -Direction Inbound -Action Allow -Protocol TCP -LocalPort 1883
New-NetFirewallRule -DisplayName "Copilot SILAB TCP 7777" -Direction Inbound -Action Allow -Protocol TCP -LocalPort 7777
```

### 重启 Mosquitto 服务

用管理员权限打开 PowerShell，执行：

```powershell
Restart-Service mosquitto
```

如果提示找不到服务，打开 Windows 的 `Services`，查找 `Mosquitto Broker`
或 `mosquitto`，手动启动或重启。若完全没有服务，重新运行安装程序，并选择
安装服务。

### 测试 Broker 是否正常

打开两个 PowerShell 窗口。

窗口 1：

```powershell
& "C:\Program Files\mosquitto\mosquitto_sub.exe" -h 127.0.0.1 -t test/copilot
```

窗口 2：

```powershell
& "C:\Program Files\mosquitto\mosquitto_pub.exe" -h 127.0.0.1 -t test/copilot -m hello
```

如果窗口 1 出现：

```text
hello
```

说明 Broker 本机测试通过。

## 第三步：找到 Windows 的两个 IP

打开 PowerShell：

```powershell
ipconfig
```

记录两个 IPv4 地址：

| 要找的地址 | 用途 |
| --- | --- |
| WiFi 适配器 IPv4 | ESP32 的 MQTT Broker URI |
| 以太网适配器 IPv4 | SILAB 的 `Destination_IP` |

如果分不清哪个是网线 IP，可以先拔掉网线，再执行一次 `ipconfig`。消失的那
个就是网线局域网 A。

## 第四步：启动 Bridge 可视化程序

双击：

```text
tools\windows\start_bridge_gui.bat
```

GUI 打开后按下面步骤操作：

1. 在 `MQTT Broker` 区域设置：

   ```text
   Host = 127.0.0.1
   Port = 1883
   ```

2. 点击 `Connect`。

3. 正式实验不要点击 `Start Dev Broker`。

4. 在 `ESP32 USB Setup` 区域，把 `Broker URI` 设置为 Windows 的 WiFi
   局域网 B IP：

   ```text
   mqtt://<Windows WiFi IP>:1883
   ```

   例子：

   ```text
   mqtt://192.168.0.10:1883
   ```

5. 用 USB 连接 ESP32，点击 `Save MQTT`。

6. 如果 ESP32 还没有配置 WiFi，填写 `SSID` 和 `Pass`，点击 `Save WiFi`。

7. 等待 ESP32 状态刷新。正常时应看到：

   ```text
   WiFi connected
   MQTT connected
   SD mounted
   ESP Host 自动填写，例如 192.168.0.56
   ```

8. 保持 `Direct ESP TCP` 勾选。

9. 点击 `Play`，ESP32 应播放：

   ```text
   /sdcard/audio/boot/001.wav
   ```

10. 点击 `Stop`，ESP32 应停止播放，并回到中性表情。

## 第五步：启动 SILAB TCP Host

在同一个 GUI 里：

1. 在 `SILAB TCP Host` 区域设置：

   ```text
   Bind = 0.0.0.0
   Port = 7777
   Threshold = 0.5
   Aliases = 1=boot
   ```

2. 如果 SILAB 需要接收 ACK，可以勾选 `Send TCP ACK`。

3. 点击 `Start TCP Host`。

4. 在 SILAB 里设置：

   ```text
   Destination_IP = Windows 网线局域网 A 的 IPv4
   Destination_Port = 7777
   ```

SILAB 发送格式：

```text
trigger<TAB>scene<TAB>seq<LF>
```

例子：

```text
1	1	1
```

默认 `Aliases = 1=boot`，所以这条命令会播放：

```text
/sdcard/audio/boot/001.wav
```

触发规则：

- `trigger >= 0.5` 表示触发有效。
- 只在 `0 -> 1` 上升沿播放一次。
- SILAB 固定频率连续发送 `1` 时，不会重复播放。
- 必须先回到 `0`，下一次 `1` 才会再次触发播放。

## 使用模拟 SILAB 脚本

当没有 SILAB 硬件，或正式实验前要检查链路时使用。

1. 启动 Bridge GUI。
2. 点击 MQTT `Connect`。
3. 点击 `Start TCP Host`。
4. 双击：

   ```text
   tools\windows\start_silab_simulator.bat
   ```

5. 如果模拟器和 GUI 在同一台 Windows 电脑上，`Bridge PC IP` 直接按回车，
   使用默认 `127.0.0.1`。

6. `Bridge TCP port` 直接按回车，使用默认 `7777`。

GUI 日志里应该出现类似内容：

```text
[silab] connected ...
[silab] #... play trigger=1 scene=boot seq=1
[direct] play boot/001 sent ...
[esp32] screen speaking ...
```

## 使用 SILAB 数据包测试脚本

这个脚本用于确认真实 SILAB 发来的数据格式是否正确。它只接收和打印数据，
不会控制 ESP32。

1. 关闭 Bridge GUI 的 TCP Host，或者直接关闭 Bridge GUI。
2. 双击：

   ```text
   tools\windows\start_silab_probe.bat
   ```

3. 在 SILAB 里设置：

   ```text
   Destination_IP = Windows 网线局域网 A 的 IPv4
   Destination_Port = 7777
   ```

4. 运行 SILAB 场景。

正常输出类似：

```text
format: OK
parsed: trigger=1.0 scene=1 seq=1
bridge_audio_id: scene=boot seq=1
trigger_active=True rising_edge=True
would_play: /sdcard/audio/boot/001.wav
```

脚本还会把机器可读日志追加到：

```text
silab_probe_log.jsonl
```

## 正式实验启动检查表

1. Windows 已连接网线局域网 A 和 WiFi 局域网 B。
2. Mosquitto 服务正在运行。
3. Windows 防火墙已允许 TCP `1883` 和 `7777`。
4. ESP32 已上电，TF 卡已插入。
5. Bridge GUI 已启动。
6. GUI MQTT 已连接 `127.0.0.1:1883`。
7. ESP32 状态显示 MQTT connected、SD mounted。
8. `ESP Host` 已自动填写，`Direct ESP TCP` 已勾选。
9. 手动 `Play` 和 `Stop` 均正常。
10. GUI 的 `Start TCP Host` 已启动。
11. SILAB 发送目标为 Windows 网线局域网 A IP，端口 `7777`。

## 常见问题

### ESP32 MQTT 一直 disconnected

检查：

- ESP32 的 Broker URI 是 `mqtt://<Windows WiFi IP>:1883`，不是 `127.0.0.1`。
- Mosquitto 服务正在运行。
- Windows 防火墙允许 TCP `1883`。
- ESP32 和 Windows WiFi 在同一个局域网 B。

### GUI 手动 Play 可以，但 SILAB 不触发

检查：

- GUI 的 `Start TCP Host` 是否已启动。
- SILAB 的 `Destination_IP` 是否为 Windows 网线局域网 A IP。
- Windows 防火墙是否允许 TCP `7777`。
- `silab_tcp_probe.py` 是否还在运行。probe 和 GUI 不能同时占用 `7777`。

### probe 无法启动

说明 TCP `7777` 端口已经被占用。关闭 Bridge GUI 的 TCP Host，或关闭其它
使用 `7777` 的程序。

PowerShell 检查：

```powershell
netstat -ano | findstr :7777
```

### 手动 Play 延迟大或没有声音

检查：

- `Direct ESP TCP` 是否已勾选。
- `ESP Host` 是否已填写，例如 `192.168.0.56`。
- ESP32 状态里是否有 `tcp_host`，例如 `192.168.0.56:7777`。
- TF 卡状态是否为 `mounted`。
- 当前测试音频是否存在：

  ```text
  /sdcard/audio/boot/001.wav
  ```

### Stop 无法停止

检查 ESP32 固件版本，必须包含：

```text
5b317fe Add GUI audio stop control
```

然后重新启动 Bridge GUI，确认界面里出现 `Stop` 按钮。

