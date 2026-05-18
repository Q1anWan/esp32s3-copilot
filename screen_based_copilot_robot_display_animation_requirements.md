# 屏幕型小机器人（Copilot Robot）显示内容与动画规范

**版本**：v1.0  
**日期**：2026-05-14  
**适用范围**：L3/L4 自动驾驶视频实验、大实验中的 post-failure communication 条件、以及 thesis 中不涉及信任校准的“小机器人事实解释/状态更新”子研究。  
**核心目标**：把小机器人定义为一个 **screen-based in-cabin copilot interface（屏幕型车内协驾界面）**。它可以用屏幕表情、朝向、说话动画和中性光效来提高播报的可感知性与空间指向感，但不能让屏幕单独承担解释、警告、校准或安全承诺功能。

---

## 1. 设计边界

### 1.1 小机器人是什么

本项目中的“小机器人”不是完整的人形具身机器人，而是一个放置在车内前方的 **屏幕型 copilot robot**。它的关键通道包括：

- 屏幕脸 / screen face；
- 左、右、前等方向性表情或朝向；
- 说话动画；
- 固定颜色的低强度光效；
- 内置扬声器；
- 可选的物理 yaw / pitch 或屏幕内虚拟朝向变化。

在论文和实验材料中，推荐统一使用：

```text
screen-based copilot robot
in-cabin copilot robot
small screen-based robot
copilot robot display
```

不建议继续使用：

```text
embodied robot
humanoid robot
robot warning system
takeover warning device
```

原因是当前研究目标不是证明“具身性”本身，也不是设计接管警报，而是研究由小机器人这一统一车内界面发出的事实解释或状态更新如何影响理解、情境信任提高、可预测性感知、工作负荷和交互体验。

---

## 2. 实验有效性总原则

### 2.1 屏幕不能成为新的实验因子，除非被明确操控

如果所有条件都使用小机器人播报，那么小机器人是 **统一播报通道**，不是独立实验因子。此时屏幕动画必须保持高度一致，不能在某些条件中额外提供语义信息。

正确原则：

```text
语义信息主要来自语音；
屏幕只负责：提示“我要说话了”、显示“正在说话”、提供粗粒度方向感；
屏幕不能额外告诉用户“为什么”“是否危险”“是否需要准备接管”。
```

错误做法：

```text
What-only 语音 + 屏幕显示行人图标
空白对照 + 屏幕显示“监测行人”
事实解释组 + 屏幕显示绿色“安全”
校准组 + 屏幕闪红并显示“请接管”
```

这些都会污染条件操控。

---

### 2.2 Thesis 主分析不做信任校准，屏幕不能泄露校准信息

你的 thesis 主线不做 trust calibration。因此，在 thesis 主分析使用的数据中，屏幕不应显示：

- 能力边界；
- residual uncertainty；
- low confidence；
- readiness / intervention；
- “请准备接管”；
- “可能识别不确定”；
- “系统仍在验证”；
- 红色风险等级；
- 倒计时；
- 安全保证；
- 危险预测。

这些内容如果出现在大实验的 G2/G4 校准条件中，也应主要通过语音操控，并在数据日志中标记为：

```text
calibration_content_present = 1
```

你的 thesis 主分析应筛选：

```text
calibration_content_present = 0
```

---

### 2.3 屏幕不能替代语音解释

小机器人屏幕可以做：

```text
中性待机
预播报注意 cue
方向性脸部朝向
说话动画
播报结束回中
```

小机器人屏幕不应做：

```text
长文字解释
事件原因图标
风险图标
复杂地图
目标框
置信度条
视觉化推理链
安全承诺
接管提示
```

这样可以保持研究问题干净：

```text
事实性语音解释 / 状态更新是否有效？
屏幕 cue 是否让播报更容易被注意和定位？
```

而不是变成：

```text
复杂视觉 HMI 是否提高信任？
```

---

## 3. 屏幕状态机

正式实验中建议实现一个统一 screen state machine。所有条件都调用同一状态机，只是某些状态是否触发、触发频率和语音内容不同。

| 状态 ID | 名称 | 视觉表现 | 是否含语义 | 适用条件 |
|---|---|---|---|---|
| S0 | Neutral Idle | 低亮中性脸或中性点阵，静止 | 否 | 所有组 |
| S1 | Pre-message Orienting Cue | 眼睛/脸部朝向左、右、前；可有柔和光圈 | 否，只有方向感 | 有播报条件 |
| S2 | Speaking | 中性嘴部动画，保持当前方向 | 否，语义来自语音 | 有播报条件 |
| S3 | Return to Neutral | 回正、光效淡出 | 否 | 有播报条件 |
| S4 | Silent Neutral Control | 与 S0 一致，不播报、不变化 | 否 | G0 / blank / minimal control |
| S5 | Debug / Engineering Only | 显示时间戳、event_id、状态 | 是，仅工程 | 禁止 participant-facing |

### 3.1 S0 Neutral Idle

S0 是所有条件的默认状态。建议使用：

- 低亮度；
- 无闪烁；
- 无文字；
- 无风险颜色；
- 无持续运动；
- 无明显“故障”感；
- 可以是简单中性脸、两个点状眼睛、或极简待机标识。

不建议使用纯黑屏作为默认状态，尤其在小机器人屏幕设备上。纯黑屏容易被被试理解为“设备关机”“机器人坏了”或“这组没有机器人”，从而引入混淆。更稳妥的是：

```text
屏幕低亮待机，但不表达任何语义。
```

### 3.2 S1 Pre-message Orienting Cue

S1 用于语音前的注意定向。要求：

```text
t_cue = t_speech - 0.5s
```

可接受范围：

```text
0.3s ≤ pre-cue duration ≤ 0.8s
```

表现方式：

- 屏幕脸看向事件大致方向；
- 或屏幕内眼睛/脸部偏向事件大致方向；
- 可叠加低强度固定色光圈；
- 不显示目标图标；
- 不显示文字；
- 不显示危险等级。

### 3.3 S2 Speaking

S2 与语音同步。要求：

- 嘴部动画从语音开始时启动；
- 嘴部动画在语音结束时停止；
- 音画同步误差目标不超过 ±100 ms；
- 嘴部动画强度中性，不夸张；
- 不使用夸张表情；
- 不根据“危险程度”改变颜色、速度或表情；
- 不显示语音字幕，除非实验明确把字幕作为新因子。

### 3.4 S3 Return to Neutral

语音结束后，屏幕应回到 S0。建议：

```text
return_duration = 0.3s – 0.8s
```

不建议在事件结束后继续显示“监控中”“安全通过”“已解决”等文字，因为这些属于额外状态反馈，会改变实验条件。

### 3.5 S4 Silent Neutral Control

G0 / blank / minimal control 条件中，小机器人应保持与其他条件相同的物理存在和中性待机状态。推荐：

```text
屏幕低亮中性待机；
不转向；
不说话；
不显示语义文字；
不显示监控、风险、接管、解释类图标。
```

如果 G0 使用 minimal operational message，例如 “Automated driving has resumed”，则该消息必须被记录为 G0 的最低限度状态提示，不应带原因、不应带校准、不应带屏幕方向 cue。

### 3.6 S5 Debug / Engineering Only

调试时可以显示：

- event_id；
- group_id；
- t_cue；
- t_speech；
- sync status；
- calibration_content_present；
- robot state。

但这些调试内容必须从 participant-facing 版本中移除。正式视频或现场实验中不能让被试看见。

---

## 4. 屏幕内容要求

### 4.1 正式推荐：不显示语义文字

屏幕型小机器人最稳的 participant-facing 方案是：

```text
不显示长文本；
不显示解释文本；
不显示字幕；
不显示事件原因；
不显示系统置信度；
不显示风险等级。
```

理由：

1. 文字会引导被试看屏幕，增加视觉负荷；
2. 文字可能与语音内容形成重复或冲突；
3. 文字会使屏幕成为新的解释通道；
4. 文字更容易在 What-only、What+Why、blank 和 calibration 条件之间产生条件污染。

### 4.2 如果必须显示文字

只有在大实验团队强制要求状态文字时，才允许使用极短文字。此时必须遵守：

| 项目 | 要求 |
|---|---|
| 字数 | 中文不超过 4–6 个字，英文不超过 2–3 个词 |
| 内容 | 只能是非语义状态，例如“自动驾驶中”“播报中” |
| 禁止 | “安全”“危险”“接管”“不确定”“低置信”“行人”“障碍”“让行原因” |
| 字体 | 大字号，实验室座位距离下一眼可读 |
| 对比度 | 正常文字至少 4.5:1；大字至少 3:1 |
| 显示时长 | 与语音同步，不额外延长 |
| 条件一致性 | 同类消息在所有组中视觉格式一致 |

推荐保留的文字只有：

```text
自动驾驶中
播报中
```

不推荐：

```text
监测行人
保持安全
请注意
准备接管
识别不确定
路口风险
```

---

## 5. 动画参数规范

### 5.1 动画总原则

屏幕动画必须：

- 短；
- 慢；
- 中性；
- 不惊吓；
- 不闪烁；
- 不诱导风险等级；
- 不吸引长时间注视；
- 不改变语音信息的含义。

### 5.2 建议参数

| 参数 | 推荐值 | 不推荐 |
|---|---:|---|
| pre-cue 时间 | 0.5 s，预实验可调到 0.3–0.8 s | >1.0 s，像预警 |
| 转向/朝向过渡 | 250–500 ms | 瞬间跳变 |
| 回正过渡 | 300–800 ms | 长时间停留 |
| 嘴部动画 | 跟随语音，低幅度 | 大幅夸张跳动 |
| 闪烁 | 不使用 | 高频闪、红光闪 |
| 循环动效 | 不使用或极少 | 持续呼吸灯、滚动条 |
| 待机动画 | 静态或极轻微 | 持续眨眼、游动、漂浮 |
| 亮度变化 | 柔和淡入淡出 | 快速明暗切换 |
| 声画同步 | 目标 ±100 ms | 明显提前/滞后 |

### 5.3 眨眼与表情

推荐：

```text
不使用持续自动眨眼。
```

如果为了设备自然性必须使用眨眼：

- 间隔不短于 6–8 秒；
- 不在关键交通事件窗口内主动眨眼；
- 不与语音开始/结束绑定；
- 不使用高亮/高对比眨眼；
- 不让眨眼成为注意 cue。

不建议使用：

- 惊讶脸；
- 担心脸；
- 开心脸；
- 道歉脸；
- 皱眉；
- 汗滴；
- 警报表情；
- 拟人化“紧张”表情。

原因是情绪表情会改变被试对系统能力、责任、错误严重性和社交代理感的判断。

---

## 6. 颜色与光效规范

### 6.1 颜色功能

屏幕颜色在本实验中只用于：

```text
区分待机 / 播报中；
提高屏幕可见性；
维持设备存在感。
```

颜色不能用于：

```text
表达危险程度；
表达系统置信度；
表达是否需要接管；
表达是否安全；
表达事件类别。
```

### 6.2 推荐色彩策略

| 用途 | 建议 |
|---|---|
| 背景 | 深色或低亮中性背景 |
| 面部线条 | 柔和浅色 |
| 播报状态 | 固定柔和蓝/青/白类低饱和颜色 |
| 光圈 | 全实验固定一种颜色 |
| 禁止色 | 红色、强橙色、强黄闪烁 |
| 风险编码 | 禁止 |
| 条件编码 | 禁止 |

### 6.3 为什么不使用红色/橙色风险色

红色、橙色和高饱和黄色容易被理解为：

- 警报；
- 危险；
- 接管；
- 系统故障；
- 高风险；
- 强烈注意请求。

这会污染“事实解释是否有效”的测量，并可能把 thesis 推向 trust calibration / takeover readiness 框架。

---

## 7. 可访问性与安全要求

### 7.1 闪烁限制

Participant-facing 屏幕不得出现高频闪烁。工程上应满足：

```text
任何高对比亮度变化不得超过 3 次/秒；
不使用红色高饱和闪烁；
不使用全屏快速明暗切换；
不使用类似警报灯的循环闪烁。
```

### 7.2 动画停止原则

任何非语音必要动画不应持续超过 5 秒。若工程上存在持续动画，应确保：

- 可关闭；
- 不在正式实验中开启；
- 不与视频道路画面竞争注意力。

### 7.3 对比度

若显示文字：

```text
普通文字：对比度 ≥ 4.5:1
大号文字：对比度 ≥ 3:1
```

若显示图形 cue 或 UI 状态：

```text
关键非文本图形对比度 ≥ 3:1
```

但再次强调：正式主实验最好不使用文字和语义图标。

### 7.4 视觉负荷

屏幕显示必须尽量避免吸引长时间注视。建议在预实验中记录或观察：

- 被试看屏幕是否明显超过道路视频；
- 被试是否反复阅读屏幕；
- 被试是否报告“屏幕太吸引注意”；
- 被试是否因屏幕错过道路事件；
- 被试是否认为屏幕像警报器。

保守验收标准：

```text
屏幕单次可理解 glance 应小于 2 秒；
屏幕不应要求连续阅读；
屏幕不应要求手动交互。
```

---

## 8. 大实验条件中的屏幕一致性

### 8.1 G0：Blank / Minimal Control

屏幕要求：

```text
S0 / S4 中性待机；
无方向 cue；
无嘴部动画；
无事件图标；
无解释文字；
无风险色；
不显示“监测中”“准备接管”“系统验证中”。
```

如果 G0 有 minimal operational message：

```text
语音可以只说 “Automated driving has resumed.”
屏幕只进入 S2 speaking，不显示额外文本或方向。
```

### 8.2 G1：Progressive Factual Transparency

屏幕要求：

```text
可使用 S1 → S2 → S3；
方向 cue 只表示大致左/右/前；
屏幕不显示原因文字；
屏幕不显示目标图标；
屏幕不显示安全保证。
```

语义由语音承担，例如：

```text
“The stop was triggered by a pedestrian-like object near the roadside.”
```

屏幕只显示中性说话动画。

### 8.3 G2：Fixed Progressive Calibration Repair

大实验中 G2 语音包含 calibration elements，但屏幕仍建议保持与 G1 相同的视觉格式。不要额外显示：

```text
uncertainty icon
risk bar
confidence percentage
ready-to-intervene indicator
red/orange warning color
```

否则 G2 不只是“calibration content”，还变成了“calibration visual HMI”。

### 8.4 G3：Adaptive Factual Transparency

屏幕要求与 G1 一样，但触发时机和频率由 adaptive policy 决定。屏幕不能显示“我正在减少播报”“低风险所以沉默”等元信息。

### 8.5 G4：Adaptive Progressive Calibration Repair

G4 是大实验理论最强组，但为了保护你的 thesis 边界，屏幕仍应保持非语义视觉格式。Calibration 信息通过语音操控并记录，不通过屏幕额外强化。

---

## 9. Thesis 子研究中的屏幕要求

你的 thesis 不做信任校准，建议主分析只使用：

```text
G0
G1
G3
```

或者使用所有 `calibration_content_present = 0` 的事件。

屏幕层面必须保证：

| 项目 | 要求 |
|---|---|
| 小机器人在所有组中位置一致 | 是 |
| 声音均从小机器人发出 | 是 |
| 屏幕待机存在感一致 | 是 |
| G0 不显示事件语义 | 是 |
| G1/G3 屏幕不显示 Why 文字 | 是 |
| G1/G3 屏幕不显示风险/校准内容 | 是 |
| 语义差异来自语音脚本 | 是 |
| 屏幕只作为注意和说话状态 cue | 是 |

推荐 thesis 中这样表述：

> The screen-based copilot robot was used as a standardized in-cabin communication interface. Its display provided only neutral attentional and speaking-state cues. It did not display semantic explanations, risk levels, uncertainty, or takeover-readiness information. Therefore, the thesis analysis focuses on robot-delivered factual updates rather than trust calibration.

---

## 10. 事件方向映射

屏幕方向 cue 只做粗粒度方向提示，不做精确目标定位。

| 场景方向 | 屏幕表现 | 允许 | 禁止 |
|---|---|---|---|
| 前方事件 | 正面脸 / 中央眼睛 | 中性正面说话 | 前方障碍图标 |
| 左侧事件 | 眼睛/脸部向左 | 左偏 15–30° 视觉朝向 | 左侧车辆图标 |
| 右侧事件 | 眼睛/脸部向右 | 右偏 15–30° 视觉朝向 | 右侧行人图标 |
| 路口/全局 | 正面或轻微扫视后回正 | 中性 | 红黄绿信号灯图标 |
| 遮挡/模糊 | 正面或相关方向 | 中性 | “不确定”“遮挡风险”图标 |
| 故障后解释 | 正面 | 中性说话 | 道歉脸、惊讶脸、红警报 |

---

## 11. 屏幕与语音脚本的关系

### 11.1 屏幕不得扩大语音语义

示例：

| 语音条件 | 屏幕可显示 | 屏幕不可显示 |
|---|---|---|
| What-only: “车辆将减速。” | 中性脸说话 | 行人图标、障碍图标 |
| What+Why: “前方车辆正在并入，车辆将减速。” | 中性脸说话、前向朝向 | “安全车距”“风险降低” |
| Factual update: “Approaching an intersection.” | 中性脸说话 | cross-traffic risk bar |
| Calibration update | 中性脸说话 | uncertainty meter、ready icon |

### 11.2 屏幕不得承担字幕功能

字幕会增加阅读负担，并可能让用户将屏幕作为主要信息来源。除非研究专门比较“有无字幕”，否则不应启用字幕。

如果为了无障碍必须有字幕，应把字幕作为所有组一致的辅助通道，并在论文中报告；但这会改变实验解释，需要重新评估。

---

## 12. 日志要求

每条屏幕事件必须记录日志，确保后续能从大实验中抽取 thesis 数据。

建议字段：

```csv
participant_id
session_id
group_id
phase_id
scene_id
message_id
message_category
calibration_content_present
visual_semantic_content_present
screen_state
screen_orientation
screen_text
screen_icon
screen_color_profile
animation_profile
brightness_level
t_state_start
t_state_end
t_cue
t_speech_start
t_speech_end
audio_file_id
sync_error_ms
dropped_frame_count
robot_display_version
robot_firmware_version
asset_version_hash
experimenter_note
```

关键字段必须有：

```text
calibration_content_present
visual_semantic_content_present
screen_text
screen_icon
screen_orientation
t_cue
t_speech_start
t_speech_end
sync_error_ms
asset_version_hash
```

这样才能证明 thesis 主分析没有混入 calibration content 或视觉语义内容。

---

## 13. Participant-facing 验收标准

正式采集前，屏幕界面必须通过预实验或内部评审。

### 13.1 可感知性

| 指标 | 建议标准 |
|---|---|
| 有播报条件下，被试知道声音来自小机器人 | ≥ 80% |
| Cue-on 条件下，被试注意到屏幕/朝向变化 | ≥ 70% |
| 被试能理解屏幕是“播报提示”而非“警报” | ≥ 70% |
| G0 中被试不认为小机器人故障 | ≥ 70% |

### 13.2 不干扰

| 指标 | 建议标准 |
|---|---|
| 被试报告屏幕“很分心” | 不超过 20% |
| 被试报告光效像危险警报 | 不超过 20% |
| 被试报告需要读屏幕才能完成任务 | 不超过 20% |
| 被试因屏幕错过道路事件 | 不超过 20% |

### 13.3 不泄露条件

| 检查 | 通过标准 |
|---|---|
| G0 屏幕是否泄露事件信息 | 否 |
| What-only 屏幕是否泄露 Why | 否 |
| Factual-only 屏幕是否泄露 calibration | 否 |
| 颜色是否编码风险等级 | 否 |
| 屏幕是否使用安全承诺 | 否 |

### 13.4 工程同步

| 项目 | 通过标准 |
|---|---|
| t_cue 与 t_speech | 约 0.5s 间隔 |
| 嘴部动画与语音 | ±100 ms 内 |
| 屏幕状态日志 | 完整 |
| 语音文件版本 | 完整 |
| 屏幕 asset version hash | 完整 |
| participant-facing 禁用 debug | 是 |

---

## 14. 禁用清单

正式实验中禁止以下屏幕内容。

### 14.1 禁用视觉符号

```text
红色警报灯
危险三角形
感叹号
接管方向盘图标
行人危险图标
碰撞图标
倒计时
置信度条
风险等级条
热力图
传感器扫描动画
目标检测框
HUD 式路面框选
```

### 14.2 禁用文字

```text
危险
安全
请接管
准备接管
低置信
高风险
系统不确定
请监控
确保安全
不会出错
已解决风险
识别失败
验证中
```

### 14.3 禁用表情

```text
惊恐
担心
开心庆祝
抱歉流汗
皱眉
哭脸
眨眼卖萌
夸张点头
摇头否定
```

### 14.4 禁用动画

```text
高频闪烁
红色闪烁
强烈呼吸灯
持续旋转
弹跳
震动
全屏明暗闪
事件目标追踪动画
滚动字幕
```

---

## 15. 推荐视觉资产

### 15.1 Face assets

建议准备：

```text
face_idle_front.png / json
face_idle_left_soft.png / json
face_idle_right_soft.png / json
face_speaking_front_01-04
face_speaking_left_01-04
face_speaking_right_01-04
face_return_neutral
```

### 15.2 Light assets

建议准备：

```text
ring_off
ring_soft_on
ring_soft_pulse_once
ring_fade_out
```

禁止：

```text
ring_red_alarm
ring_fast_blink
ring_risk_level_1_2_3
```

### 15.3 Animation profiles

建议三个 profile：

| Profile | 用途 | 参数 |
|---|---|---|
| neutral_idle | 默认状态 | 静态或极轻微 |
| pre_message_soft | 播报前 cue | 0.5s，淡入，方向朝向 |
| speaking_neutral | 说话中 | 低幅度嘴动，固定方向 |
| return_neutral | 播报结束 | 0.3–0.8s 回正 |

---

## 16. 工程接口建议

建议将屏幕控制与语音播报统一由一个 `robot_display_timeline.json` 驱动。

示例：

```json
{
  "suite_id": "pilot_l3_post_failure_v1",
  "display_version": "screen_copilot_v1.0",
  "events": [
    {
      "message_id": "S3_M02",
      "scene_id": "intersection_01",
      "group_id": "G1",
      "phase_id": "post_failure_nominal",
      "calibration_content_present": false,
      "visual_semantic_content_present": false,
      "t_cue": 42.50,
      "t_speech_start": 43.00,
      "t_speech_end": 46.20,
      "orientation": "front",
      "screen_sequence": [
        {
          "state": "pre_message_orient",
          "start": 42.50,
          "end": 43.00,
          "asset": "face_idle_front",
          "ring": "ring_soft_on"
        },
        {
          "state": "speaking",
          "start": 43.00,
          "end": 46.20,
          "asset": "face_speaking_front",
          "ring": "ring_soft_on"
        },
        {
          "state": "return_neutral",
          "start": 46.20,
          "end": 46.80,
          "asset": "face_return_neutral",
          "ring": "ring_fade_out"
        }
      ]
    }
  ]
}
```

---

## 17. 与 HREP / Safety 的关系

### 17.1 HREP 文本中应如何描述

在伦理材料中，小机器人屏幕可以描述为：

> A small screen-based in-cabin copilot robot will be placed in front of the participant. In some conditions, it will provide neutral visual cues, such as a simple screen face, directional orientation, and speaking animation, synchronized with audio messages. The display will not present flashing lights, alarm signals, safety-critical warnings, or takeover commands unless explicitly approved in the protocol.

中文可写：

> 实验中将在被试前方放置一个屏幕型车内小机器人。在部分条件下，小机器人会显示中性的屏幕表情、方向性朝向和说话动画，并与语音播报同步。屏幕不会显示高频闪烁、警报灯、安全承诺或接管命令，除非相关内容已在伦理方案中明确说明并获批。

### 17.2 Safety Protocol 中应如何描述

Safety 文档中建议写：

> The copilot robot display is a low-voltage consumer/prototype electronic device used for visual and audio presentation. It will be placed securely in the experimental setup. The display will use low-intensity, non-flashing animations and will not involve laser, high voltage, high temperature, radiation, biological, chemical, or other hazardous materials.

中文可写：

> 小机器人屏幕为低压电子显示与音频设备，仅用于呈现中性视觉动画和语音播报。设备将固定放置，不涉及激光、高压、高温、辐射、化学品、生物材料或其他危险物质。正式实验中不使用高频闪烁或强警报光效。

---

## 18. 论文 Method 可直接使用的表述

英文版：

> The copilot robot was implemented as a small screen-based in-cabin interface. The display showed a neutral face-like idle state and, during robot-delivered messages, a short pre-speech orienting cue followed by a synchronized speaking animation. The display did not present semantic explanations, subtitles, risk levels, confidence values, uncertainty indicators, or takeover-readiness cues. This design ensured that the semantic manipulation was carried by the audio message content, while the screen animation served only to support message noticeability, speaking-state awareness, and coarse spatial grounding.

中文版：

> 本研究中的小机器人被实现为一个屏幕型车内协驾界面。屏幕在默认状态下显示低亮中性表情；在小机器人播报时，屏幕先呈现短暂的语音前方向性 cue，随后显示与语音同步的说话动画。屏幕不显示语义解释、字幕、风险等级、置信度、不确定性提示或接管准备提示。该设计保证实验语义操控主要由语音内容承担，屏幕动画仅用于提高播报可感知性、说话状态感知和粗粒度空间指向感。

---

## 19. 最终冻结建议

建议冻结以下决策：

1. 小机器人统一称为 **screen-based copilot robot**。
2. 屏幕默认使用低亮中性待机，不使用纯黑“关机感”。
3. 屏幕不显示长文本、不显示字幕、不显示事件原因。
4. 屏幕不显示风险等级、置信度、不确定性或接管提示。
5. 屏幕颜色固定，不编码风险或条件。
6. 屏幕动画只包含：
   - pre-message orienting cue；
   - speaking animation；
   - return to neutral。
7. Pre-cue 默认 0.5s，预实验可调到 0.3–0.8s。
8. 嘴部动画与语音同步，误差目标 ±100 ms。
9. 不使用高频闪烁、红色警报、强烈呼吸灯。
10. 日志必须记录所有屏幕状态、版本、时间戳和是否包含 calibration/semantic visual content。
11. Thesis 主分析只使用 `calibration_content_present = 0` 的数据。
12. G2/G4 若保留在大实验中，屏幕也不额外显示 calibration visualizations，除非另立实验因子。

---

## 20. 参考资料

1. NHTSA. *Visual-Manual NHTSA Driver Distraction Guidelines for In-Vehicle Electronic Devices*. Federal Register, 2013.  
   https://www.federalregister.gov/documents/2013/04/26/2013-09883/visual-manual-nhtsa-driver-distraction-guidelines-for-in-vehicle-electronic-devices

2. NHTSA. *Visual-Manual NHTSA Driver Distraction Guidelines Initial Notice*. 2012.  
   https://www.nhtsa.gov/sites/nhtsa.gov/files/distraction_npfg-02162012.pdf

3. W3C. *WCAG 2.2 — Success Criterion 1.4.3 Contrast (Minimum)*.  
   https://www.w3.org/WAI/WCAG22/Understanding/contrast-minimum.html

4. W3C. *WCAG 2.1 — Success Criterion 1.4.11 Non-text Contrast*.  
   https://www.w3.org/WAI/WCAG21/Understanding/non-text-contrast.html

5. W3C. *WCAG 2.2 — Success Criterion 2.3.2 Three Flashes*.  
   https://www.w3.org/WAI/WCAG22/Understanding/three-flashes.html

6. W3C. *WCAG 2.1 — Success Criterion 2.2.2 Pause, Stop, Hide*.  
   https://www.w3.org/WAI/WCAG21/Understanding/pause-stop-hide.html

7. Koo, J., Kwac, J., Ju, W., Steinert, M., Leifer, L., & Nass, C. *Why did my car just do that? Explaining semi-autonomous driving actions to improve driver understanding, trust, and performance*. International Journal on Interactive Design and Manufacturing, 2015.  
   https://www.wendyju.com/publications/WhyDidMyCarJustDoThat.pdf

8. Zang, J., & Jeon, M. *The Effects of Transparency and Reliability of In-Vehicle Intelligent Agents on Driver Perception, Takeover Performance, Workload and Situation Awareness in Conditionally Automated Vehicles*. Multimodal Technologies and Interaction, 2022.  
   https://vtechworks.lib.vt.edu/items/7d445a76-6a21-4de4-ad35-23e35331f8b7

9. Carsten, O., & Martens, M. H. *How can humans understand their automated cars? HMI principles, problems and solutions*. Cognition, Technology & Work, 2019.  
   https://link.springer.com/article/10.1007/s10111-018-0484-0

10. Admoni, H., & Scassellati, B. *Social Eye Gaze in Human-Robot Interaction: A Review*. Journal of Human-Robot Interaction, 2017.  
    https://scazlab.yale.edu/sites/default/files/files/273-2310-1-PB.pdf

11. SAE J3016 User Guide, Koopman. *SAE J3016: Taxonomy and Definitions for Terms Related to Driving Automation Systems*.  
    https://users.ece.cmu.edu/~koopman/j3016/
