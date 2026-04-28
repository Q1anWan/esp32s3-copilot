
# ESP32 + LVGL 9.4.0（466×466 圆屏）NOMI 3.0 风格「符号表情」实现文档（给 AI 编码用）

> 当前实现已经切换为 `components/copilot/copilot_ui.cpp` 中的橡皮管风格头部，
> 并且公开动画状态只保留 `idle` 和 `speaking`。本文件是早期视觉方案参考，
> 不是当前交付口径。接手工程师优先阅读 `docs/ENGINEER_HANDOFF.md` 和
> `README.md`。

> 目标：用 **极简符号**（圆/弧/线/胶囊形）实现类似 NOMI 3.0 的表情动画。  
> 核心原则：**仅 1 个自绘 FaceWidget**（lv_obj）完成整张脸绘制；上位机通过 MQTT 发送“参数目标值”；设备端做 **插帧/平滑/常驻微动/播报联动**。

---

## 0. 约束与工程结论

- 屏幕：466×466 圆形屏
- GUI：LVGL 9.4.0
- UI 结构：**FaceWidget 单对象自绘**（禁止多层贴图对象树；禁止常态全屏 canvas buffer）
- 绘制方式：在 `LV_EVENT_DRAW_MAIN` 事件中使用 `lv_draw_rect / lv_draw_arc / lv_draw_line` 完成眼睛/嘴/符号绘制
- 刷新：30 FPS（33ms）优先，最低不低于 20 FPS
- 控制：MQTT（JSON 先行，后续可换 binary）

---

## 1. 视觉风格定义（NOMI 3.0 对齐）

- 背景：纯黑
- 元素：纯白（眼睛胶囊形 + 瞳孔点 + 嘴弧线/开口胶囊）
- 动态：通过 **曲率、开合、位置、节奏**表达情绪；而不是纹理细节
- “活着感”：呼吸（整体微缩放/上下浮动）+ 眨眼 + 轻微扫视 + 播报口型

---

## 2. 坐标系与默认几何（466×466）

### 2.1 屏幕与安全圆
- `W = 466, H = 466`
- 屏幕中心：`C = (233, 233)`
- 安全半径：`R_safe = 223`（留 10px 边缘余量，避免穿帮）

归一化坐标到像素：
- `px = Cx + nx * R_safe`
- `py = Cy + ny * R_safe`
- 其中 `nx, ny ∈ [-1, 1]`

### 2.2 线宽与尺寸（默认）
> 下面数值是“开箱即像”的默认值，可后续微调

- 白色：`COL_FG = #F6F6F6`（或纯白 #FFFFFF）
- 黑色：`COL_BG = #000000`
- 眼睛胶囊形填充：白色填充（可无描边）
- 嘴弧线：白色“粗弧线”
- 推荐线宽（按 466px 设计）：
  - `LW_EYE = 12`
  - `LW_MOUTH = 14`
  - `LW_ARC = 14`

> 若后续适配其他分辨率：线宽可用 `LW = round(W * 0.025)` 这类比例法缩放。

### 2.3 眼睛默认几何（两只对称）
眼睛中心点（归一化）：
- 左眼：`E_L = (-0.28, -0.12)`
- 右眼：`E_R = (+0.28, -0.12)`

眼睛胶囊形（最大睁眼）尺寸：
- `EYE_W = 0.28 * R_safe ≈ 62 px`
- `EYE_H_MAX = 0.10 * R_safe ≈ 22 px`
- 圆角：`radius = EYE_H/2`（胶囊形）

瞳孔点（默认）：
- `PUPIL_R = 6 px`

### 2.4 嘴默认几何
嘴中心点（归一化）：
- `M = (0.0, +0.20)` → `My ≈ 233 + 0.20*223 ≈ 278`

嘴（闭口弧线）：
- 宽：`MOUTH_W = 0.40 * R_safe ≈ 89 px`
- 基准半径：`MOUTH_R = 0.33 * R_safe ≈ 74 px`（用于画弧线）

嘴（开口胶囊）：
- `MOUTH_OPEN_W = 0.34 * R_safe ≈ 76 px`
- `MOUTH_OPEN_H_MAX = 0.18 * R_safe ≈ 40 px`

---

## 3. 参数系统（Rig）与默认参数表

### 3.1 参数定义（统一范围）
> 上位机与设备端使用同一套参数语义与范围。

| 参数 | 范围 | 含义 | 默认值（neutral） |
|---|---:|---|---:|
| `eye_open` | 0..1 | 眼睛开合（0=闭眼，1=全开） | 0.92 |
| `eye_squint` | 0..1 | 眯眼/挤压（0=无，1=很眯） | 0.05 |
| `gaze_x` | -1..1 | 视线水平偏移（左负右正） | 0.00 |
| `gaze_y` | -1..1 | 视线垂直偏移（上负下正） | 0.00 |
| `mouth_open` | 0..1 | 嘴开合（播报驱动） | 0.00 |
| `mouth_curve` | -1..1 | 嘴角弯曲（负=不悦/哭，正=笑） | +0.15 |
| `breath_phase` | 0..1 | 呼吸相位（设备端内部生成） | 0.00 |
| `mood` | -1..1 | 总体情绪倾向（可选，用于模板） | 0.00 |
| `icon` | enum | 叠加符号（none/sweat/heart/star/anger） | none |
| `icon_alpha` | 0..1 | 符号透明度 | 0.0 |

### 3.2 参数裁剪（必须）
- 所有参数在进入渲染前必须 clamp 到范围内
- MQTT 输入非法字段忽略；超范围值 clamp 并上报 `warn`

---

## 4. 参数到几何的映射（渲染规则）

> 关键点：**FaceWidget 只读取“当前参数 current”**。  
> 平滑/插帧发生在动画引擎中（见第 6 节）。

### 4.1 呼吸（breath）叠加到整体
呼吸相位：`phase ∈ [0,1)`  
呼吸波形：
- `b = sin(2π * phase)`
- 缩放：`scale = 1.0 + 0.012 * b`  （幅度 1.2%）
- 上下浮动：`dy = -2.0 * b` px

实现方式（推荐）：
- 不对对象做 `scale`（会触发更重的变换开销）
- 直接在绘制时把所有 feature 的 `y` 加上 `dy`，并在计算宽高时乘 `scale`（即“几何层面缩放”）

### 4.2 眼睛胶囊形高度（eye_open + eye_squint）
定义平滑函数（建议）：
- `s(x) = x*x*(3 - 2*x)`  （smoothstep）

眼高计算：
- `open = s(eye_open)`
- `squ = eye_squint`
- `EYE_H = lerp(2 px, EYE_H_MAX, open) * (1.0 - 0.55*squ)`
- 最小高度不低于 2px

闭眼阈值：
- 若 `EYE_H < 4px`：绘制为 **短线**（或细弧线），不画瞳孔

### 4.3 瞳孔位置（gaze）
瞳孔最大偏移（与眼大小相关）：
- `dx_max = 0.18 * EYE_W`
- `dy_max = 0.22 * EYE_H_MAX`

瞳孔偏移：
- `dx = gaze_x * dx_max`
- `dy = gaze_y * dy_max`

为了避免瞳孔穿出眼白：
- `dx = clamp(dx, -dx_max, +dx_max)`
- `dy = clamp(dy, -dy_max, +dy_max)`
- 若眼接近闭合（`EYE_H < 8px`），将 `dy` 衰减：`dy *= (EYE_H / 8px)`

### 4.4 嘴形（mouth_open / mouth_curve）
嘴分两种渲染模式：

#### 模式 A：闭口弧线（mouth_open 小）
当 `mouth_open < 0.12`：
- 画一条白色粗弧线表达情绪
- `k = mouth_curve`（-1..1）

建议用“弧线角度 + 半径”控制：
- 笑（k > 0）：画**下半弧**（类似 ∪），弧心在嘴中心下方
- 不悦/哭（k < 0）：画**上半弧**（类似 ∩），弧心在嘴中心上方

计算：
- `abs_k = abs(k)`
- 弧半径：`r = lerp(64px, 96px, 1 - abs_k)`（越夸张弯曲越小半径）
- 角度跨度：`span = lerp(40°, 120°, abs_k)`（越夸张跨度越大）

角度定义（LVGL 采用度数）：
- 若 k >= 0（笑）：
  - `start = 270° - span/2`
  - `end   = 270° + span/2`
  - 弧心：`(Mx, My + (r * 0.35))`
- 若 k < 0（不悦）：
  - `start = 90° - span/2`
  - `end   = 90° + span/2`
  - 弧心：`(Mx, My - (r * 0.35))`

> 注：角度方向以 LVGL 实测为准；若方向反了，交换 start/end 或调整基准角（90/270）。

#### 模式 B：开口胶囊（mouth_open 大，播报时）
当 `mouth_open >= 0.12`：
- 画一个“外白内黑”的胶囊形嘴（非常 NOMI）
- 高度随 `mouth_open`：
  - `h = lerp(8px, MOUTH_OPEN_H_MAX, s(mouth_open))`
- 宽度可随 open 略变化：
  - `w = MOUTH_OPEN_W * (1.0 - 0.08*s(mouth_open))`（张嘴略收窄更可爱）

绘制顺序：
1) 画外轮廓（白色填充，或白色边框）
2) 再画内腔（黑色填充），留出 `padding = LW_MOUTH/2` 的白边

可选增强（P1）：
- 当 `mouth_open > 0.65`，在内腔上部画一条很短的白线当“牙齿提示”。

---

## 5. FaceWidget 绘制规范（LVGL 9.4.0）

### 5.1 组件结构
- `face_widget`：一个 lv_obj，占满 466×466，背景黑（或在 draw 里先画黑底）

### 5.2 事件与刷新
- 注册 `LV_EVENT_DRAW_MAIN` 回调：
  - 读取 `face_anim_current()` 得到当前参数
  - 计算 breath 的 dy/scale
  - 依次绘制：背景 → 左眼 → 右眼 → 嘴 → icon（可选）
- 使用 `lv_timer_create(tick_cb, 33ms)`（目标 30fps）
  - tick 中调用 `face_anim_tick(dt)` 更新参数
  - `lv_obj_invalidate(face_widget)` 触发重绘

### 5.3 绘制顺序（必须）
1. 背景：黑色全屏（一次填充）
2. 眼睛（白胶囊/短线）+ 瞳孔点
3. 嘴（弧线或开口胶囊）
4. icon（可选）

---

## 6. 动画与插帧引擎（设备端“丝滑”）

### 6.1 二阶弹簧模型（推荐默认）
对每个参数维护：
- `x` 当前值
- `v` 速度
- `t` 目标值

更新（每 tick）：
- `v += (k*(t - x) - c*v) * dt`
- `x += v * dt`

推荐默认：
- `k = 60 ~ 90`
- `c = 14 ~ 20`
（眼睛可以更“Q弹”：k 高一点；嘴更稳：c 高一点）

### 6.2 多源融合与优先级（必须实现）
参数来源：
- `remote_target`（MQTT set/tween）
- `idle_layer`（呼吸/眨眼/扫视）
- `speech_layer`（音频包络→mouth_open）

融合规则（默认）：
- 若 `remote_active`（最近 500ms 内收到 MQTT set/tween）：
  - base = remote_target（除 breath_phase 仍由本机生成）
- 否则：
  - base = neutral + idle_layer

mouth_open 最终值：
- `mouth_open = max(base.mouth_open, speech_layer.mouth_open)`

眨眼（trigger blink）覆盖规则：
- blink 期间直接覆盖 `eye_open` 为 blink 曲线（见 6.3）

### 6.3 Idle 生成器（P0）
- 呼吸：`breath_phase += dt / 3.8s`（周期约 3.8 秒）
- 扫视：每 1.2~2.6s 随机生成一个 gaze 目标（小幅），用弹簧逼近
  - `gaze_x_target ∈ [-0.35, +0.35]`
  - `gaze_y_target ∈ [-0.20, +0.20]`
- 眨眼：每 2.0~6.0s 触发一次 blink 曲线
  - 闭合 60ms → 保持 30ms → 打开 90ms（可调）

blink 曲线（建议）：
- `eye_open *= (1 - blink_weight(t))`
- blink_weight 可用分段 smoothstep 实现

### 6.4 Speaking 联动（P0）
- 播放 PCM 时计算能量包络（RMS 或绝对值平均）
- `env = clamp(gain * rms, 0..1)`
- `speech_layer.mouth_open = smooth(env)`（再过一层低通/弹簧，避免抖）

---

## 7. MQTT 协议（P0）

### 7.1 Topics（建议）
- `face/cmd/set`：设置目标参数（推荐）
- `face/cmd/trigger`：触发动作（blink/surprise）
- `face/cmd/mode`：切模式（idle/remote/speaking）
- `face/state`：状态上报（当前参数、fps、heap、ack）

### 7.2 Payload（JSON）

#### set（参数目标 + 推荐过渡）
```json
{
  "id": "a81f",
  "t_ms": 220,
  "ease": "spring",
  "p": {
    "eye_open": 0.85,
    "gaze_x": 0.2,
    "gaze_y": -0.1,
    "mouth_curve": 0.6,
    "mouth_open": 0.0,
    "eye_squint": 0.1
  }
}

#### trigger（短动作）

```json
{ "id":"b14c", "act":"blink", "n":1 }
```

#### state（回传）

```json
{
  "ack_id":"a81f",
  "ok": true,
  "mode":"remote",
  "fps": 30,
  "heap": 123456,
  "p": {
    "eye_open": 0.90,
    "gaze_x": 0.05,
    "gaze_y": 0.00,
    "mouth_open": 0.10,
    "mouth_curve": 0.20
  }
}
```

### 7.3 频率建议

* set：手动滑杆 15~30Hz 均可（设备端会平滑）
* state：1~5Hz（telemetry 低频即可）

---

## 8. 代码模块与接口（AI 必须按此落地）

### 8.1 目录

```
components/face_ui/
  include/
    face_widget.h
    face_params.h
    face_anim.h
    face_mqtt.h
    face_audio.h
  src/
    face_widget.c
    face_params.c
    face_anim.c
    face_mqtt.c
    face_audio.c
    face_presets.c   (可选)
main/
  app_main.c
```

### 8.2 关键接口（必须）

#### face_params.h

* `typedef struct { float eye_open, eye_squint, gaze_x, gaze_y, mouth_open, mouth_curve; int icon; float icon_alpha; } face_params_t;`
* `void face_params_clamp(face_params_t* p);`

#### face_anim.h

* `void face_anim_init(void);`
* `void face_anim_set_target(const face_params_t* target, uint32_t t_ms, bool override);`
* `void face_anim_trigger_blink(int n);`
* `void face_anim_tick(float dt_sec);`
* `const face_params_t* face_anim_current(void);`

#### face_widget.h

* `lv_obj_t* face_widget_create(lv_obj_t* parent);`
* `void face_widget_bind_source(lv_obj_t* face, const face_params_t* (*get_cur)(void));`

#### face_mqtt.h

* `void face_mqtt_start(void);`
* `void face_mqtt_handle_message(const char* topic, const uint8_t* payload, size_t len);`

#### face_audio.h

* `void face_audio_on_pcm(const int16_t* pcm, size_t n, int sample_rate);`

---

## 9. 验收用例（P0）

1. Idle（无 MQTT）

* 运行 60s：呼吸 + 随机眨眼 + 轻微扫视持续；不卡顿；fps ≥ 20

2. MQTT gaze 连续控制

* 上位机每 50ms 发送 `gaze_x` 从 -1 → +1
* 眼神移动连续，不抖动、不抽动

3. Speaking 联动

* 播放一段 TTS：mouth_open 随音量开合；结束后回落到闭口弧线表情

4. 断连兜底

* MQTT 断开：系统自动回到 idle（不崩溃）；恢复后可继续 remote 控制

---

## 10. 默认“表情模板”（可选，建议 P1）

> 模板不是固定动画，只是“参数初值组合”。

* neutral：eye_open=0.92, mouth_curve=+0.15
* happy：eye_squint=0.35, mouth_curve=+0.85
* sad：eye_open=0.65, mouth_curve=-0.65, gaze_y=+0.15
* angry：eye_open=0.80, mouth_curve=-0.25, gaze_x=0, icon=anger
* surprised：eye_open=1.0, mouth_open=0.75, mouth_curve=0.0

---

## 11. 实现注意事项（AI 必须遵守）

* FaceWidget 必须是单对象自绘（不可创建大量 lv_img 叠加）
* 禁止常态全屏 lv_canvas 缓冲（内存开销过大）
* LVGL API（创建对象、invalidate）只能在 UI 线程调用
* MQTT 回调线程只解析/入队，不直接操作 LVGL
* 所有参数输入必须 clamp + 容错（未知字段忽略）

---
