# Copilot MQTT Protocol

Topic layout (default):
- Command: `copilot/<device_id>/cmd`
- Status:  `copilot/<device_id>/status`

`device_id` comes from `CONFIG_COPILOT_DEVICE_ID`. If empty, a MAC-based ID is used.

## Commands (JSON)

### Expression
```json
{"type":"emotion","name":"happy","duration_ms":420,"prelight_ms":500,"sound":"chime"}
```
Fields:
- `name`: `neutral|happy|sad|angry|surprised|sleepy|dizzy`
- `id`: numeric expression index (0..6), optional alternative to `name`
- `duration_ms`: transition time (default `CONFIG_COPILOT_EXPR_TRANSITION_MS`)
- `prelight_ms`: ring prelight time (default `CONFIG_COPILOT_PRELIGHT_MS`)
- `sound`: optional sound id or `true` to use a default beep

### Motion
```json
{"type":"motion","ax":0.3,"ay":-0.2,"yaw":12.0,"speed":15.0}
```
Fields:
- `ax`: forward/back acceleration (g, roughly -1..1)
- `ay`: lateral acceleration (g, roughly -1..1)
- `yaw`: steering yaw (deg)
- `speed`: optional, reserved for future use

### Sound
```json
{"type":"sound","id":"beep_short","prelight_ms":350}
```
IDs: `beep_short|beep_long|chime|tap`

### Ring
```json
{"type":"ring","on":true}
```

