# Agent Rules

## Scope

- Own ESP32-S3 firmware, device protocols, on-device UI, audio playback, MQTT integration, and hardware tests.
- Do not perform participant-data cleaning, statistical analysis, or thesis writing in this repository.
- Do not implement the authoritative voice-script or TTS generation workflow here. Consume a validated audio package and manifest from `D:\WorkSpace\copilot\silent_failure_tts`.

## Safety and secrets

- Never commit Wi-Fi credentials, MQTT credentials, API keys, access tokens, private certificates, or local configuration files.
- Keep generated builds, flash images, logs, and local SDK state out of Git unless a user explicitly requests a versioned release artifact.
- Preserve the deployed audio path contract: `/sdcard/audio/<scene>/<seq>.wav`.

## Cross-repository handoff

- A voice package must include its source-script hash, generation configuration, manifest, and per-WAV SHA-256 before deployment.
- Record firmware commit and audio-manifest hash when producing a deployment release.
- Experimental observations and participant-derived data go to `Experiments`, not this repository.
