# NOMI-like desktop robot audio trigger for Copilot PC bridge
#
# Interface contract:
# - SILAB acts as TCP client.
# - The experiment PC bridge GUI acts as TCP host/server.
# - The PC forwards accepted trigger events to ESP32 over MQTT.
# - Every packet is one UTF-8/ASCII text line:
#     trigger<TAB>scene<TAB>seq<LF>
#
# Fixed-rate sending is OK. ESP32 plays only on the rising edge:
# - trigger 0: idle/reset latch
# - trigger 1: play once
# - trigger 1 repeated: ignored until trigger returns to 0
#
# Numeric scene IDs map to TF-card folders:
# - scene=1, seq=1 -> /sdcard/audio/scene001/001.wav
# - scene=2, seq=5 -> /sdcard/audio/scene002/005.wav
#
# Before running:
# - Change Destination_IP to the experiment PC Ethernet LAN A IP.
# - Keep Destination_Port aligned with the bridge GUI TCP port.
# - Replace the AudioSceneId/AudioSeqId placeholder sources with constant
#   outputs in your SILAB project if the exact DPUConstant syntax differs.

DPUComparator NOMIStartTrigger
{
    Computer = {OPERATOR};
    Index = 60;

    # Vehicle is considered moving once speed is above 1 km/h.
    Min = 1;
    Max = 1;
};

# Reference constant blocks for the audio ID. If your SILAB version names
# constant blocks differently, keep the DataStreamSender and connect equivalent
# constant signals to DataPort2 and DataPort3.
DPUConstant AudioSceneId
{
    Computer = {OPERATOR};
    Index = 62;
    Value = 1;
};

DPUConstant AudioSeqId
{
    Computer = {OPERATOR};
    Index = 63;
    Value = 1;
};

DPUDataStreamSender NOMIAudioSender
{
    Computer = {OPERATOR};
    Index = 61;

    # Replace this with the experiment PC Ethernet LAN A IP.
    Destination_IP = "192.168.1.100";
    Destination_Port = 7777;

    UsedDataPorts = 3;
    Version = 3.0;
    TCPMode = 1;

    # Enable if the receiver gets too much traffic during debugging.
    # PeriodFactor = 4;
};

Connections =
{
    VDyn.v_kmh -> NOMIStartTrigger.In,

    # Field 1: trigger flag.
    NOMIStartTrigger.Greater -> NOMIAudioSender.DataPort1,

    # Field 2: scene ID. 1 means /sdcard/audio/scene001/.
    AudioSceneId.Out -> NOMIAudioSender.DataPort2,

    # Field 3: sequence ID. 1 means 001.wav.
    AudioSeqId.Out -> NOMIAudioSender.DataPort3
};
