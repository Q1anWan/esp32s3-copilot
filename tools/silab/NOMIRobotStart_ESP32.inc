# Legacy NOMI-like direct SILAB audio trigger example.
#
# Current production tests should prefer the Python logic decision program,
# because it can send string SCENE and string SEQ values directly:
#   TRIGGER;TIMESTAMP;SCENE;SEQ;SPEED<LF>
#
# This INC is kept only for SILAB-only numeric constant experiments.
#
# SILAB sends one text line per fixed-rate sample:
#   TRIGGER;TIME_LOW;TIME_HIGH;SCENE;SEQ;SPEED<LF>
#
# Field definition:
# - TRIGGER:   1 while active, 0 while idle.
# - TIME_LOW:  Low 6 digits of Unix time.
# - TIME_HIGH: floor(Unix time / 100000). The bridge uses integer/floor.
# - SCENE:     Legacy numeric audio scene ID.
# - SEQ:       Legacy numeric audio sequence ID.
# - SPEED:     VDyn.v_kmh, used by the experimenter for observation/debug.
#
# The bridge reconstructs:
#   unix_time = TIME_HIGH * 100000 + (TIME_LOW % 100000)
#
# The bridge forwards accepted trigger rising edges to ESP32.
# The bridge forwards every valid sample to HRT as:
#   unix_time,trigger,scene,seq,speed;
#
# Before running:
# - Copy NOMITimeBase.inc next to this file or into SILAB's system include path.
# - Update NOMITimeBase.inc before the experiment day.
# - Change Destination_IP to the experiment PC Ethernet LAN A IP.
# - Keep Destination_Port aligned with the bridge GUI SILAB TCP port.

include system\NOMITimeBase.inc

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
# constant signals to DataPort4 and DataPort5.
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

    UsedDataPorts = 6;
    Version = 3.0;
    TCPMode = 1;

    # Enable if the receiver gets too much traffic during debugging.
    # PeriodFactor = 4;
};

Connections =
{
    ~SystemTime.unixtime -> NOMITimeLow6.In,
    ~SystemTime.unixtime -> NOMITimeHigh100000.In,
    VDyn.v_kmh -> NOMIStartTrigger.In,

    # Field 1: trigger flag.
    NOMIStartTrigger.Greater -> NOMIAudioSender.DataPort1,

    # Field 2: low 6 digits of Unix timestamp.
    NOMITimeLow6.Out -> NOMIAudioSender.DataPort2,

    # Field 3: Unix timestamp / 100000. The bridge floors this value.
    NOMITimeHigh100000.Out -> NOMIAudioSender.DataPort3,

    # Field 4: legacy numeric scene ID.
    AudioSceneId.Out -> NOMIAudioSender.DataPort4,

    # Field 5: legacy numeric sequence ID.
    AudioSeqId.Out -> NOMIAudioSender.DataPort5,

    # Field 6: speed in km/h for observation/status/debug.
    VDyn.v_kmh -> NOMIAudioSender.DataPort6
};
