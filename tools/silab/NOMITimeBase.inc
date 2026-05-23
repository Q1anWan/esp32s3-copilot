# Auto-generated / experiment-day time base for NOMIRobotStart_ESP32.inc
#
# Legacy direct-SILAB numeric test frame format:
#   TRIGGER;TIME_LOW;TIME_HIGH;SCENE;SEQ;SPEED
#
# Production Python logic decision programs should send the simpler string
# frame instead:
#   TRIGGER;TIMESTAMP;SCENE;SEQ;SPEED
#
# TIME_LOW  = low 6 digits of Unix time.
# TIME_HIGH = Unix time / 100000. The bridge floors this value to an integer.
#
# Example for 2026-05-20 00:00:00 China time:
#   unix_time       = 1779206400
#   low6_base       = 1779000000
#   TIME_LOW offset = -1779000000
#
# The bridge reconstructs:
#   unix_time = floor(TIME_HIGH) * 100000 + (floor(TIME_LOW) % 100000)

DPUAmplifier NOMITimeLow6
{
    Computer = {OPERATOR};
    Index = 65;

    Multiplicator = 1;
    Divisor = 1;
    Offset = -1779000000;
};

DPUAmplifier NOMITimeHigh100000
{
    Computer = {OPERATOR};
    Index = 64;

    Multiplicator = 1;
    Divisor = 100000;
    Offset = 0;
};
