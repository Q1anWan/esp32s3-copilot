#!/usr/bin/env python3
"""Generate NOMITimeBase.inc for the current experiment day.

Run this on the SILAB/experiment PC before a test day, then place the generated
NOMITimeBase.inc where NOMIRobotStart_ESP32.inc can include it.
"""

from __future__ import annotations

import argparse
import datetime as dt
from pathlib import Path


def make_content(base_unix: int) -> str:
    low6_base = (base_unix // 1_000_000) * 1_000_000
    return f"""# Auto-generated time base for NOMIRobotStart_ESP32.inc
#
# base_unix       = {base_unix}
# low6_base       = {low6_base}
# TIME_LOW offset = -{low6_base}

DPUAmplifier NOMITimeLow6
{{
    Computer = {{OPERATOR}};
    Index = 65;

    Multiplicator = 1;
    Divisor = 1;
    Offset = -{low6_base};
}};

DPUAmplifier NOMITimeHigh100000
{{
    Computer = {{OPERATOR}};
    Index = 64;

    Multiplicator = 1;
    Divisor = 100000;
    Offset = 0;
}};
"""


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate NOMITimeBase.inc for SILAB trigger/time_low/time_high frame.")
    parser.add_argument("--date", default="", help="Experiment date YYYY-MM-DD. Default: today in local timezone")
    parser.add_argument("--out", default="NOMITimeBase.inc", help="Output path. Default: NOMITimeBase.inc")
    args = parser.parse_args()

    if args.date:
        day = dt.date.fromisoformat(args.date)
    else:
        day = dt.datetime.now().astimezone().date()
    midnight = dt.datetime.combine(day, dt.time.min).astimezone()
    base_unix = int(midnight.timestamp())
    Path(args.out).write_text(make_content(base_unix), encoding="utf-8")
    print(f"Wrote {args.out}: date={day.isoformat()} base_unix={base_unix}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
