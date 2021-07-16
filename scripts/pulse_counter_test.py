#!/usr/bin/env python3

import warnings
import time
from .pulse_counter import PulseCounter

warnings.filterwarnings("ignore")


if __name__ == "__main__":
    desc = """Test Pulse Counter functionality"""

    # Create instance of PulseCounter
    pulsecounter = PulseCounter()

    # Clear the pulse counter
    pulsecounter.reset_count()

    for i in range(120):

        print(pulsecounter.read_16bit_counters(reset_after_read=True))
        time.sleep(1)

    SystemExit(0)
