"""
    This script generates the fixed point (Q15) phase calibration values for
    interpolating CT samples between voltage samples.

    Currently, this generates values for a single phase fixed offset system.

    Usage: phasecal.py <MAINS_FREQ> <SAMPLE_RATE> <PHI_0> .. <PHI_N>
"""

import sys
import math

if len(sys.argv) < 4:
    print("Must provide mains frequency, sample rate, CT phase.")
    print("Usage: phasecal.py <MAINS_FREQ> <SAMPLE_RATE> <PHI_0> .. <PHI_N>")
    exit(1)

OVERSAMPLING = 2
FIXED_CONV = 1 << 15
phaseCT = []
mains_freq = 0
sample_rate_hz = 0
sample_rate_rad = 0

try:
    mains_freq = int(sys.argv[1])
except ValueError as e:
    print(f'{sys.argv[1]} cannot be converted to int: {e}')
    exit(1)

try:
    sample_rate_hz = int(sys.argv[2])
except ValueError as e:
    print(f'{sys.argv[2]} cannot be converted to int: {e}')
    exit(1)

num_ct = len(sys.argv) - 3
for idxCT in range(num_ct):
    try:
        phaseCT.append(float(sys.argv[idxCT + 3]))
    except ValueError as e:
        print(f'{sys.argv[idxCT + 3]} cannot be converted to float: {e}')
        exit(1)

# Timer precision is 1 us, round ADC time precision accordingly
t_adc = float(int(1 / ((num_ct + 1) * sample_rate_hz) * 1E6) / 1E6)
t_sample = t_adc * (num_ct + 1)
sample_rate_rad = t_sample * math.pi * 2 * mains_freq

print(f'Q15 Phase calibrations for {num_ct} CTs @ {sample_rate_hz} Hz')
print(f'ADC sample rate: {sample_rate_hz * OVERSAMPLING * (num_ct + 1)} Hz ({t_adc * 1E6 / OVERSAMPLING:.1f} us)\n')

for idxCT in range(num_ct):
    phase_shift = ((phaseCT[idxCT] / 360.0)
                   + (idxCT + 1) * t_adc * mains_freq * math.pi * 2)

    phaseY = (math.sin(phase_shift) / math.sin(sample_rate_rad))
    phaseX = (math.cos(phase_shift) - phaseY * math.cos(sample_rate_rad))

    phaseX = int(phaseX * FIXED_CONV)
    phaseY = int(phaseY * FIXED_CONV)

    print(f'CT{idxCT}:\tx: {phaseX}  \ty: {phaseY}')
