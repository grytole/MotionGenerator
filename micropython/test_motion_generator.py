import time
from motion_generator import MotionGenerator

MAX_VEL = 200
MAX_ACC = 500
INIT_POS = 0

NUM_TICKS = 100
NUM_STEPS = 5
TEST_PERIOD_S = 5
test_refs = [0, -25, 150, 100, 165]

mg = MotionGenerator(MAX_VEL, MAX_ACC, INIT_POS)
total_time_us = 0
ref = 0.0

for t in range(NUM_TICKS):
    ts = time.ticks_us()
    if t % (NUM_TICKS // NUM_STEPS) == 0 and len(test_refs) > 0:
        ref = test_refs.pop(0)

    pos = mg.update(ref)
    vel = mg.velocity()
    acc = mg.acceleration()

    total_time_us += time.ticks_diff(time.ticks_us(), ts)
    print("ref:{} pos:{} vel:{} acc:{}".format(ref, pos, vel, acc))
    time.sleep(TEST_PERIOD_S / NUM_TICKS)

total_time_s = total_time_us / 1_000_000
tick_time_s = total_time_s / NUM_TICKS
update_freq_hz = 1 / tick_time_s

print("Done:")
print(" {} iterations took {} s".format(NUM_TICKS, total_time_s))
print(" {} s per iteration".format(tick_time_s))
print(" {} Hz".format(round(update_freq_hz, 1)))
