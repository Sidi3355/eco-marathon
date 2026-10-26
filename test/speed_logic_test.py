"""Bench model of the pulse-period speed calculation and the pace logic.
Run before flashing to sanity-check config values: python3 test/speed_logic_test.py"""

WHEEL_CIRC_M, MAGNETS = 1.60, 4
LAP_M, LAPS, LIMIT_S = 1500, 10, 36 * 60
MIN_PERIOD_US = 15000


def speed_kmh(period_us):
    return (WHEEL_CIRC_M / MAGNETS) / (period_us * 1e-6) * 3.6


def pulse_hz(kmh):
    return (kmh / 3.6) / (WHEEL_CIRC_M / MAGNETS)


# 1. Decoding is exact and the bounce filter does not clip real speeds
for kmh in (5, 15, 25, 35, 60):
    period = 1e6 / pulse_hz(kmh)
    assert abs(speed_kmh(period) - kmh) < 1e-9
    assert period > MIN_PERIOD_US, f"{kmh} km/h would be rejected as bounce"
    print(f"{kmh:>3} km/h -> {pulse_hz(kmh):6.2f} Hz on the signal generator, period {period:8.0f} us")

# 2. Resolution: one microsecond of timer jitter at 35 km/h
p = 1e6 / pulse_hz(35)
print(f"resolution at 35 km/h: {speed_kmh(p) - speed_kmh(p + 1):.5f} km/h per us")

# 3. Pace logic: projected finish against the limit
def margin(elapsed_s, dist_m):
    avg = dist_m / elapsed_s
    return LIMIT_S - (elapsed_s + (LAPS * LAP_M - dist_m) / avg)

min_avg = LAPS * LAP_M / 1000 / (LIMIT_S / 3600)
print(f"minimum average {min_avg:.2f} km/h")
assert abs(margin(600, 600 * min_avg / 3.6)) < 1e-6       # exactly on the limit
assert margin(600, 600 * 26 / 3.6) > 60                    # 26 km/h: coast
assert margin(600, 600 * 24 / 3.6) < 0                     # 24 km/h: burn
print("all checks passed")
