import math

SPEED_OF_LIGHT_M_S = 299_792_458.0


def range_to_rtt_ns(distance_m, turnaround_delay_ns) -> float:
    if not math.isfinite(distance_m) or distance_m < 0.0:
        raise ValueError("Distance must be finite and non-negative")
    if not math.isfinite(turnaround_delay_ns) or turnaround_delay_ns < 0.0:
        raise ValueError("Turnaround delay must be finite and non-negative")

    propagation_time_s = 2.0 * distance_m / SPEED_OF_LIGHT_M_S
    propagation_time_ns = propagation_time_s * 1e9

    return propagation_time_ns + turnaround_delay_ns

def rtt_to_range_m(rtt_ns:float, calibrated_delay_ns:float) -> float:
    """Recover distance using the calibrated station turnaround."""
    if not math.isfinite(rtt_ns) or not math.isfinite(calibrated_delay_ns):
        raise ValueError("RTT and calibrated delay must be finite")

    if calibrated_delay_ns < 0.0 or rtt_ns < calibrated_delay_ns:
        raise ValueError("RTT must be at least the non-negative calibrated delay")

    propagation_time_s = (rtt_ns - calibrated_delay_ns) * 1e-9

    return 0.5 * SPEED_OF_LIGHT_M_S * propagation_time_s