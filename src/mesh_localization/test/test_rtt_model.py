import math

import numpy as np
import pytest

from mesh_localization.core import NS_TO_RANGE_M, RttSimulator, SensorConfig


def test_round_trip_units_and_calibration():
    simulator = RttSimulator(SensorConfig(
        rtt_noise_std_ns=0, loss_probability=0, delivery_jitter_s=0))
    packet = simulator.sample_range(1, 2, [100, 200, 50], [0, 0, 0], 1.5)
    assert packet.observation.range_m == pytest.approx(math.sqrt(52_500), abs=1e-9)
    assert packet.observation.rtt_ns > 100_000
    assert packet.delivery_stamp == pytest.approx(1.54)
    biased = RttSimulator(SensorConfig(
        rtt_noise_std_ns=0, loss_probability=0,
        calibrated_turnaround_delay_ns=99_990.0))
    biased_packet = biased.sample_range(1, 2, [100, 200, 50], [0, 0, 0], 1.5)
    assert biased_packet.observation.range_m - packet.observation.range_m == pytest.approx(
        10 * NS_TO_RANGE_M)


def test_ns_noise_statistics_and_repeatability():
    config = SensorConfig(loss_probability=0, rtt_noise_std_ns=1.0)
    first, second = RttSimulator(config), RttSimulator(config)
    samples = np.array([
        first.sample_range(1, 2, [100, 0, 0], [0, 0, 0], index * 0.05).observation.range_m
        for index in range(5000)])
    repeats = np.array([
        second.sample_range(1, 2, [100, 0, 0], [0, 0, 0], index * 0.05).observation.range_m
        for index in range(5000)])
    np.testing.assert_array_equal(samples, repeats)
    assert abs(np.mean(samples) - 100) < 0.01
    assert np.std(samples) == pytest.approx(NS_TO_RANGE_M, rel=0.05)


def test_delivery_faults_do_not_change_measurement_noise():
    normal = RttSimulator(SensorConfig(loss_probability=0, delivery_jitter_s=0))
    faulty = RttSimulator(SensorConfig(loss_probability=0.5, delivery_jitter_s=0.08))
    saw_drop, saw_reordering = False, False
    delivery_stamps = []
    for index in range(100):
        reference = normal.sample_range(1, 1, [20, 30, 50], [0, 0, 0], index * 0.01)
        packet = faulty.sample_range(1, 1, [20, 30, 50], [0, 0, 0], index * 0.01)
        if packet is None:
            saw_drop = True
        else:
            assert packet.observation.sequence == index
            assert packet.observation.range_m == reference.observation.range_m
            delivery_stamps.append(packet.delivery_stamp)
    saw_reordering = any(b < a for a, b in zip(delivery_stamps, delivery_stamps[1:]))
    assert saw_drop and saw_reordering
    assert faulty.counters[(1, 1)]['generated'] == 100
    assert faulty.counters[(1, 1)]['dropped'] + len(delivery_stamps) == 100


def test_disabled_links_consume_sequences_and_count_losses():
    simulator = RttSimulator(SensorConfig(loss_probability=0))
    assert simulator.sample_range(1, 1, [1, 2, 3], [0, 0, 0], 1, disabled=True) is None
    packet = simulator.sample_range(1, 1, [1, 2, 3], [0, 0, 0], 2)
    simulator.mark_delivered(1, 1)
    assert packet.observation.sequence == 1
    assert simulator.counters[(1, 1)] == {'generated': 2, 'dropped': 1, 'delivered': 1}
