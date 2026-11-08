from dataclasses import dataclass, replace
import math
import numpy as np

SPEED_OF_LIGHT_M_S = 299_792_458.0
NS_TO_RANGE_M = SPEED_OF_LIGHT_M_S * 1e-9 / 2.0

def _finite_positive(value, name, allow_zero=False):
    if not math.isfinite(value) or (value < 0.0 if allow_zero else value <= 0):
        raise ValueError(f'{name} must be finite and {">=" if allow_zero else ">"} 0')

def validate_stations(stations):
    if len(stations) < 3:
        raise ValueError('at least three stations are required')
    result = {}
    for station_id, position in stations.items():
        point = np.asarray(position, dtype=float)
        if int(station_id) < 1 or point.shape != (3,) or not np.all(np.isfinite(point)):
            raise ValueError('station IDs must be positive and positions finite ENU triples')
        result[int(station_id)] = point.copy()
    xy = np.array(list(result.values()))[:, :2]
    if np.linalg.matrix_rank(xy[1:] - xy[0], tol=1e-7) < 2:
        raise ValueError('station horizontal positions must not be collinear')
    return result

@dataclass(frozen=True)
class SensorConfig:
    seed: int = 42
    rtt_noise_std_ns: float = 1.0
    turnaround_delay_ns: float = 100_000.0
    calibrated_turnaround_delay_ns: float = 100_000.0
    delivery_latency_s: float = 0.04
    delivery_jitter_s: float = 0.01
    loss_probability: float = 0.02
    baro_noise_std_m: float = 0.5

    def __post_init__(self):
        if self.seed < 0:
            raise ValueError('seed must be nonnegative')
        for name in ('rtt_noise_std_ns', 'turnaround_delay_ns',
                        'calibrated_turnaround_delay_ns', 'delivery_latency_s',
                        'delivery_jitter_s', 'baro_noise_std_m'):
            _finite_positive(getattr(self, name), name, allow_zero=True)
        if not math.isfinite(self.loss_probability) or not 0 <= self.loss_probability <= 1:
            raise ValueError('loss_probability must be in [0, 1]')

@dataclass(frozen=True)
class RangeObservation:
    stamp: float
    station_id: int
    sequence: int
    rtt_ns: float
    turnaround_delay_ns: float
    range_m: float
    variance_m2: float
    valid: bool = True

@dataclass(frozen=True)
class SimulatedRange:
    vehicle_id: int
    observation: RangeObservation
    delivery_stamp: float

@dataclass(frozen=True)
class HeightObservation:
    stamp: float
    height_m: float
    variance_m2: float
    valid: bool = True

class RttSimulator:
    def __init__(self, config=SensorConfig()):
            self.config = config
            self._noise_rng = {}
            self._delivery_rng = {}
            self._baro_rng = {}
            self._sequences = {}
            self.counters = {}
    
    def set_config(self, **updates):
        new_config = replace(self.config, **updates)
        if new_config.seed != self.config.seed:
            raise ValueError('seed cannot change without restarting the sensor')
        self.config = new_config

    def _stream(self, collection, vehicle_id, station_id, purpose):
        key = (vehicle_id, station_id)
        if key not in collection:
            collection[key] = np.random.default_rng(
                np.random.SeedSequence([self.config.seed, vehicle_id, station_id, purpose]))
        return collection[key]

    def sample_range(self, vehicle_id, station_id, position, station, stamp, disabled=False):
        point, anchor = np.asarray(position, dtype=float), np.asarray(station, dtype=float)
        if (point.shape != (3,) or anchor.shape != (3,) or
                not np.all(np.isfinite(point)) or not np.all(np.isfinite(anchor)) or
                not math.isfinite(stamp) or stamp < 0):
            raise ValueError('sensor positions and measurement timestamp must be finite')
        key = (int(vehicle_id), int(station_id))
        sequence = self._sequences.get(key, 0)
        self._sequences[key] = sequence + 1
        stats = self.counters.setdefault(key, {'generated': 0, 'dropped': 0, 'delivered': 0})
        stats['generated'] += 1
        noise_rng = self._stream(self._noise_rng, *key, 1)
        delivery_rng = self._stream(self._delivery_rng, *key, 2)
        distance = float(np.linalg.norm(point - anchor))
        noise_ns = float(noise_rng.normal(0.0, self.config.rtt_noise_std_ns))
        rtt_ns = distance / NS_TO_RANGE_M + self.config.turnaround_delay_ns + noise_ns
        calibrated_delay = self.config.calibrated_turnaround_delay_ns
        measured_range = (rtt_ns - calibrated_delay) * NS_TO_RANGE_M
        # Loss, noise, and jitter are sampled even for disabled links, keeping
        # the remaining streams reproducible when a fault is introduced.
        dropped = bool(delivery_rng.random() < self.config.loss_probability)
        latency = max(0.0, self.config.delivery_latency_s + float(
            delivery_rng.normal(0.0, self.config.delivery_jitter_s)))
        if disabled or dropped:
            stats['dropped'] += 1
            return None
        variance = max((self.config.rtt_noise_std_ns * NS_TO_RANGE_M) ** 2, 1e-8)
        observation = RangeObservation(
            stamp=float(stamp), station_id=key[1], sequence=sequence,
            rtt_ns=rtt_ns, turnaround_delay_ns=calibrated_delay,
            range_m=measured_range, variance_m2=variance,
            valid=math.isfinite(measured_range) and measured_range >= 0.0)
        return SimulatedRange(key[0], observation, stamp + latency)

    def mark_delivered(self, vehicle_id, station_id):
        self.counters[(vehicle_id, station_id)]['delivered'] += 1

    def sample_height(self, vehicle_id, height_m, stamp):
        if not math.isfinite(height_m) or not math.isfinite(stamp) or stamp < 0:
            raise ValueError('height and timestamp must be finite')
        rng = self._stream(self._baro_rng, int(vehicle_id), 0, 3)
        return HeightObservation(
            stamp, height_m + float(rng.normal(0, self.config.baro_noise_std_m)),
            max(self.config.baro_noise_std_m ** 2, 1e-8))
