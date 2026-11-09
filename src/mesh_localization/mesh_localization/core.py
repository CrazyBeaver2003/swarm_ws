import math
import numpy as np
from typing import Mapping
from collections import deque
from dataclasses import dataclass, field, replace

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

@dataclass(frozen=True)
class PositionFix:
    stamp: float
    position: np.ndarray
    covariance: np.ndarray
    station_count: int
    residual_chi2: float
    geometry_condition: float


def solve_position(
    stations: Mapping[int, np.ndarray],
    ranges: Mapping[int, RangeObservation],
    baro: HeightObservation,
    residual_gate=16.0,
    max_geometry_condition=1000.0,
    max_horizontal_std_m=50.0,
):
    """Calculate XY from slant ranges, using barometric height for Z.

    Inputs must contain valid observations from known stations,
    referring to one measurement epoch.
    """
    if len(ranges) < 3 or not baro.valid or not math.isfinite(baro.height_m):
        raise ValueError(
            'three ranges and a valid barometer measurement are required'
        )
    _finite_positive(baro.variance_m2, 'barometer variance')
    ids = sorted(ranges)
    anchors = np.array([stations[key] for key in ids], dtype=float)
    distances = np.array([ranges[key].range_m for key in ids], dtype=float)
    variances = np.array([ranges[key].variance_m2 for key in ids], dtype=float)

    if (
        not np.all(np.isfinite(distances))
        or np.any(distances < 0)
        or not np.all(np.isfinite(variances))
        or np.any(variances <= 0)
    ):
        raise ValueError('invalid range values or variances')

    dz = baro.height_m - anchors[:, 2]

    # Начальная оценка XY из разностей квадратов дальностей.
    matrix = 2.0 * (anchors[1:, :2] - anchors[0, :2])

    rhs = (
        np.sum(anchors[1:, :2] ** 2, axis=1)
        - np.sum(anchors[0, :2] ** 2)
        + dz[1:] ** 2
        - dz[0] ** 2
        - distances[1:] ** 2
        + distances[0] ** 2
    )

    if np.linalg.matrix_rank(matrix, tol=1e-7) < 2:
        raise ValueError('collinear station geometry')

    xy = np.linalg.lstsq(matrix, rhs, rcond=None)[0]

    # Уточнение методом Гаусса — Ньютона.
    for _ in range(12):
        delta_xy = xy - anchors[:, :2]
        predicted = np.sqrt(np.sum(delta_xy ** 2, axis=1) + dz ** 2)

        if np.any(predicted < 1e-6):
            raise ValueError('range derivative undefined at a station')

        jacobian = delta_xy / predicted[:, None]
        vertical_jacobian = dz / predicted

        # Ошибка баровысоты общая для всех вычисленных дальностей.
        measurement_covariance = (
            np.diag(variances)
            + baro.variance_m2
            * np.outer(vertical_jacobian, vertical_jacobian)
        )

        weighted_jacobian = np.linalg.solve(
            measurement_covariance, jacobian
        )
        normal = jacobian.T @ weighted_jacobian
        condition = float(np.linalg.cond(normal))

        if not math.isfinite(condition) or condition > max_geometry_condition:
            raise ValueError('poor station geometry')

        step = np.linalg.solve(
            normal,
            jacobian.T @ np.linalg.solve(
                measurement_covariance, predicted - distances
            ),
        )
        xy -= step

        if float(np.linalg.norm(step)) < 1e-7:
            break
    else:
        raise ValueError('position solver did not converge')

    # Пересчитываем показатели в окончательной точке.
    delta_xy = xy - anchors[:, :2]
    predicted = np.sqrt(np.sum(delta_xy ** 2, axis=1) + dz ** 2)

    if np.any(predicted < 1e-6):
        raise ValueError('range derivative undefined at a station')

    jacobian = delta_xy / predicted[:, None]
    vertical_jacobian = dz / predicted

    measurement_covariance = (
        np.diag(variances)
        + baro.variance_m2
        * np.outer(vertical_jacobian, vertical_jacobian)
    )

    weighted_jacobian = np.linalg.solve(measurement_covariance, jacobian)
    normal = jacobian.T @ weighted_jacobian
    condition = float(np.linalg.cond(normal))

    if not math.isfinite(condition) or condition > max_geometry_condition:
        raise ValueError('poor station geometry')

    covariance_xy = np.linalg.inv(normal)
    residual = predicted - distances
    chi2 = float(
        residual @ np.linalg.solve(measurement_covariance, residual)
    )

    if (
        not math.isfinite(chi2)
        or chi2 > residual_gate * max(1, len(ids) - 2)
    ):
        raise ValueError('inconsistent range residuals')

    horizontal_std = math.sqrt(
        float(np.linalg.eigvalsh(covariance_xy)[-1])
    )
    if horizontal_std > max_horizontal_std_m:
        raise ValueError('horizontal uncertainty exceeds limit')

    derivative_xy_z = (
        -covariance_xy
        @ jacobian.T
        @ np.linalg.solve(measurement_covariance, vertical_jacobian)
    )

    covariance = np.zeros((3, 3))
    covariance[:2, :2] = covariance_xy
    covariance[:2, 2] = derivative_xy_z * baro.variance_m2
    covariance[2, :2] = covariance[:2, 2]
    covariance[2, 2] = baro.variance_m2

    return PositionFix(
        stamp=next(iter(ranges.values())).stamp,
        position=np.array([xy[0], xy[1], baro.height_m]),
        covariance=0.5 * (covariance + covariance.T),
        station_count=len(ids),
        residual_chi2=chi2,
        geometry_condition=condition,
    )


class ConstantVelocityFilter:
    """Estimate [x, y, z, vx, vy, vz] from position fixes."""

    def __init__(self, acceleration_noise_density=4.0, innovation_gate=25.0):
        _finite_positive(
            acceleration_noise_density, 'acceleration noise density'
        )
        _finite_positive(innovation_gate, 'innovation gate')

        self.acceleration_noise_density = acceleration_noise_density
        self.innovation_gate = innovation_gate

        self.state = None
        self.covariance = None
        self.stamp = None
        self.update_count = 0

    def _predict(self, stamp):
        if stamp < self.stamp:
            raise ValueError(
                'prediction time precedes the last accepted fix'
            )

        dt = stamp - self.stamp

        # Модель движения с постоянной скоростью.
        transition = np.eye(6)
        transition[:3, 3:] = np.eye(3) * dt

        # Неопределённость из-за неучтённых ускорений.
        q = self.acceleration_noise_density
        process = np.zeros((6, 6))
        process[:3, :3] = np.eye(3) * q * dt ** 3 / 3.0
        process[:3, 3:] = np.eye(3) * q * dt ** 2 / 2.0
        process[3:, :3] = process[:3, 3:].T
        process[3:, 3:] = np.eye(3) * q * dt

        return (
            transition @ self.state,
            transition @ self.covariance @ transition.T + process,
        )

    def update(self, fix):
        _finite_positive(
            fix.stamp, 'measurement timestamp', allow_zero=True
        )

        position = np.asarray(fix.position, dtype=float)
        measurement_covariance = np.asarray(
            fix.covariance, dtype=float
        )

        if (
            position.shape != (3,)
            or measurement_covariance.shape != (3, 3)
            or not np.all(np.isfinite(position))
            or not np.all(np.isfinite(measurement_covariance))
        ):
            raise ValueError('invalid position fix')

        if (
            not np.allclose(
                measurement_covariance,
                measurement_covariance.T,
                rtol=1e-10,
                atol=1e-12,
            )
            or np.min(np.linalg.eigvalsh(measurement_covariance)) <= 0
        ):
            raise ValueError(
                'position covariance must be symmetric positive definite'
            )

        # Первый принятый замер задаёт начальное состояние.
        if self.state is None:
            self.state = np.r_[position, np.zeros(3)]

            self.covariance = np.zeros((6, 6))
            self.covariance[:3, :3] = measurement_covariance
            self.covariance[3:, 3:] = np.eye(3) * 225.0

            self.stamp = fix.stamp
            self.update_count = 1
            return True

        # Повторные и более старые оценки не принимаем.
        if fix.stamp <= self.stamp + 1e-9:
            return False

        predicted, predicted_covariance = self._predict(fix.stamp)

        innovation = position - predicted[:3]
        innovation_covariance = (
            predicted_covariance[:3, :3] + measurement_covariance
        )

        mahalanobis = float(
            innovation
            @ np.linalg.solve(innovation_covariance, innovation)
        )

        # Отбраковка слишком большого скачка положения.
        if (
            not math.isfinite(mahalanobis)
            or mahalanobis > self.innovation_gate
        ):
            return False

        gain = np.linalg.solve(
            innovation_covariance,
            predicted_covariance[:, :3].T,
        ).T

        corrected = predicted + gain @ innovation

        # Обновление ковариации в форме Джозефа.
        residual_matrix = np.eye(6)
        residual_matrix[:, :3] -= gain

        covariance = (
            residual_matrix
            @ predicted_covariance
            @ residual_matrix.T
            + gain @ measurement_covariance @ gain.T
        )

        self.state = corrected
        self.covariance = 0.5 * (covariance + covariance.T)
        self.stamp = fix.stamp
        self.update_count += 1
        return True

    def predict(self, stamp):
        _finite_positive(
            stamp, 'prediction timestamp', allow_zero=True
        )

        if self.state is None:
            return None, None

        return self._predict(stamp)

@dataclass(frozen=True)
class LocalizerConfig:
    stale_timeout_s: float = 0.8
    degraded_after_s: float = 0.25
    max_measurement_age_s: float = 1.0
    baro_sync_tolerance_s: float = 0.08
    future_tolerance_s: float = 0.02
    reinitialize_after_s: float = 3.0
    acceleration_noise_density: float = 4.0
    innovation_gate: float = 25.0
    residual_gate: float = 16.0
    max_geometry_condition: float = 1000.0
    max_horizontal_std_m: float = 50.0

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            _finite_positive(getattr(self, name), name)
        if self.degraded_after_s >= self.stale_timeout_s:
            raise ValueError('degraded_after_s must be smaller than stale_timeout_s')
        if self.reinitialize_after_s <= self.stale_timeout_s:
            raise ValueError('reinitialize_after_s must exceed stale_timeout_s')


@dataclass(frozen=True)
class Estimate:
    stamp: float
    position: np.ndarray = field(default_factory=lambda: np.full(3, np.nan))
    velocity: np.ndarray = field(default_factory=lambda: np.full(3, np.nan))
    position_covariance: np.ndarray = field(
        default_factory=lambda: np.full((3, 3), np.inf)
    )
    velocity_covariance: np.ndarray = field(
        default_factory=lambda: np.full((3, 3), np.inf)
    )
    status: int = 0
    quality: float = 0.0
    used_station_count: int = 0
    measurement_age_s: float = math.inf
    reset_counter: int = 0


class MeshLocalizer:
    """Collect measurement epochs, update the filter, and predict to now."""

    INVALID, VALID, DEGRADED = 0, 1, 2

    def __init__(self, stations, config=LocalizerConfig()):
        self.stations = validate_stations(stations)
        self.config = config
        self.filter = self._new_filter()
        self._pending = {}
        self._baro = deque(maxlen=200)
        self._seen = set()
        self._seen_order = deque()
        self._last_now = None
        self.reset_counter = 0
        self.used_station_count = 0
        self.rejections = 0
        self.last_rejection = ''

    def _new_filter(self):
        return ConstantVelocityFilter(
            self.config.acceleration_noise_density,
            self.config.innovation_gate,
        )

    def reset(self):
        self.filter = self._new_filter()
        self._pending.clear()
        self._baro.clear()
        self._seen.clear()
        self._seen_order.clear()
        self._last_now = None
        self.used_station_count = 0
        self.last_rejection = ''
        self.reset_counter = (self.reset_counter + 1) % (2 ** 32)

    def _check_clock(self, now):
        _finite_positive(now, 'clock', allow_zero=True)
        if self._last_now is not None and now < self._last_now:
            self.reset()
        self._last_now = now

    def _valid_stamp(self, stamp, now):
        return (
            math.isfinite(stamp)
            and stamp >= 0
            and -self.config.future_tolerance_s <= now - stamp
            <= self.config.max_measurement_age_s
        )

    def add_range(self, observation, now):
        self._check_clock(now)
        if (
            not observation.valid
            or type(observation.station_id) is not int
            or observation.station_id not in self.stations
            or type(observation.sequence) is not int
            or observation.sequence < 0
            or not self._valid_stamp(observation.stamp, now)
            or not math.isfinite(observation.rtt_ns)
            or observation.rtt_ns < 0
            or not math.isfinite(observation.turnaround_delay_ns)
            or observation.turnaround_delay_ns < 0
            or not math.isfinite(observation.range_m)
            or observation.range_m < 0
            or not math.isfinite(observation.variance_m2)
            or observation.variance_m2 <= 0
        ):
            return False

        inferred = (
            observation.rtt_ns - observation.turnaround_delay_ns
        ) * NS_TO_RANGE_M
        if inferred < 0 or abs(inferred - observation.range_m) > max(
            1e-6, abs(inferred) * 1e-9
        ):
            return False

        epoch = round(observation.stamp * 1e9)
        stamp = epoch / 1_000_000_000
        if self.filter.stamp is not None and stamp <= self.filter.stamp + 1e-9:
            return False

        duplicate = (observation.station_id, observation.sequence)
        if duplicate in self._seen:
            return False
        if observation.station_id in self._pending.get(epoch, {}):
            return False

        self._seen.add(duplicate)
        self._seen_order.append(duplicate)
        if len(self._seen_order) > 4096:
            self._seen.remove(self._seen_order.popleft())

        self._pending.setdefault(epoch, {})[observation.station_id] = observation
        if len(self._pending) > 200:
            del self._pending[min(self._pending)]
        return True

    def add_height(self, observation, now):
        self._check_clock(now)
        if (
            not observation.valid
            or not self._valid_stamp(observation.stamp, now)
            or not math.isfinite(observation.height_m)
            or not math.isfinite(observation.variance_m2)
            or observation.variance_m2 <= 0
        ):
            return False
        self._baro.append(observation)
        return True

    def process(self, now):
        self._check_clock(now)
        self._baro = deque(
            (item for item in self._baro if self._valid_stamp(item.stamp, now)),
            maxlen=200,
        )

        for epoch in sorted(list(self._pending)):
            stamp = epoch / 1_000_000_000
            ranges = self._pending[epoch]

            if (
                now - stamp > self.config.max_measurement_age_s
                or (
                    self.filter.stamp is not None
                    and stamp <= self.filter.stamp + 1e-9
                )
            ):
                del self._pending[epoch]
                continue

            # Future packets can wait in the buffer, but cannot be fused yet.
            if stamp > now:
                break
            if len(ranges) < 3:
                continue

            candidates = [
                item for item in self._baro
                if item.stamp <= now
                and abs(item.stamp - stamp) <= self.config.baro_sync_tolerance_s
            ]
            if not candidates:
                continue
            baro = min(candidates, key=lambda item: abs(item.stamp - stamp))
            del self._pending[epoch]

            try:
                fix = solve_position(
                    self.stations,
                    ranges,
                    baro,
                    self.config.residual_gate,
                    self.config.max_geometry_condition,
                    self.config.max_horizontal_std_m,
                )
                fix = replace(fix, stamp=stamp)
                restart = (
                    self.filter.stamp is not None
                    and stamp - self.filter.stamp > self.config.reinitialize_after_s
                )
                candidate = self._new_filter() if restart else self.filter
                if not candidate.update(fix):
                    raise ValueError('position innovation rejected')

                self.filter = candidate
                if restart:
                    self.reset_counter = (self.reset_counter + 1) % (2 ** 32)
                self.used_station_count = fix.station_count
                self.last_rejection = ''
            except (ValueError, np.linalg.LinAlgError, FloatingPointError) as error:
                self.rejections += 1
                self.last_rejection = str(error)

    def estimate(self, now):
        self.process(now)
        if self.filter.stamp is None:
            return Estimate(stamp=now, reset_counter=self.reset_counter)

        age = now - self.filter.stamp
        state, covariance = self.filter.predict(now)
        horizontal_std = math.sqrt(max(
            0.0, float(np.linalg.eigvalsh(covariance[:2, :2])[-1])
        ))

        if (
            age > self.config.stale_timeout_s
            or horizontal_std > self.config.max_horizontal_std_m
        ):
            status = self.INVALID
        elif age > self.config.degraded_after_s or self.filter.update_count < 3:
            status = self.DEGRADED
        else:
            status = self.VALID

        quality = math.exp(-age / self.config.stale_timeout_s) / (
            1.0 + horizontal_std ** 2 / 25.0
        )
        if self.filter.update_count < 3:
            quality *= 0.5
        if status == self.INVALID:
            quality = 0.0

        return Estimate(
            stamp=now,
            position=state[:3],
            velocity=state[3:],
            position_covariance=covariance[:3, :3],
            velocity_covariance=covariance[3:, 3:],
            status=status,
            quality=float(quality),
            used_station_count=self.used_station_count,
            measurement_age_s=age,
            reset_counter=self.reset_counter,
        )
