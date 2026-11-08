import math
from rcl_interfaces.msg import ParameterDescriptor
from .core import validate_stations

READ_ONLY = ParameterDescriptor(read_only=True)
DEFAULT_STATIONS = {
    1: [0.0, 0.0, 0.0],
    2: [500.0, 0.0, 0.0],
    3: [250.0, 433.0127018922193, 0.0],
}

def fixed_parameter(node, name, value):
    return node.declare_parameter(name, value, READ_ONLY).value

def station_parameters(node):
    station_ids = fixed_parameter(node, 'station_ids', [1, 2, 3])
    if (len(station_ids) < 3 or len(set(station_ids)) != len(station_ids) or
            any(not 1 <= item <= 2 ** 32 - 1 for item in station_ids)):
        raise ValueError('station_ids must contain at least 3 distinct positive uint32 IDs')
    stations = {}
    for station_id in station_ids:
        default = DEFAULT_STATIONS.get(station_id, [math.nan, math.nan, math.nan])
        stations[station_id] = fixed_parameter(
            node, f'station_{station_id}_position_enu_m', default)
    return validate_stations(stations)


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def set_stamp(stamp, seconds_value):
    nanoseconds = max(0, round(seconds_value * 1e9))
    stamp.sec, stamp.nanosec = divmod(nanoseconds, 1_000_000_000)


def require_positive(value, name):
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return value
