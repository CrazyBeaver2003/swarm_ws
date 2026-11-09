from dataclasses import dataclass
import math


def clamp(value, low, high):
    return max(low, min(high, value))

def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

@dataclass(frozen=True)
class State:
    x:float
    y: float
    z: float
    vx: float
    vy: float

    @property
    def course(self):
        return math.atan2(self.vx, self.vy)

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)


@dataclass(frozen=True)
class Command:
    course:float
    altitude:float  # ENU; высоту начала координат добавит ROS-нода.
    speed:float

class Follower:
    def __init__(self, behind=60.0, above=35.0, right= 0.0):
        self.behind = behind
        self.above = above
        self.right = right
        self.requested_behind = behind
        self.requested_right = right
        self.heading = None
        self.omega = 0.0
        self.previous = None

    def set_slot(self, behind, right):
        if not (
            math.isfinite(behind)
            and math.isfinite(right)
            and behind > 0.0
        ):
            raise ValueError('Invalid formation offset')

        self.requested_behind = behind
        self.requested_right = right


    def advance_slot(self, dt):
        db = self.requested_behind - self.behind
        dr = self.requested_right - self.right

        distance = math.hypot(db, dr)

        if distance < 1e-9:
            return 0.0, 0.0

        # Скорость изменения смещения относительно лидера — до 2 м/с.
        scale = min(1.0, 2.0 * dt / distance)

        step_b = db * scale
        step_r = dr * scale

        self.behind += step_b
        self.right += step_r

        return step_b / dt, step_r / dt
            

    def step(self, leader, follower, dt, active):
        if not math.isfinite(dt) or not 0.0 < dt <= 0.25:
            raise ValueError('dt must be in (0, 0.25]')

        alpha = dt/(2.0 + dt)

        if self.heading is None:
            self.heading = leader.course

        delta = alpha * wrap(leader.course - self.heading)
        self.heading = wrap(self.heading + delta)

        # Курс увеличивается по часовой стрелке,
        # угловая скорость в ENU — против часовой.
        self.omega += alpha * (-delta / dt - self.omega)
        self.omega = clamp(self.omega, -leader.speed/120.0, leader.speed/120.0)

        behind_rate, right_rate = 0.0, 0.0

        if active:
            behind_rate, right_rate = self.advance_slot(dt)

        forward_x = math.sin(leader.course)
        forward_y = math.cos(leader.course)

        ox = -self.behind * forward_x + self.right * forward_y
        oy = -self.behind * forward_y - self.right * forward_x

        target = (
            leader.x + ox,
            leader.y + oy,
            leader.z + self.above,
        )

        ex = target[0] - follower.x
        ey = target[1] - follower.y

        cx = 0.055 * ex
        cy = 0.055 * ey

        capture = clamp(
            (math.hypot(ex, ey) - 80.0) / 220.0,
            0.0,
            1.0,
        )

        correction_limit = 4.0 + 16.0 * capture

        scale = min(
            1.0,
            correction_limit / max(math.hypot(cx, cy), 1e-9),
        )

        # Скорость движущегося места в строю + коррекция ошибки.
        slot_vx = (
            -behind_rate * forward_x
            + right_rate * forward_y
        )

        slot_vy = (
            -behind_rate * forward_y
            - right_rate * forward_x
        )
        
        # 4 составляющие:
        # движение лидера
        # + вращение строя
        # + перемещение места при перестроении
        # + исправление ошибки ведомого
        vx = leader.vx - self.omega * oy + slot_vx + cx * scale
        vy = leader.vy + self.omega * ox + slot_vy + cy * scale



        norm = math.hypot(vx, vy)


        desired_course = (
            math.atan2(vx, vy)
            if norm > 1e-6
            else follower.course
        )

        desired_speed = clamp(norm, 10.0, 19.0)

        # В Hold не накапливаем команды, которые самолёт не исполняет.
        old = self.previous if active else None

        if old is None:
            old = Command(
                follower.course,
                follower.z,
                clamp(follower.speed, 10.0, 19.0),
            )

        speed = old.speed + clamp(
            desired_speed - old.speed,
            -dt,
            dt,
        )

        actual_speed = max(follower.speed, speed, 10.0)

        turn_rate = min(
            9.80665 * math.tan(math.radians(25.0)) / actual_speed,
            actual_speed / 120.0,
        )

        course = wrap(
            old.course + clamp(
                wrap(desired_course - old.course),
                -turn_rate * dt,
                turn_rate * dt,
            )
        )

        altitude = old.altitude + clamp(
            target[2] - old.altitude,
            -1.5 * dt,
            1.5 * dt,
        )

        command = Command(course, altitude, speed)
        self.previous = command if active else None

        return command, target
