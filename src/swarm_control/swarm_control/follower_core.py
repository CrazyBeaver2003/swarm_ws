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
    def __init__(self, behind=10.0, above=0.0, right=0.0, entry_right_m=None):
        if entry_right_m is not None and not math.isfinite(entry_right_m):
            raise ValueError('Entry corridor offset must be finite')
        self.behind = behind
        self.above = above
        self.right = right
        self.requested_behind = behind
        self.requested_right = right
        self._transition_remaining_s = 0.0
        self.heading = None
        self.omega = 0.0
        self.previous = None
        self.entry_right_m = entry_right_m
        self.entry_phase = 'inactive'
        self.guidance_right_m = right
        self._entry_ready_s = 0.0
        self._along_integral_m_s = 0.0
        self._entry_filtered_errors = None
        self._entry_aligned = False
        self.entry_diagnostics = {
            'raw': {}, 'filtered': {}, 'ready_s': 0.0, 'aligned': False,
        }

    @property
    def transition_remaining_s(self):
        return self._transition_remaining_s

    def set_slot(self, behind, right, transition_duration_s=None):
        if not (
            math.isfinite(behind)
            and math.isfinite(right)
            and behind > 0.0
        ):
            raise ValueError('Invalid formation offset')

        if transition_duration_s is not None and (
            not math.isfinite(transition_duration_s)
            or transition_duration_s < 0.0
        ):
            raise ValueError('Transition duration must be finite and nonnegative')

        same_target = (
            math.isclose(behind, self.requested_behind, rel_tol=0.0, abs_tol=1e-9)
            and math.isclose(right, self.requested_right, rel_tol=0.0, abs_tol=1e-9)
        )
        if same_target:
            return
        if self._transition_remaining_s > 0.0:
            raise ValueError('Wait for the current formation transition to finish')

        distance = math.hypot(behind - self.behind, right - self.right)
        minimum_duration = distance / 2.0
        duration = (
            minimum_duration
            if transition_duration_s is None
            else transition_duration_s
        )
        if duration + 1e-9 < minimum_duration:
            raise ValueError('Transition would exceed the 2 m/s slot speed limit')

        self.requested_behind = behind
        self.requested_right = right
        self._transition_remaining_s = duration if distance > 1e-9 else 0.0


    def advance_slot(self, dt):
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError('Slot timestep must be finite and positive')

        db = self.requested_behind - self.behind
        dr = self.requested_right - self.right
        remaining = self._transition_remaining_s
        if remaining <= 0.0:
            return 0.0, 0.0

        # Общая длительность для всех ведомых даёт одинаковую долю пути:
        # соседние места не догоняют друг друга при перестроении.
        scale = min(1.0, dt / remaining)

        step_b = db * scale
        step_r = dr * scale

        self.behind += step_b
        self.right += step_r
        self._transition_remaining_s = max(0.0, remaining - dt)
        if self._transition_remaining_s <= 1e-9:
            self.behind = self.requested_behind
            self.right = self.requested_right
            self._transition_remaining_s = 0.0

        return step_b / dt, step_r / dt

    def entry_guidance(self, leader, follower, target, forward_x, forward_y,
                       behind_rate, right_rate, dt, active):
        """Join beside the formation before moving laterally into the final slot."""
        if not active:
            self.entry_phase = 'inactive'
            self._entry_ready_s = 0.0
            self._along_integral_m_s = 0.0
            self._entry_filtered_errors = None
            self._entry_aligned = False
            self.entry_diagnostics = {
                'raw': {}, 'filtered': {}, 'ready_s': 0.0, 'aligned': False,
            }
            self.guidance_right_m = self.right
            return right_rate

        def motion_errors(right):
            ox = -self.behind * forward_x + right * forward_y
            oy = -self.behind * forward_y - right * forward_x
            vx = leader.vx - self.omega * oy - behind_rate * forward_x
            vy = leader.vy + self.omega * ox - behind_rate * forward_y
            return (wrap(math.atan2(vx, vy) - follower.course),
                    math.hypot(vx, vy) - follower.speed)

        if self.entry_right_m is None:
            self.entry_phase = 'tracking'
        elif self.entry_phase == 'inactive':
            at_slot = math.dist(target, (follower.x, follower.y, follower.z)) <= 3.0
            course_error, speed_error = motion_errors(self.right)
            aligned = abs(course_error) <= 0.15 and abs(speed_error) <= 1.0
            self.entry_phase = 'tracking' if at_slot and aligned else 'approach'
            self._entry_ready_s = 0.0
            self._entry_filtered_errors = None
            self._entry_aligned = False

        if self.entry_phase == 'tracking':
            self.guidance_right_m = self.right
            self.entry_diagnostics = {
                'raw': {}, 'filtered': {},
                'ready_s': self._entry_ready_s, 'aligned': True,
            }
            return right_rate

        if self.entry_phase == 'approach':
            self.guidance_right_m = self.entry_right_m

        dx, dy = follower.x - leader.x, follower.y - leader.y
        along_error = -self.behind - (dx * forward_x + dy * forward_y)
        lateral_error = self.guidance_right_m - (dx * forward_y - dy * forward_x)
        course_error, speed_error = motion_errors(self.guidance_right_m)
        raw = {
            'along_m': along_error,
            'lateral_m': lateral_error,
            'course_rad': course_error,
            'speed_m_s': speed_error,
        }
        # Only the entry decision is filtered, not the navigation state or the
        # independent separation guard. Raw mesh speed fluctuates enough that
        # requiring two unbroken seconds of raw readiness can block entry forever.
        alpha = dt / (1.0 + dt)
        if self._entry_filtered_errors is None:
            self._entry_filtered_errors = dict(raw)
        else:
            for name, value in raw.items():
                previous = self._entry_filtered_errors[name]
                error = wrap(value - previous) if name == 'course_rad' else value - previous
                filtered = previous + alpha * error
                self._entry_filtered_errors[name] = wrap(filtered) if name == 'course_rad' else filtered
        filtered = self._entry_filtered_errors
        inner = {'along_m': 2.0, 'lateral_m': 3.0, 'course_rad': 0.15, 'speed_m_s': 1.0}
        outer = {'along_m': 3.0, 'lateral_m': 4.5, 'course_rad': 0.22, 'speed_m_s': 1.5}
        if self._entry_aligned:
            if any(abs(filtered[name]) > limit for name, limit in outer.items()):
                self._entry_aligned = False
                self._entry_ready_s = 0.0
        else:
            ready = all(abs(filtered[name]) <= limit for name, limit in inner.items())
            self._entry_ready_s = self._entry_ready_s + dt if ready else 0.0
            if self._entry_ready_s + 1e-9 >= 2.0:
                self._entry_aligned = True
        self.entry_diagnostics = {
            'raw': raw, 'filtered': dict(filtered),
            'ready_s': self._entry_ready_s, 'aligned': self._entry_aligned,
        }
        if not self._entry_aligned:
            # Pause at the current guide on a sustained departure from alignment;
            # resuming requires the tighter band for another two seconds.
            return 0.0
        self.entry_phase = 'merge'

        # Merge only after matching the leader's motion beside the assigned
        # slot. The returned rate is included in the moving-target feedforward.
        step_right = clamp(self.right - self.guidance_right_m, -0.5 * dt, 0.5 * dt)
        self.guidance_right_m += step_right
        if abs(self.right - self.guidance_right_m) < 1e-9:
            self.guidance_right_m = self.right
            self.entry_phase = 'tracking'
            self.entry_diagnostics = {
                'raw': {}, 'filtered': {},
                'ready_s': self._entry_ready_s, 'aligned': True,
            }
        return step_right / dt

    def step(self, leader, follower, dt, active):
        if not math.isfinite(dt) or not 0.0 < dt <= 0.25:
            raise ValueError('dt must be in (0, 0.25]')

        alpha = dt/(2.0 + dt)

        if self.heading is None:
            self.heading = leader.course

        angular_limit = leader.speed / 120.0
        delta = clamp(
            alpha * wrap(leader.course - self.heading),
            -angular_limit * dt,
            angular_limit * dt,
        )
        self.heading = wrap(self.heading + delta)

        # Курс увеличивается по часовой стрелке,
        # угловая скорость в ENU — против часовой. Это производная именно
        # того курса, которым ниже поворачиваем места в строю.
        self.omega = -delta / dt

        behind_rate, right_rate = 0.0, 0.0

        if active:
            behind_rate, right_rate = self.advance_slot(dt)

        forward_x = math.sin(self.heading)
        forward_y = math.cos(self.heading)

        ox = -self.behind * forward_x + self.right * forward_y
        oy = -self.behind * forward_y - self.right * forward_x

        target = (
            leader.x + ox,
            leader.y + oy,
            leader.z + self.above,
        )

        guidance_right_rate = self.entry_guidance(
            leader, follower, target, forward_x, forward_y,
            behind_rate, right_rate, dt, active)
        ox = -self.behind * forward_x + self.guidance_right_m * forward_y
        oy = -self.behind * forward_y - self.guidance_right_m * forward_x

        # Publish the actual formation slot, not this temporary approach point:
        # mission assembly must not accept a plane still in the entry corridor.
        ex = leader.x + ox - follower.x
        ey = leader.y + oy - follower.y

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
            + guidance_right_rate * forward_y
        )

        slot_vy = (
            -behind_rate * forward_y
            - guidance_right_rate * forward_x
        )
        
        # 4 составляющие:
        # движение лидера
        # + вращение строя
        # + перемещение места при перестроении
        # + исправление ошибки ведомого
        vx = leader.vx - self.omega * oy + slot_vx + cx * scale
        vy = leader.vy + self.omega * ox + slot_vy + cy * scale

        if active and self.entry_right_m is not None:
            # Remove the small steady EAS/ground-speed mismatch near the slot.
            # Without this term a constant speed bias can prevent the tighter
            # entry alignment gate from ever being reached.
            along_error = ex * forward_x + ey * forward_y
            old_integral = self._along_integral_m_s
            if abs(along_error) < 20.0:
                candidate = clamp(old_integral + 0.001 * along_error * dt, -0.7, 0.7)
                old_norm = math.hypot(vx + old_integral * forward_x,
                                      vy + old_integral * forward_y)
                new_norm = math.hypot(vx + candidate * forward_x,
                                      vy + candidate * forward_y)
                if (10.0 <= new_norm <= 19.0
                        or (new_norm > 19.0 and new_norm < old_norm)
                        or (new_norm < 10.0 and new_norm > old_norm)):
                    self._along_integral_m_s = candidate
            vx += self._along_integral_m_s * forward_x
            vy += self._along_integral_m_s * forward_y

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
