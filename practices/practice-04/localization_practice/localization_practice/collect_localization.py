import csv
from math import atan2, cos, hypot, isfinite, sin
from pathlib import Path
from typing import NamedTuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from matplotlib.figure import Figure
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

SAMPLE_PERIOD = 0.1
AMCL_POSE_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Pose(NamedTuple):
    x: float
    y: float
    yaw: float


class Sample(NamedTuple):
    time: float
    truth: Pose
    estimate: Pose
    position_error: float
    yaw_error: float


def planar_pose(transform):
    translation, rotation = transform.translation, transform.rotation
    yaw = atan2(
        2 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1 - 2 * (rotation.y**2 + rotation.z**2),
    )
    return Pose(translation.x, translation.y, yaw)


def measure(elapsed, truth, estimate):
    angle_delta = estimate.yaw - truth.yaw
    position_error = hypot(estimate.x - truth.x, estimate.y - truth.y)
    yaw_error = abs(atan2(sin(angle_delta), cos(angle_delta)))
    return Sample(elapsed, truth, estimate, position_error, yaw_error)


def convergence_time(samples, position_threshold, yaw_threshold, confirmation_duration):
    interval_start = None
    previous_time = None
    for sample in samples:
        if (
            previous_time is not None
            and sample.time - previous_time > 2 * SAMPLE_PERIOD + 1e-9
        ):
            interval_start = None
        previous_time = sample.time
        if (
            sample.position_error > position_threshold
            or sample.yaw_error > yaw_threshold
        ):
            interval_start = None
            continue
        if interval_start is None:
            interval_start = sample.time
        if sample.time - interval_start >= confirmation_duration:
            return interval_start
    return None


class LocalizationCollector(Node):
    def __init__(self):
        super().__init__("collect_localization")
        self.duration = self.declare_parameter("duration", 200.0).value
        self.position_threshold = self.declare_parameter(
            "position_threshold", 0.15
        ).value
        self.yaw_threshold = self.declare_parameter("yaw_threshold", 0.12).value
        self.confirmation_duration = self.declare_parameter(
            "confirmation_duration", 10.0
        ).value
        self.output_prefix = Path(
            self.declare_parameter("output_prefix", "results/experiment").value
        )
        for name, value in (
            ("duration", self.duration),
            ("position_threshold", self.position_threshold),
            ("yaw_threshold", self.yaw_threshold),
            ("confirmation_duration", self.confirmation_duration),
        ):
            if not isfinite(value) or value <= 0:
                raise ValueError(f"Параметр «{name}» должен быть конечным числом больше нуля.")
        if self.confirmation_duration > self.duration:
            raise ValueError(
                "Время подтверждения сходимости не может превышать длительность эксперимента."
            )

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.start_time = None
        self.samples = []
        self.finished = False
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_first_amcl_pose, AMCL_POSE_QOS
        )
        self.timer = self.create_timer(SAMPLE_PERIOD, self.collect)

    def on_first_amcl_pose(self, message):
        if self.start_time is not None:
            return
        self.start_time = Time.from_msg(message.header.stamp)
        seconds = self.start_time.nanoseconds * 1e-9
        self.get_logger().info(f"Получена первая оценка AMCL. Начало сбора: {seconds:.3f} с симуляции.")

    def collect(self):
        if self.start_time is None:
            return

        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        try:
            self.collect_pose_pair(now)
        except TransformException:
            pass

        if elapsed < self.duration:
            return
        if not self.samples:
            raise RuntimeError("Не удалось получить из TF2 ни одной пары поз на одну отметку времени.")
        self.save()
        self.finished = True

    def collect_pose_pair(self, now):
        estimate = self.buffer.lookup_transform("map", "base_link", Time())
        stamp = Time.from_msg(estimate.header.stamp)
        sample_time = (stamp - self.start_time).nanoseconds * 1e-9
        if not 0 <= sample_time <= self.duration or stamp > now:
            return
        if self.samples and sample_time <= self.samples[-1].time:
            return

        truth = self.buffer.lookup_transform("map", "base_link_gt", stamp)
        truth_pose = planar_pose(truth.transform)
        estimated_pose = planar_pose(estimate.transform)
        self.samples.append(measure(sample_time, truth_pose, estimated_pose))

    def save(self):
        converged_at = convergence_time(
            self.samples,
            self.position_threshold,
            self.yaw_threshold,
            self.confirmation_duration,
        )
        prefix = self.output_prefix
        prefix.parent.mkdir(parents=True, exist_ok=True)
        with prefix.with_name(prefix.name + ".csv").open(
            "w", newline="", encoding="utf-8"
        ) as file:
            writer = csv.writer(file)
            writer.writerow(
                (
                    "time_s",
                    "gt_x_m",
                    "gt_y_m",
                    "gt_yaw_rad",
                    "amcl_x_m",
                    "amcl_y_m",
                    "amcl_yaw_rad",
                    "position_error_m",
                    "yaw_error_rad",
                )
            )
            for sample in self.samples:
                writer.writerow(
                    (
                        sample.time,
                        *sample.truth,
                        *sample.estimate,
                        sample.position_error,
                        sample.yaw_error,
                    )
                )

        result = (
            "Сходимость не подтверждена"
            if converged_at is None
            else f"Время сходимости: {converged_at:.3f} с"
        )
        prefix.with_name(prefix.name + ".txt").write_text(
            result + "\n", encoding="utf-8"
        )
        self.save_plot(prefix.with_name(prefix.name + ".png"), converged_at)
        self.get_logger().info(
            f"{result}. Записано {len(self.samples)} измерений: {prefix}"
        )

    def save_plot(self, filename, converged_at):
        figure = Figure(figsize=(8, 12), layout="constrained")
        trajectory, position, yaw = figure.subplots(3, 1)
        trajectory.plot(
            [sample.truth.x for sample in self.samples],
            [sample.truth.y for sample in self.samples],
            label="Истинная поза (GT)",
        )
        trajectory.plot(
            [sample.estimate.x for sample in self.samples],
            [sample.estimate.y for sample in self.samples],
            label="Оценка AMCL",
        )
        trajectory.set(xlabel="X, м", ylabel="Y, м", aspect="equal", title="Траектории")
        trajectory.grid()
        trajectory.legend()
        times = [sample.time for sample in self.samples]
        for axis, values, threshold, label in (
            (
                position,
                [sample.position_error for sample in self.samples],
                self.position_threshold,
                "Ошибка положения, м",
            ),
            (
                yaw,
                [sample.yaw_error for sample in self.samples],
                self.yaw_threshold,
                "Ошибка угла рысканья, рад",
            ),
        ):
            axis.plot(times, values, label="Ошибка")
            axis.axhline(threshold, color="tab:red", linestyle="--", label="Порог")
            if converged_at is not None:
                axis.axvline(
                    converged_at,
                    color="tab:green",
                    linestyle=":",
                    label=f"Сходимость: {converged_at:.2f} с",
                )
            axis.set(
                xlabel="Время от начала эксперимента, с",
                ylabel=label,
                xlim=(0, self.duration),
            )
            axis.grid()
            axis.legend()
        figure.savefig(filename, dpi=150)


def main():
    rclpy.init()
    node = LocalizationCollector()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
