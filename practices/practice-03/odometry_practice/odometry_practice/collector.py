import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
from tf_transformations import euler_from_quaternion


class Collector(Node):
    def __init__(self):
        super().__init__("collector")
        self.sim_time = None
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.cmd_x = None
        self.cmd_y = None
        self.cmd_yaw = None
        self.previous_time = None
        self.command_history = []
        self.start_time = None
        self.readings = []
        self.finished = False
        self.duration = float(self.declare_parameter("duration", 100.0).value)
        self.encoder_noise = float(self.declare_parameter("encoder_noise", 0.0).value)
        self.output_dir = Path(
            self.declare_parameter("output_dir", str(Path.cwd() / "results")).value
        ).expanduser()
        if self.duration <= 0:
            raise ValueError("duration must be positive")

        self.tf_buffer = Buffer(node=self)
        self.tf_transformer = TransformListener(self.tf_buffer, self)
        self.time_subscription = self.create_subscription(
            Clock, "/clock", self.on_time, qos_profile_sensor_data
        )
        self.speed_subscription = self.create_subscription(
            TwistStamped, "cmd_vel", self.on_speed, 10
        )
        self.reading = self.create_timer(0.1, self.collect_reading)

    def on_time(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_speed(self, msg):
        self.linear_speed = msg.twist.linear.x
        self.angular_speed = msg.twist.angular.z
        command_time = Time.from_msg(msg.header.stamp).nanoseconds
        if command_time == 0:
            if self.sim_time is None:
                return
            command_time = round(self.sim_time * 1e9)
        self.command_history.append(
            (command_time, self.linear_speed, self.angular_speed)
        )

    def collect_reading(self):
        if self.finished or self.sim_time is None:
            return
        try:
            self.gt_transform = self.tf_buffer.lookup_transform(
                "map", "base_link_gt", Time()
            )
            self.odom_transform = self.tf_buffer.lookup_transform(
                "map", "base_link", Time()
            )
            self.gt_time = Time.from_msg(self.gt_transform.header.stamp)
            self.odom_time = Time.from_msg(self.odom_transform.header.stamp)
            if self.gt_time.nanoseconds > self.odom_time.nanoseconds:
                self.reading_time = self.odom_time
            else:
                self.reading_time = self.gt_time
            if self.start_time is not None:
                end_time = self.start_time.nanoseconds + round(self.duration * 1e9)
                if self.reading_time.nanoseconds > end_time:
                    self.reading_time = Time(
                        nanoseconds=end_time, clock_type=self.reading_time.clock_type
                    )
            self.gt_transform = self.tf_buffer.lookup_transform(
                "map", "base_link_gt", self.reading_time
            )
            self.odom_transform = self.tf_buffer.lookup_transform(
                "map", "base_link", self.reading_time
            )
            self.x = self.gt_transform.transform.translation.x
            self.y = self.gt_transform.transform.translation.y
            self.q = self.gt_transform.transform.rotation
            self.gt_yaw = euler_from_quaternion(
                [self.q.x, self.q.y, self.q.z, self.q.w]
            )[2]
            self.odom_x = self.odom_transform.transform.translation.x
            self.odom_y = self.odom_transform.transform.translation.y
            self.odom_q = self.odom_transform.transform.rotation
            self.odom_yaw = euler_from_quaternion(
                [self.odom_q.x, self.odom_q.y, self.odom_q.z, self.odom_q.w]
            )[2]
        except TransformException:
            return
        if not self.update_command_pose():
            return
        elapsed = (self.reading_time.nanoseconds - self.start_time.nanoseconds) * 1e-9
        self.readings.append({
            "time_s": elapsed,
            "simulation_time_s": self.reading_time.nanoseconds * 1e-9,
            "gt_x_m": self.x, "gt_y_m": self.y, "gt_yaw_rad": self.gt_yaw,
            "odom_x_m": self.odom_x, "odom_y_m": self.odom_y,
            "odom_yaw_rad": self.odom_yaw,
            "cmd_x_m": self.cmd_x, "cmd_y_m": self.cmd_y,
            "cmd_yaw_rad": self.cmd_yaw,
            "odom_position_error_m": math.hypot(self.odom_x - self.x, self.odom_y - self.y),
            "cmd_position_error_m": math.hypot(self.cmd_x - self.x, self.cmd_y - self.y),
            "odom_yaw_error_rad": abs(math.atan2(
                math.sin(self.odom_yaw - self.gt_yaw),
                math.cos(self.odom_yaw - self.gt_yaw),
            )),
            "cmd_yaw_error_rad": abs(math.atan2(
                math.sin(self.cmd_yaw - self.gt_yaw),
                math.cos(self.cmd_yaw - self.gt_yaw),
            )),
        })
        if len(self.readings) == 1 or int(elapsed // 10) > int(self.readings[-2]["time_s"] // 10):
            self.get_logger().info(f"Collected {elapsed:.1f}/{self.duration:g} s of simulation")
        if elapsed >= self.duration - 1e-9:
            self.save_results()

    def update_command_pose(self):
        current_time = self.reading_time.nanoseconds
        commands = sorted(
            (command for command in self.command_history if command[0] <= current_time),
            key=lambda command: command[0],
        )
        if not commands:
            return False

        if self.previous_time is None:
            _, linear_speed, angular_speed = commands[-1]
            if linear_speed == 0.0 and angular_speed == 0.0:
                return False

            self.cmd_x = self.x
            self.cmd_y = self.y
            self.cmd_yaw = self.gt_yaw
            self.previous_time = self.reading_time
            self.start_time = self.reading_time
            return True

        previous_time = self.previous_time.nanoseconds
        
        if current_time <= previous_time:
            return False
        cursor = previous_time
        linear_speed = 0.0
        angular_speed = 0.0
        for stamp, next_linear_speed, next_angular_speed in commands:
            if stamp > previous_time:
                self.integrate_command(
                    linear_speed, angular_speed, (stamp - cursor) * 1e-9
                )
                cursor = stamp
            linear_speed = next_linear_speed
            angular_speed = next_angular_speed

        self.integrate_command(
            linear_speed, angular_speed, (current_time - cursor) * 1e-9
        )
        self.previous_time = self.reading_time
        return True

    def integrate_command(self, linear_speed, angular_speed, dt):
        yaw = self.cmd_yaw
        new_yaw = yaw + angular_speed * dt
        if abs(angular_speed) < 1e-9:
            self.cmd_x += linear_speed * math.cos(yaw) * dt
            self.cmd_y += linear_speed * math.sin(yaw) * dt
        else:
            radius = linear_speed / angular_speed
            self.cmd_x += radius * (math.sin(new_yaw) - math.sin(yaw))
            self.cmd_y -= radius * (math.cos(new_yaw) - math.cos(yaw))
        self.cmd_yaw = new_yaw


    def save_results(self):
        import matplotlib
        matplotlib.use("Agg")
        from matplotlib import pyplot as plt

        self.reading.cancel()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        name = "noise_" + format(self.encoder_noise, "g").replace(".", "_")
        csv_path = self.output_dir / (name + ".csv")
        with csv_path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(self.readings[0]))
            writer.writeheader()
            writer.writerows(self.readings)

        times = [row["time_s"] for row in self.readings]
        figure, axes = plt.subplots(1, 3, figsize=(18, 5), layout="constrained")
        for prefix, label, color in (
            ("gt", "Истинная (GT)", "black"),
            ("odom", "По энкодерам", "tab:blue"),
            ("cmd", "По командам", "tab:orange"),
        ):
            axes[0].plot(
                [row[f"{prefix}_x_m"] for row in self.readings],
                [row[f"{prefix}_y_m"] for row in self.readings],
                label=label, color=color, linewidth=1.5,
            )
        axes[0].plot(self.readings[0]["gt_x_m"], self.readings[0]["gt_y_m"],
                     "go", markersize=5, label="Начало")
        axes[0].set(xlabel="X, м", ylabel="Y, м", title="Траектории")
        axes[0].set_aspect("equal", adjustable="box")
        for prefix, label, color in (
            ("odom", "По энкодерам", "tab:blue"),
            ("cmd", "По командам", "tab:orange"),
        ):
            axes[1].plot(times, [row[f"{prefix}_position_error_m"] for row in self.readings],
                         label=label, color=color)
            axes[2].plot(times, [row[f"{prefix}_yaw_error_rad"] for row in self.readings],
                         label=label, color=color)
        axes[1].set(xlabel="Время от начала, с", ylabel="Ошибка положения, м",
                    title="Абсолютная ошибка положения")
        axes[2].set(xlabel="Время от начала, с", ylabel="Ошибка угла, рад",
                    title="Абсолютная ошибка угла")
        for axis in axes:
            axis.grid(True, alpha=0.3)
            axis.legend()
        figure.suptitle(
            f"Шум энкодеров: {self.encoder_noise:g} рад; длительность: {self.duration:g} с"
        )
        image_path = self.output_dir / (name + ".png")
        figure.savefig(image_path, dpi=160)
        plt.close(figure)
        self.finished = True
        self.get_logger().info(
            f"COMPLETE: {len(self.readings)} readings; data: {csv_path}; plot: {image_path}"
        )



def main(args=None):
    rclpy.init(args=args)
    node = Collector()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
