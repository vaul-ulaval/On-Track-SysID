import rclpy
from rclpy.node import Node
import numpy as np
import os
import yaml
import csv
from datetime import datetime
from tqdm import tqdm
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from on_track_sys_id.helpers.train_model import nn_train


class OnTrackSysId(Node):
    def __init__(self):
        super().__init__("on_track_sys_id")

        self.declare_parameter("racecar_version", "SIM")
        self.racecar_version = self.get_parameter("racecar_version").value
        self.get_logger().info(f"Racecar version: {self.racecar_version}")

        self.rate = 50

        self.package_path: str = (
            self.declare_parameter(
                "package_path", "/home/nicolaslauzon/ws/vaul/sim_ws/src/On-Track-SysID"
            ).value
            or ""
        )
        self.load_parameters()
        self.setup_data_storage()

        self.declare_parameter("save_LUT_name", "default_LUT")
        self.declare_parameter("plot_model", False)

        self.save_LUT_name = self.get_parameter("save_LUT_name").value
        self.plot_model = self.get_parameter("plot_model").value

        odom_topic: str = (
            self.declare_parameter("odom_topic", "/ego_racecar/odom").value or ""
        )
        ackermann_topic: str = (
            self.declare_parameter("ackermann_cmd_topic", "/drive").value or ""
        )

        self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.create_subscription(
            AckermannDriveStamped, ackermann_topic, self.ackermann_cb, 10
        )

        self.timer = self.create_timer(1.0 / self.rate, self.loop)
        self.pbar = tqdm(total=self.timesteps, desc="Collecting data", ascii=True)

    def setup_data_storage(self):
        self.data_duration = self.nn_params["data_collection_duration"]
        self.timesteps = self.data_duration * self.rate
        self.data = np.zeros((self.timesteps, 4))
        self.counter = 0
        self.current_state = np.zeros(4)

    def load_parameters(self):
        yaml_file = os.path.join(self.package_path, "params/nn_params.yaml")
        with open(yaml_file, "r") as file:
            self.nn_params = yaml.safe_load(file)

    def export_data_as_csv(self):
        user_input = input(
            "\033[33m[WARN] Press 'Y' and then ENTER to export data as CSV, or press ENTER to continue without dumping: \033[0m"
        )
        if user_input.lower() == "y":
            data_dir = os.path.join(self.package_path, "data", self.racecar_version)
            os.makedirs(data_dir, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_file = os.path.join(
                data_dir, f"{self.racecar_version}_sys_id_data_{timestamp}.csv"
            )

            with open(csv_file, mode="w") as file:
                writer = csv.writer(file)
                writer.writerow(["v_x", "v_y", "omega", "delta"])
                writer.writerows(self.data)
            self.get_logger().info(f"DATA HAS BEEN EXPORTED TO: {csv_file}")

    def odom_cb(self, msg):
        self.current_state[0] = msg.twist.twist.linear.x
        self.current_state[1] = msg.twist.twist.linear.y
        self.current_state[2] = msg.twist.twist.angular.z

    def ackermann_cb(self, msg):
        self.current_state[3] = msg.drive.steering_angle

    def collect_data(self):
        if self.current_state[0] > 1:
            self.data = np.roll(self.data, -1, axis=0)
            self.data[-1] = self.current_state
            self.counter += 1
            self.pbar.update(1)
        if self.counter == self.timesteps + 1:
            self.pbar.close()
            self.get_logger().info("Data collection completed.")
            self.run_nn_train()
            self.export_data_as_csv()
            self.get_logger().info("Training completed. Shutting down...")
            self.destroy_node()
            rclpy.shutdown()

    def run_nn_train(self):
        self.get_logger().info("Training neural network...")
        nn_train(self.data, self.racecar_version, self.save_LUT_name, self.plot_model)

    def loop(self):
        if self.context.ok():
            self.collect_data()


def main(args=None):
    rclpy.init(args=args)
    node = OnTrackSysId()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
