import os
from ament_index_python.packages import get_package_share_directory
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo

class TrackingCamSync(Node):
    def __init__(self):
        super().__init__("tracking_cam_sync")

        #pkg_share = get_package_share_directory("starling_cams")
        tracking_intrinsics_fn = os.path.join("/workspace/src/starling_cams/cam_configs", "opencv_tracking_down_intrinsics.yml")
        config_data = self.load_intrinsics_cfg(tracking_intrinsics_fn)
        intrinsics = self.get_instrinsics(config_data)
        self.info_msg = self.create_info_msg(intrinsics)

        sub_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        pub_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        input_cam_topic = "/tracking_down"
        output_image_topic = "/camera_down/image_raw"
        output_info_topic = "/camera_down/camera_info"

        self.voxl_image_sub_ = self.create_subscription(
                Image,
                input_cam_topic,
                self.image_callback,
                sub_qos_profile
                )
        self.tracking_image_pub_ = self.create_publisher(
                Image,
                output_image_topic,
                pub_qos_profile
                )
        self.tracking_info_pub_ = self.create_publisher(
                CameraInfo,
                output_info_topic,
                pub_qos_profile
                )

    def load_intrinsics_cfg(self, fn: str) -> dict:
        with open(fn, "r") as file:
            config_data = yaml.safe_load(file)
        return config_data

    def get_instrinsics(self, config_data: dict) -> dict:
        intrinsics = {
                "M" : config_data["M"]["data"],
                "D" : config_data["D"]["data"],
                }
        for key in config_data:
            if key == "M" or key == "D":
                intrinsics[key] = config_data[key]["data"]
            else:
                intrinsics[key] = config_data[key]
        return intrinsics

    def create_info_msg(self, intr: dict) -> CameraInfo:
        info_msg = CameraInfo()
        info_msg.width = intr["width"]
        info_msg.height = intr["height"]

        if intr["distortion_model"] == "fisheye":
            info_msg.distortion_model = "equidistant"
        else:
            self.get_logger().error(f'Incompatible distortion_model: {intr["distortion_model"]}')

        K = intr["M"]
        fx = K[0]
        fy = K[4]
        cx = K[2]
        cy = K[5]
        Tx = Ty = 0 # For monocular cameras

        info_msg.k = K
        info_msg.d = intr["D"] # [k1, k2, k3, k4]
        info_msg.r = [1.0, 0.0, 0.0, #rectification (unused in this case)
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0] 
        info_msg.p = [fx, 0, cx, Tx,
                      0, fy, cy, Ty,
                      0, 0, 1, 0]
        return info_msg

    def image_callback(self, msg):
        #self.get_logger().info("Got image callback")
        timestamp = msg.header.stamp
        frame_id = msg.header.frame_id
        self.info_msg.header.stamp = timestamp
        self.info_msg.header.frame_id = frame_id
        self.tracking_info_pub_.publish(self.info_msg)
        self.tracking_image_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    cam_node = TrackingCamSync()
    rclpy.spin(cam_node)
    cam_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
