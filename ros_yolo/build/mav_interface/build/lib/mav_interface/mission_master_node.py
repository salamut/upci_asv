#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandLong


class MissionMaster(Node):
    """
    Node ini tugasnya:
    - Tunggu FCU connect
    - ARM otomatis
    - Set mode AUTO
    - Kirim MAV_CMD_MISSION_START
    - Setelah itu cuma monitor (tidak ganggu avoidance)
    """

    def __init__(self):
        super().__init__('mission_master')

        # --- Subscribe state Pixhawk ---
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, 10
        )

        # --- Service clients ---
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client   = self.create_client(SetMode,      '/mavros/set_mode')
        self.cmd_client    = self.create_client(CommandLong,  '/mavros/cmd/command')

        # --- State vars ---
        self.current_state = None
        self.mission_started = False

        # Timer utama: state machine
        self.timer = self.create_timer(1.0, self.main_loop)

    # ============================
    # Callbacks
    # ============================
    def state_cb(self, msg: State):
        self.current_state = msg

    # ============================
    # Helper: call services
    # ============================
    def arm(self, value: bool = True):
        if not self.arming_client.service_is_ready():
            self.get_logger().warn("Arming service not ready")
            return

        req = CommandBool.Request()
        req.value = value
        self.arming_client.call_async(req)
        self.get_logger().info(f"Request ARM = {value}")

    def set_mode(self, mode: str):
        if not self.mode_client.service_is_ready():
            self.get_logger().warn("SetMode service not ready")
            return

        req = SetMode.Request()
        req.custom_mode = mode
        self.mode_client.call_async(req)
        self.get_logger().info(f"Request mode → {mode}")

    def start_mission(self):
        """
        Kirim MAV_CMD_MISSION_START (command=300)
        """
        if not self.cmd_client.service_is_ready():
            self.get_logger().warn("CommandLong service not ready")
            return

        req = CommandLong.Request()
        req.command = 300  # MAV_CMD_MISSION_START
        req.confirmation = 0
        # param1 = first item, param2 = last item (0 = use default)
        req.param1 = 0.0
        req.param2 = 0.0

        self.cmd_client.call_async(req)
        self.get_logger().info("MAV_CMD_MISSION_START sent")
        self.mission_started = True

    # ============================
    # MAIN LOOP (state machine)
    # ============================
    def main_loop(self):
        # Belum ada state dari Pixhawk
        if self.current_state is None:
            self.get_logger().warn("No FCU state yet...")
            return

        # 1) Pastikan sudah connect ke FCU
        if not self.current_state.connected:
            self.get_logger().warn("FCU not connected yet")
            return

        # 2) ARM kalau belum
        if not self.current_state.armed:
            self.get_logger().info("Auto-arming...")
            self.arm(True)
            return  # tunggu loop berikutnya

        # 3) Set mode AUTO kalau belum
        if self.current_state.mode != "AUTO" and not self.mission_started:
            # Jangan paksa AUTO kalau avoidance lagi GUIDED;
            # mission start cuma perlu sekali di awal.
            self.get_logger().info(f"Current mode: {self.current_state.mode}, requesting AUTO")
            self.set_mode("AUTO")
            return

        # 4) Kalau sudah AUTO & armed & mission belum start → start mission
        if (self.current_state.mode == "AUTO"
                and self.current_state.armed
                and not self.mission_started):
            self.get_logger().info("AUTO & armed → starting mission")
            self.start_mission()
            return

        # 5) Setelah mission_started = True → hanya monitor
        self.get_logger().debug(
            f"Mission running. Mode={self.current_state.mode}, armed={self.current_state.armed}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MissionMaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
