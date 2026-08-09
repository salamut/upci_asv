import rclpy
from rclpy.node import Node

from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import WaypointReached


class WaypointSpeedControl(Node):

    def __init__(self):

        super().__init__('waypoint_speed_control')

        # =================================================
        # MAVROS COMMAND CLIENT
        # =================================================
        self.cmd_client = self.create_client(
            CommandLong,
            '/mavros/cmd/command'
        )

        while not self.cmd_client.wait_for_service(timeout_sec=1.0):

            self.get_logger().info(
                'Waiting MAVROS command service...'
            )

        # =================================================
        # SUBSCRIBE WAYPOINT REACHED
        # =================================================
        self.wp_sub = self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.waypoint_callback,
            10
        )

        self.get_logger().info(
            'Waypoint Speed Controller Started'
        )

    # =====================================================
    # WAYPOINT CALLBACK
    # =====================================================
    def waypoint_callback(self, msg):

        wp = msg.wp_seq

        self.get_logger().info(
            f'Reached waypoint {wp}'
        )

        # =============================================
        # RULE:
        # waypoint ganjil <=5 -> speed 2
        # lainnya -> speed 4
        # =============================================
        if wp <= 5 and wp % 2 == 1:

            speed = 2.0

        else:

            speed = 4.0

        self.change_speed(speed)

    # =====================================================
    # CHANGE SPEED FUNCTION
    # =====================================================
    def change_speed(self, speed_mps):

        req = CommandLong.Request()

        # MAV_CMD_DO_CHANGE_SPEED
        req.command = 178

        req.param1 = 1.0
        req.param2 = speed_mps
        req.param3 = -1.0
        req.param4 = 0.0

        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0

        req.broadcast = False

        future = self.cmd_client.call_async(req)

        future.add_done_callback(
            lambda f: self.speed_result_callback(
                f,
                speed_mps
            )
        )

    # =====================================================
    # RESULT CALLBACK
    # =====================================================
    def speed_result_callback(self, future, speed):

        try:

            result = future.result()

            if result.success:

                self.get_logger().info(
                    f'Speed changed to {speed} m/s'
                )

            else:

                self.get_logger().error(
                    'Failed to change speed'
                )

        except Exception as e:

            self.get_logger().error(str(e))


# =========================================================
# MAIN
# =========================================================
def main(args=None):

    rclpy.init(args=args)

    node = WaypointSpeedControl()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()