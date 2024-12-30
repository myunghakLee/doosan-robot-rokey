# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import cv2

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 50, 50

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1


def image_callback_color(msg):
    # print("Received an image in color")
    # print("Image size: %dx%d" % (msg.width, msg.height))

    # img = np.array(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    # msg.data to cv2
    cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    cv2.imshow("Camera Feed", cv_image)
    cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("force_control", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            amovel,
            wait,
            DR_MV_RA_OVERRIDE,
            movel,
            mwait,
        )

        from DR_common2 import posx, posb, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    # set_tool("Tool Weight_2FG")
    # set_tcp("2FG_TCP")

    # 초기 위치
    JReady = posj(0.00, 0.00, 90.00, -90.00, 90.0, -180.00)
    movej(JReady, vel=VELOCITY, acc=ACC)

    color_image_node = rclpy.create_node("color_image_subscriber")
    color_image_node.create_subscription(
        Image, "/camera/camera/color/image_raw", image_callback_color, 10
    )

    try:
        rclpy.spin(color_image_node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")

    finally:
        color_image_node.destroy_node()
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
