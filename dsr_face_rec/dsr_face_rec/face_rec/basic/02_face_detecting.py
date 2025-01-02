# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import face_recognition
import threading
import time
import numpy as np
import cv2

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 50, 50

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

face_center = None
img_center = None
old_time = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("force_control", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    # try:
    #     from DSR_ROBOT2 import (
    #         get_current_posx,
    #         set_tool,
    #         set_tcp,
    #         movej,
    #         amovel,
    #         wait,
    #         DR_MV_RA_OVERRIDE,
    #         movel,
    #         mwait,
    #         DR_TOOL,
    #     )

    #     from DR_common2 import posx, posb, posj

    # except ImportError as e:
    #     print(f"Error importing DSR_ROBOT2 : {e}")
    #     return


    def image_callback_color(msg):
        face_center = None
        img_center = None
        global old_time
        current_time = time.time()

        frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        # 현재 프레임에서 얼굴 영역과 임베딩 추출
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_center = (frame.shape[1] // 2, frame.shape[0] // 2)

        print("Image Recognition")

        face_locations = face_recognition.face_locations(rgb_frame, model="cnn")
        print("Image Recognition Finish")

        for (top, right, bottom, left) in face_locations:

            # 매칭 결과에 따라 라벨 결정
            name = "Unknown"

            # 얼굴 영역에 라벨 표시
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)

            # 중심에 점 찍기
            cv2.circle(
                frame,
                (left + (right - left) // 2, top + (bottom - top) // 2),
                5,
                (0, 255, 0),
                -1,
            )
            face_center = (left + (right - left) // 2, top + (bottom - top) // 2)
            face_center_str = (
                f"({left + (right - left) // 2}, {top + (bottom - top) // 2})"
            )
            # cv2.putText(
            #     frame,
            #     face_center_str,
            #     (left, top - 30),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     0.75,
            #     (0, 255, 0),
            #     2,
            # )

            cv2.putText(
                frame,
                face_center_str + name,
                (left, top - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 255, 0),
                2,
            )

        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)


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

        move_thread.join()


if __name__ == "__main__":
    main()
