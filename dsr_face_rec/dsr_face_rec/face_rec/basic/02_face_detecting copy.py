import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from facenet_pytorch import MTCNN
import torch

class FaceDetectionNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')

        # GPU 사용 가능 여부 확인
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # MTCNN 모델 초기화
        self.mtcnn = MTCNN(keep_all=True, device=self.device)

        # CvBridge 초기화
        self.bridge = CvBridge()

        # ROS2 구독 설정
        self.subscription = self.create_subscription(
            Image,
            'abc',  # Topic 이름
            self.listener_callback,
            10
        )

        self.get_logger().info("Face detection node started, subscribing to 'abc'")

    def listener_callback(self, msg):
        try:
            # ROS2 이미지 메시지를 OpenCV 형식으로 변환
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 얼굴 탐지 수행
            boxes, _ = self.mtcnn.detect(frame)

            # 탐지된 얼굴을 박스로 표시
            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = [int(coord) for coord in box]
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # 결과 프레임 디스플레이
            cv2.imshow('Face Detection', frame)
            cv2.waitKey(1)  # OpenCV 윈도우 업데이트

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    face_detection_node = FaceDetectionNode()

    try:
        rclpy.spin(face_detection_node)
    except KeyboardInterrupt:
        pass

    face_detection_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
