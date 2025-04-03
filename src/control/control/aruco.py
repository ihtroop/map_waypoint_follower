import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray, String

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        self.bridge = CvBridge()
        
        # Initialize target dimensions
        self.target_width = None
        self.target_height = None
        
        # Create a subscriber to receive images from the /camera/image_raw topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Initialize numpy array to store marker information
        self.marker_info = np.zeros((0, 12), dtype=np.int32)  # [marker-id, tl_x, tr_x, bl_x, br_x, tl_y, tr_y, bl_y, br_y, center_x, center_y, area]

        # List to store detected marker IDs
        self.detected_markers = []

        # Load the aruco dictionary
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

        # Create aruco parameters
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Create a publisher for marker info
        self.marker_info_pub = self.create_publisher(Int32MultiArray, 'marker_info', 10)
        
        # Create a publisher for marker status
        self.marker_status_pub = self.create_publisher(String, 'marker_status', 10)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV frame
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        # Update target dimensions based on the received frame's resolution
        self.target_height, self.target_width = frame.shape[:2]
        
        # Refresh marker_info numpy array for every frame
        self.marker_info = np.zeros((0, 12), dtype=np.int32)
        
        # Detect ArUco markers and annotate the frame
        annotated_frame = self.detect_markers(frame)
        popup_frame = cv2.resize(annotated_frame, (self.target_width, self.target_height))

        # Publish marker information
        self.publish_marker_info()

        # Display the annotated frame
        cv2.imshow('Webcam with ArUco Detection', popup_frame)
        cv2.waitKey(1)

    def detect_markers(self, frame):
        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Draw a green line in the middle
        cv2.line(frame, (int(self.target_width // 3), 0), (int(self.target_width // 3), self.target_height), (0, 255, 0), 2)
        cv2.line(frame, (2*int(self.target_width // 3), 0), (2* int(self.target_width // 3), self.target_height), (0, 255, 0), 2)
        
        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        # Process detected markers
        if ids is not None:
            for i, marker_id in enumerate(ids):
                marker_corners = corners[i].squeeze()  # Get corners for each marker

                # Calculate area of the quadrilateral formed by ArUco tag
                area = self.calculate_quadrilateral_area(marker_corners)

                # Calculate the center coordinates of the ArUco tag
                center_x = int((marker_corners[0][0] + marker_corners[2][0]) / 2)
                center_y = int((marker_corners[0][1] + marker_corners[2][1]) / 2)

                # Append marker information with area and center coordinates to marker_info array
                total_area = int(area)
                marker_info_row = np.array([marker_id[0], marker_corners[0][0], marker_corners[1][0],
                                            marker_corners[2][0], marker_corners[3][0],
                                            marker_corners[0][1], marker_corners[1][1],
                                            marker_corners[2][1], marker_corners[3][1],
                                            center_x, center_y, total_area], dtype=np.int32)
                self.marker_info = np.vstack((self.marker_info, marker_info_row))

                # Annotate the ArUco tag
                self.annotate_aruco(frame, marker_corners, marker_id[0], center_x, center_y, total_area)

                # Publish the marker status for every detected marker
                self.publish_marker_status()

        return frame


    def calculate_quadrilateral_area(self, corners):
        # Calculate area of triangle 1
        x1_tl, y1_tl = corners[0]
        x2_tr, y2_tr = corners[1]
        x3_br, y3_br = corners[3]
        area_triangle1 = 0.5 * abs(x1_tl * (y2_tr - y3_br) + x2_tr * (y3_br - y1_tl) + x3_br * (y1_tl - y2_tr))

        # Calculate area of triangle 2
        x4_bl, y4_bl = corners[2]
        area_triangle2 = 0.5 * abs(x1_tl * (y4_bl - y3_br) + x4_bl * (y3_br - y1_tl) + x3_br * (y1_tl - y4_bl))

        # Calculate total area of the quadrilateral
        total_area = area_triangle1 + area_triangle2
        return total_area

    def annotate_aruco(self, frame, corners, marker_id, center_x, center_y, area):
        # Convert center coordinates to integers
        center_x = int(center_x)
        center_y = int(center_y)
        
        # Annotate the numerical value of area in the center of the ArUco tag
        cv2.putText(frame, str(area), (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Annotate a box on the ArUco tag (lavender color)
        cv2.polylines(frame, [np.int32(corners)], True, (230, 230, 250), 2)

        # Annotate the marker id on the top edge (center-aligned) in lavender color, outside
        cv2.putText(frame, str(marker_id), (center_x, int(corners[0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 250), 1, cv2.LINE_AA)

        # Annotate the corner coordinates in very small text at their respective coordinates (pink color)
        for corner in corners:
            cv2.putText(frame, f"({corner[0]}, {corner[1]})", (int(corner[0]), int(corner[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 192, 203), 1)

        # Annotate a red circular dot at the center coordinates
        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

    def publish_marker_info(self):
        # Create Int32MultiArray message
        marker_info_msg = Int32MultiArray()
        marker_info_msg.data = self.marker_info.flatten().tolist()

        # Publish marker info
        self.marker_info_pub.publish(marker_info_msg)
    
    def publish_marker_status(self):
        # Create String message with "1"
        marker_status_msg = String()
        marker_status_msg.data = "1"

        # Publish marker status
        self.marker_status_pub.publish(marker_status_msg)


def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
