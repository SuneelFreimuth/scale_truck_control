import rospy
from sensor_msgs.msg import Image
from scale_truck_control.msg import LaneCoefs
from cv_bridge import CvBridge
from ultrafastLaneDetector import UltrafastLaneDetector, ModelType
import cv2

POLY_DEGREE = 2
MODEL_PATH = 'models/lane.onnx'
MODEL_TYPE = ModelType.TUSIMPLE
DISPLAY_IMAGE = False

cv_bridge = CvBridge()
lane_detector = UltrafastLaneDetector(MODEL_PATH, MODEL_TYPE)
pub_lane_coefs = None

def on_usb_cam_image(msg: Image):
    img = cv_bridge.imgmsg_to_cv2(msg)
    output = lane_detector.inference(img)
    lanes_points, lanes_detected = UltrafastLaneDetector.process_output(output, lane_detector.cfg)
    if DISPLAY_IMAGE:
        UltrafastLaneDetector.draw_lanes(img, lanes_points, lanes_detected,
            lane_detector.cfg, True)
        cv2.imshow('Detected Lanes', img)
    # Polyfit
    # Publish

def detect_lanes():
    rospy.init_node('onnx_lane_detector')
    rospy.Subscriber('/usb_cam/image_raw', Image, on_usb_cam_image)
    global pub_lane_coefs
    pub_lane_coefs = rospy.Publisher('/lane_coefs', LaneCoefs)
    rospy.spin()

if __name__ == '__main__':
    try:
        detect_lanes()
    except rospy.ROSInterruptException:
        pass
