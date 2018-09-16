import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import base64
from cv_bridge import CvBridge, CvBridgeError

import rospy

bridge = CvBridge()


def stream_image(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    _, buf = cv2.imencode('.jpg', cv_image)
    encoded = "data:image/jpg;base64," + base64.b64encode(buf)
    print encoded


rospy.Subscriber('/detector/image', Image, stream_image)

rospy.init_node('test_stream')
rospy.spin()