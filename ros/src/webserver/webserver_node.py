#!/usr/bin/env python
from sensor_msgs.msg import Image
import rospy
from flask import Flask, Response
import cv2
from cv_bridge import CvBridge, CvBridgeError



def run():
    app = Flask(__name__)
    app.run(host='127.0.0.1', port=5000)

    bridge = CvBridge()
    rospy.Subscriber('/detector/image', Image, get_image)

    img = b''

    def get_image(resp):
        try:
            cv_image = bridge.imgmsg_to_cv2(resp, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        _, buf = cv2.imencode('.jpg', cv_image)
        img = buf.tostring()

    def gen(self):
        while True:
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + img + b'\r\n')

    @app.route('/')
    def video_feed(self):
        return Response(self.gen(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    run()
    rospy.spin()
