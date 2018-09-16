#!/usr/bin/python
import firebase_admin
from firebase_admin import credentials, db
import rospy
from std_msgs.msg import Bool, String, Int64
from sensor_msgs.msg import Image
from detector.msg import RegionOfInterestArray


from time import sleep
import cv2
import os
import random
import base64
from time import time
from cv_bridge import CvBridge, CvBridgeError

import rospy

from RunState import RunState


class Commander:
    def __init__(self):
        self.bridge = CvBridge()

        cred = credentials.Certificate('secret.json')
        self.app = firebase_admin.initialize_app(
            credential=cred,
            options={
                'databaseURL' : 'https://agribot-2019.firebaseio.com/',
            }
        )
        
        # Pubs
        self.motor_pub = rospy.Publisher('motor/start', Bool, queue_size=5)

        # Subs
        # rospy.Subscriber('/actuator', Int64, self.actuator)
        rospy.Subscriber('/detector/image', Image, self.get_image)
        rospy.Subscriber('/detector/rois', RegionOfInterestArray, self.send_roi)

        robo_ref = db.reference('/robot/0')
        robo_ref.get().get('running', False)

        self.connection = robo_ref.listen(self.robot_command)

        self.state = RunState.IDLE
        self.roi = None

    def robot_command(self, resp):
        running = bool(resp.data.get('running'))

        if running and RunState.IDLE:
            self.state = RunState.SEARCH
        else:
            self.state = RunState.IDLE

    def send_roi(self, roiArray):
        # TODO: Select first roi
        if len(roiArray.rois) > 0:
            self.roi = roiArray.rois[0]
        db.reference('/roi/0').push({
            "time": time(),
            "num": len(roiArray.rois),
        })

    def get_image(self, resp):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(resp, "bgr8")
            self.got_image = True
        except CvBridgeError as e:
            print(e)
            return

        _, buf = cv2.imencode('.jpg', cv_image)
        encoded = "data:image/jpg;base64," + base64.b64encode(buf)

        db.reference('/video/0').set({
            "image": encoded,
        })

    def actuator(self, resp): # Resp is Int64
        num_sends = resp.data
        db.reference('/actuator/0').set({
            "time": time(),
            "num": num_sends,
        })

    def enable_motor(self, enable):
        # TODO: Add distance estimates
        if enable:
            self.motor_pub.publish(Bool(True))
        else:
            self.motor_pub.publish(Bool(False))

    def loop(self):
        if self.state == RunState.IDLE:
            self.enable_motor(False)
        elif self.state == RunState.SEARCH:
            self.enable_motor(True)

            if self.roi is not None:
                self.roi = None
                self.state = RunState.IDLE
        else:
            self.state = RunState.IDLE

    def run(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()
 
        self.close()

    def close(self):
        self.connection.close()


if __name__ == "__main__":
    rospy.init_node('commander')
    c = Commander()
    try:
        c.run()
    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
        c.close()

    rospy.spin()
