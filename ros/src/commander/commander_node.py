from firebase import firebase
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

bridge = CvBridge()
class Commander():
    def __init__(self):
        cred = credentials.Certificate('secret.json')
        self.app = firebase_admin.initialize_app(
            credential=cred,
            options={
                'databaseURL' : 'https://agribot-2019.firebaseio.com/',
            }
        )
        
        # Pubs
        self.pub_motor = rospy.Publisher('motor/start', Bool, queue_size=5)

        # Subs
        # rospy.Subscriber('/actuator', Int64, self.actuator)
        rospy.Subscriber('/detector/image', Image, self.get_image)
        rospy.Subscriber('/detector/rois', RegionOfInterestArray, self.send_roi)

        rospy.init_node('commander')

        robo_ref = db.reference('/robot/0')
        robo_ref.get().get('running', False)

        self.connection = robo_ref.listen(self.robot_command)

    def robot_command(self, resp):
        running = bool(resp.data.get('running'))
        self.pub_motor.publish(running)

    def send_roi(self, resp):
        print('????', resp.data)
        db.reference('/roi/0').push({
            "time": time(),
            "num": len(resp.data),
        })

    def get_image(self, resp):
        try:
            cv_image = bridge.imgmsg_to_cv2(resp, "bgr8")
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

    def run(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()
 
        self.close()

    def close(self):
        self.connection.close()


if __name__ == "__main__":
    c = Commander()
    try:
        c.run()
    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
        c.close()

    rospy.spin()
