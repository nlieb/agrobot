from firebase import firebase
import firebase_admin
from firebase_admin import credentials, db

from time import sleep
import cv2
import os
import random
import base64
from time import time

import rospy

class FirebaseApp():
    def __init__(self):
        cred = credentials.Certificate("secret.json")
        self.app = firebase_admin.initialize_app(
            credential=cred,
            options={
                'databaseURL' : 'https://agribot-2019.firebaseio.com/',
            }
        )

        robo_ref = db.reference('/robot/0')
        robo_ref.get().get('running', False)

        self.connection = robo_ref.listen(self.robot_command)

    def robot_command(self, test):
        running = test.data.get('running')
        if running:
            self.send_run()
        else:
            self.send_stop()

    def send_run(self):
        print('Send Run') # TODO: CHANGE THIS!!!

    def send_stop(self):
        print('Send Stop') # TODO: CHANGE THIS!!!

    def send_roi(self, num):
        db.reference('/roi/0').push({
            "time": time(),
            "num": num,
        })

    def sketch_run(self):
        img_path = 'imgs'
        imgs = sorted(os.path.join(img_path, img) for img in os.listdir(img_path))
        for img in imgs:
            size = random.randint(0, 5)
            mat_img = cv2.imread(img)

            self.send_roi(size)
            self.send_image(mat_img)
            sleep(.5)

        self.close()

    def send_image(self, mat_img):
        _, buf = cv2.imencode('.jpg', mat_img)
        encoded = "data:image/jpg;base64," + base64.b64encode(buf)

        db.reference('/video/0').set({
            "image": encoded,
        })

    def update(self, mat_img, num):
        self.send_image(mat_img)
        self.send_roi(num)

    def run(self):
        while True:
            sleep(.5)
        self.close()

    def close(self):
        self.connection.close()


if __name__ == "__main__":
    f = FirebaseApp()
    try:
        f.sketch_run()
    except KeyboardInterrupt:
        f.close()

    rospy.spin()
