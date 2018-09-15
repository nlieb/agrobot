from firebase import firebase
import firebase_admin
from firebase_admin import credentials, db

from time import sleep
import cv2
import os
import random
from time import time

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
        while True:
            size = random.randint(0, 5)
            self.send_roi(size)
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
