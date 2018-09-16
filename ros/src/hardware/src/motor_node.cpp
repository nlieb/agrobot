
#include <ros/ros.h>
#include <jhpwmpca9685.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>


PCA9685 *pca9685 = new PCA9685();

void motorStartCb(const std_msgs::Bool &msg) {
  printf("base speed cb\n");

  if(msg.data) {
    pca9685->setPulseLength(14, 350);
    pca9685->setPulseLength(15, 350);
  } else {
    pca9685->setPulseLength(14, 0);
    pca9685->setPulseLength(15, 0);
  }
}

void motorLateralCb(const std_msgs::Bool &msg) {
  printf("base speed cb\n");

  if(msg.data) {
    pca9685->setPulseLength(2, 350);
    pca9685->setPulseLength(1, 350);
  } else {
    pca9685->setPulseLength(0, 0);
    pca9685->setPulseLength(1, 0);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "motor_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  int err = pca9685->openPCA9685();
  if (err < 0) {
    ROS_ERROR("Error: %d", pca9685->error);
  }

  ROS_DEBUG("PCA9685 Device Address: 0x%02X\n", pca9685->kI2CAddress);
  pca9685->setAllPWM(0, 0);
  pca9685->reset();
  pca9685->setPWMFrequency(200);
  pca9685->setAllPWM(0, 0);

  ros::Subscriber start_sub =
          nh.subscribe("motor/start", 1, &motorStartCb);
  ros::Subscriber lateral_sub =
          nh.subscribe("motor/lateral", 1, &motorLateralCb);

  ros::Rate r(30);
  std_msgs::Bool start_button;

  while (ros::ok()) {
//    start_pub.publish(start_button);

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}