
int main(int argc, char** argv) {
  ros::init(argc, argv, "motor_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  // read params
  int start_btn_gpio = 481;
  int rate = 2;

  ros::Publisher start_pub =
          nh.advertise<std_msgs::Bool>("/gpio/write", 4);

  ros::Rate r(rate);
  std_msgs::Bool start_button;

  while (ros::ok()) {
    gpioGetValue(start_btn_gpio, &mission_start);

    start_button.data = static_cast<unsigned char>(not mission_start);
    start_pub.publish(start_button);

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}