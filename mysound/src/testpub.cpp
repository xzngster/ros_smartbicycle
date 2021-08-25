#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include <vector>

int main(int argc, char **argv)
{
  std::vector<int> data(5,0);
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Int32MultiArray>("direction", 1000);
  ros::Rate loop_rate(8);
  std::cout << "node start!!!\n";
  char go;
  while (ros::ok())
  {
    char ch;
    scanf("%c", &ch);
    ROS_INFO("publishing...\n");
    std_msgs::Int32MultiArray msg;
    data[1] = 2;
    msg.data = data;
    chatter_pub.publish(msg);
    loop_rate.sleep();
  }
  ros::spinOnce();
  return 0;
}