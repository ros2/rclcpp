#include "rclcpp/Node.h"
#include "rclcpp/Publisher.h"
#include "std_msgs/Int32.h"


main(int argc, char** argv)
{
  rclcpp::Node* n = create_node();
  rclcpp::Publisher<std_msgs::Int32>* p = n->create_publisher<std_msgs::Int32>("topic_name");
  std_msgs::Int32 ros_msg;
  ros_msg.data = 23;
  p->publish(ros_msg);
  ros_msg.data = 42;
  p->publish(ros_msg);
  return 0;
}
