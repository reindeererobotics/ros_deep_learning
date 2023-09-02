// create a ros2 node that will compress raw images to compressed images using the image_transport library

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "image_transport/image_transport.hpp"
// #include "image_transport/publisher.hpp"
#include "image_transport/raw_publisher.hpp"
#include "image_transport/raw_subscriber.hpp"
#include "image_transport/camera_common.hpp"
#include "image_transport/publisher_plugin.hpp"
#include "image_transport/subscriber_plugin.hpp"
// #include "image_transport/compression_common.hpp"
// #include "image_transport/compressed_subscriber.hpp"
// #include "image_transport/compressed_publisher.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;


sensor_msgs::msg::CompressedImage::SharedPtr comp_img;

void subcallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
  // subscribe, compress, and assign the raw img msg data to compressed img
  comp_img->format= 'jpeg';
  comp_img->data=msg->data;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("node_video_transport", options);

  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub = it.subscribe("/cam0/video_source0", 3, &subcallback);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
