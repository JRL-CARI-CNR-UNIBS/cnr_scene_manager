#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <geometric_shapes/shape_operations.h>

#include <cnr_param/cnr_param.h>
#include <cnr_logger/cnr_logger.h>
#include <cnr_tf_named_object_loader/cnr_tf_named_object_loader.h>

#include <cnr_scene_manager_msgs/msg/object.hpp>
#include <cnr_scene_manager_msgs/srv/add_objects.hpp>
#include <cnr_scene_manager_msgs/srv/move_objects.hpp>
#include <cnr_scene_manager_msgs/srv/remove_objects.hpp>

using namespace  cnr_tf_named_object_loader;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto n = rclcpp::Node::make_shared("cnr_scene_manager_test");

  auto add_client = n->create_client<cnr_scene_manager_msgs::srv::AddObjects>("/cnr_scene_manager/add_objects");
  auto move_client = n->create_client<cnr_scene_manager_msgs::srv::MoveObjects>("/cnr_scene_manager/move_objects");
  auto remove_client = n->create_client<cnr_scene_manager_msgs::srv::RemoveObjects>("/cnr_scene_manager/remove_objects");

  cnr_scene_manager_msgs::msg::Object obj_msg;
  obj_msg.object_type = "box_obj";

  obj_msg.pose.header.frame_id = "world";
  obj_msg.pose.header.stamp = n->get_clock()->now();

  obj_msg.pose.pose.position.x = 1;
  obj_msg.pose.pose.position.y = 1;
  obj_msg.pose.pose.position.z = 1;

  obj_msg.pose.pose.orientation.x = 0;
  obj_msg.pose.pose.orientation.y = 0;
  obj_msg.pose.pose.orientation.z = 0;
  obj_msg.pose.pose.orientation.w = 1;

  auto add_request = std::make_shared<cnr_scene_manager_msgs::srv::AddObjects::Request>();
  add_request->objects.push_back(obj_msg);
  add_request->timeout = 1.0;

  while (!add_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(n->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(n->get_logger(), "service not available, waiting again...");
  }

  auto add_result = add_client->async_send_request(add_request);

  std::vector<std::string> ids;

  if (rclcpp::spin_until_future_complete(n, add_result) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    ids = add_result.future.get()->ids;
    for(const auto& s:ids)
      RCLCPP_INFO(n->get_logger(), "Id: %s", s.c_str());
  } else {
    RCLCPP_ERROR(n->get_logger(), "Failed to call service add_objects");
  }

  rclcpp::sleep_for(std::chrono::seconds(5));

  auto move_request = std::make_shared<cnr_scene_manager_msgs::srv::MoveObjects::Request>();
  move_request->obj_ids = ids;
  move_request->timeout = 1.0;

  geometry_msgs::msg::Pose pose = obj_msg.pose.pose;
  pose.position.x = pose.position.x+2;

  move_request->poses.push_back(pose);

  while (!move_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(n->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(n->get_logger(), "service not available, waiting again...");
  }

  auto move_result = move_client->async_send_request(move_request);

  if (rclcpp::spin_until_future_complete(n, move_result) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(n->get_logger(), "Moved!");
  } else {
    RCLCPP_ERROR(n->get_logger(), "Failed to call service move_objects");
  }

  rclcpp::sleep_for(std::chrono::seconds(5));

  auto remove_request = std::make_shared<cnr_scene_manager_msgs::srv::RemoveObjects::Request>();
  remove_request->obj_ids = ids;
  remove_request->timeout = 1.0;

  while (!remove_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(n->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(n->get_logger(), "service not available, waiting again...");
  }

  auto remove_result = remove_client->async_send_request(remove_request);

  if (rclcpp::spin_until_future_complete(n, remove_result) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(n->get_logger(), "Removed!");
  } else {
    RCLCPP_ERROR(n->get_logger(), "Failed to call service remove_objects");
  }

  return 0;
}
