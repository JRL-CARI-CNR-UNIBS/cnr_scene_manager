#include <random>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <geometric_shapes/shape_operations.h>

#include <cnr_tf_named_object_loader/cnr_tf_named_object_loader.h>

#include <cnr_scene_manager_msgs/msg/object.hpp>
#include <cnr_scene_manager_msgs/srv/add_objects.hpp>
#include <cnr_scene_manager_msgs/srv/move_objects.hpp>
#include <cnr_scene_manager_msgs/srv/remove_objects.hpp>
#include <rclcpp/qos.hpp>
using namespace  cnr_tf_named_object_loader;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto n = rclcpp::Node::make_shared("cnr_scene_manager_test_move");

  auto add_client = n->create_client<cnr_scene_manager_msgs::srv::AddObjects>("/cnr_scene_manager/add_objects");
  auto move_client = n->create_client<cnr_scene_manager_msgs::srv::MoveObjects>("/cnr_scene_manager/move_objects");
  auto remove_client = n->create_client<cnr_scene_manager_msgs::srv::RemoveObjects>("/cnr_scene_manager/remove_objects");

  auto get_plannning_scene = n->create_client<moveit_msgs::srv::GetPlanningScene        >("/get_planning_scene");
  auto add_obj             = n->create_client<cnr_scene_manager_msgs::srv::AddObjects   >("/cnr_scene_manager/add_objects"   );
  auto move_obj            = n->create_client<cnr_scene_manager_msgs::srv::MoveObjects  >("/cnr_scene_manager/move_objects"  );
  auto remove_obj          = n->create_client<cnr_scene_manager_msgs::srv::RemoveObjects>("/cnr_scene_manager/remove_objects");

  if(not add_obj->wait_for_service(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR_STREAM(n->get_logger(),"unable to connect to /cnr_scene_manager/add_objects");
    return 1;
  }
  if(not move_obj->wait_for_service(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR_STREAM(n->get_logger(),"unable to connect to /cnr_scene_manager/move_obj");
    return 1;
  }
  if(not remove_obj->wait_for_service(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR_STREAM(n->get_logger(),"unable to connect to /cnr_scene_manager/remove_obj");
    return 1;
  }
  if(not get_plannning_scene->wait_for_service(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR_STREAM(n->get_logger(),"unable to connect to /get_plannning_scene");
    return 1;
  }

  rclcpp::sleep_for(std::chrono::seconds(5));

  int64_t n_objs_param = n->declare_parameter<int64_t>("n_objs", 1);
  if(n_objs_param < 1)
  {
    RCLCPP_ERROR_STREAM(n->get_logger(),"n_objs should be positive, it is "<<n_objs_param);
    return 1;
  }
  size_t n_objs = static_cast<size_t>(n_objs_param);

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.pose.position.x = 0;
  pose_msg.pose.position.y = 0;
  pose_msg.pose.position.z = 0;
  pose_msg.pose.orientation.x = 0;
  pose_msg.pose.orientation.y = 0;
  pose_msg.pose.orientation.z = 0;
  pose_msg.pose.orientation.w = 1;

  pose_msg.header.frame_id = "world";
  pose_msg.header.stamp = n->get_clock()->now();

  cnr_scene_manager_msgs::msg::Object obj;
  obj.object_type = "moving_sphere";
  obj.pose = pose_msg;

  auto add_req = std::make_shared<cnr_scene_manager_msgs::srv::AddObjects::Request>();
  add_req->objects = std::vector<cnr_scene_manager_msgs::msg::Object>(n_objs,obj);

  auto result = add_client->async_send_request(add_req);
  if (rclcpp::spin_until_future_complete(n, result)!=rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(n->get_logger(),"Call to srv not ok");
    return 1;
  }

  std::vector<std::string> obj_id;

  auto future = result.get();
  if(not future->success)
  {
    RCLCPP_ERROR_STREAM(n->get_logger(),"Adding objects failed");
    return 1;
  }
  else
  {
    for(const auto& id:future->ids)
      RCLCPP_INFO_STREAM(n->get_logger(),"Object added with id "<<id);
  }

  RCLCPP_INFO_STREAM(n->get_logger(),"Start moving objects");

  std::random_device rseed;
  std::mt19937 gen(rseed());
  std::uniform_real_distribution<double> rd(-1.0,1.0);

  size_t n_step = 0;
  double dt = 0.010;

  std::vector<std::vector<double>> vel(n_objs,{0.0,0.0,0.0});
  std::vector<geometry_msgs::msg::Pose> poses(n_objs,pose_msg.pose);

  auto move_req = std::make_shared<cnr_scene_manager_msgs::srv::MoveObjects::Request>();
  move_req->poses = poses;
  move_req->obj_ids = future->ids;

  rclcpp::WallRate  lp(1.0/dt);
  double time_move,time_pln_scn,time_cycle;
  while(rclcpp::ok())
  {
    auto tic_cycle = n->get_clock()->now();

    if(n_step>10)
      n_step = 0;

    for(size_t i=0;i<n_objs;i++)
    {
      if(n_step == 0)
        vel[i] = {rd(gen),rd(gen),rd(gen)};

      poses[i].position.x = poses[i].position.x+vel[i][0]*dt;
      poses[i].position.y = poses[i].position.y+vel[i][1]*dt;
      poses[i].position.z = poses[i].position.z+vel[i][2]*dt;

      move_req->poses[i] = poses[i];
    }

    auto tic = n->get_clock()->now();

    auto result = move_client->async_send_request(move_req);
    if (rclcpp::spin_until_future_complete(n, result)!=rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(n->get_logger(),"Call to srv not ok");
      return 1;
    }

    auto future = result.get();
    if(not future->success)
    {
      RCLCPP_ERROR_STREAM(n->get_logger(),"Moving objects failed");
      return 1;
    }
    auto time = (n->get_clock()->now()-tic);
    time_move = time.seconds()*1000.0+time.nanoseconds()/1000000.0;

    auto ps_req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    tic = n->get_clock()->now();
    auto result_ps = get_plannning_scene->async_send_request(ps_req);
    if (rclcpp::spin_until_future_complete(n, result_ps)!=rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(n->get_logger(),"Call to srv not ok");
      return 1;
    }

    auto future_ps = result_ps.get();
    if(not future_ps)
    {
      RCLCPP_ERROR_STREAM(n->get_logger(),"Failed getting planning scene msg");
      return 1;
    }
    time = (n->get_clock()->now()-tic);
    time_pln_scn = time.seconds()*1000.0+time.nanoseconds()/1000000.0;

    n_step++;

    time = (n->get_clock()->now()-tic_cycle);
    time_cycle = time.seconds()*1000.0+time.nanoseconds()/1000000.0;
    RCLCPP_INFO_STREAM(n->get_logger(),"Time move call "<<time_move<<"ms\tTime pln scn "<<time_pln_scn<<"ms\tTime cycle "<<time_cycle<<"ms");

    lp.sleep();
  }

  return 0;
}
