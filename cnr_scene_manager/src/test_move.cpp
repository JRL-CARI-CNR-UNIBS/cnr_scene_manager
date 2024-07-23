#include <random>
#include <tf/tf.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <geometric_shapes/shape_operations.h>

#include <cnr_param/cnr_param.h>
#include <cnr_logger/cnr_logger.h>
#include <cnr_tf_named_object_loader/cnr_tf_named_object_loader.h>

#include <cnr_scene_manager_msgs/AddObjects.h>
#include <cnr_scene_manager_msgs/MoveObjects.h>
#include <cnr_scene_manager_msgs/RemoveObjects.h>

using namespace  cnr_tf_named_object_loader;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cnr_scene_manager_test_move");
  ros::NodeHandle n;

  std::string package_name = "cnr_scene_manager";
  std::string package_path = ros::package::getPath(package_name);

  if(package_path.empty())
    throw std::invalid_argument("Failed to get path for package!");

  std::string logger_file = package_path+"/config/logger_param.yaml";
  cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("cnr_scene_manager_logger",logger_file);

  auto add_obj    = n.serviceClient<cnr_scene_manager_msgs::AddObjects   >("/cnr_scene_manager/add_objects"   );
  auto move_obj   = n.serviceClient<cnr_scene_manager_msgs::MoveObjects  >("/cnr_scene_manager/move_objects"  );
  auto remove_obj = n.serviceClient<cnr_scene_manager_msgs::RemoveObjects>("/cnr_scene_manager/remove_objects");
  auto get_plannning_scene= n.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  if(not add_obj.waitForExistence(ros::Duration(10)))
  {
    CNR_ERROR(logger,"unable to connect to /cnr_scene_manager/add_objects");
    return 1;
  }
  if(not move_obj.waitForExistence(ros::Duration(10)))
  {
    CNR_ERROR(logger,"unable to connect to /cnr_scene_manager/move_obj");
    return 1;
  }
  if(not remove_obj.waitForExistence(ros::Duration(10)))
  {
    CNR_ERROR(logger,"unable to connect to /cnr_scene_manager/remove_obj");
    return 1;
  }
  if(not get_plannning_scene.waitForExistence(ros::Duration(10)))
  {
    CNR_ERROR(logger,"unable to connect to /get_plannning_scene");
    return 1;
  }

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.pose.position.x = 0;
  pose_msg.pose.position.y = 0;
  pose_msg.pose.position.z = 0;
  pose_msg.pose.orientation.x = 0;
  pose_msg.pose.orientation.y = 0;
  pose_msg.pose.orientation.z = 0;
  pose_msg.pose.orientation.w = 1;

  pose_msg.header.frame_id = "world";
  pose_msg.header.stamp = ros::Time::now();

  cnr_scene_manager_msgs::Object obj;
  obj.object_type = "sphere_obj";
  obj.pose = pose_msg;

  cnr_scene_manager_msgs::AddObjects add_srv;

  add_srv.request.objects.push_back(obj);

  if(not add_obj.call(add_srv))
  {
    CNR_ERROR(logger,"call to add_obj srv not ok");
    return 1;
  }

  std::string obj_id;

  if(not add_srv.response.success)
  {
    CNR_ERROR(logger,"Adding the object failed");
    return 1;
  }
  else
  {
    CNR_ERROR(logger,"Object added with id %s",obj_id.c_str());
  }

  CNR_INFO(logger,"Start moving the object");

  std::random_device rseed;
  std::mt19937 gen(rseed());
  std::uniform_real_distribution<double> rd(-1.0,1.0);

  size_t n_step = 0;
  double dt = 0.010;
  std::vector<double> vel = {0.0,0.0,0.0};

  ros::WallRate lp(1.0/dt);
  ros::WallTime tic, toc;
  double time_move,time_pln_scn;
  while(ros::ok())
  {
    if(n_step == 10)
    {
      vel = {rd(gen),rd(gen),rd(gen)};
      n_step = 0;
    }

    pose_msg.pose.position.x = pose_msg.pose.position.x+vel[0]*dt;
    pose_msg.pose.position.y = pose_msg.pose.position.y+vel[1]*dt;
    pose_msg.pose.position.z = pose_msg.pose.position.z+vel[2]*dt;

    cnr_scene_manager_msgs::MoveObjects move_srv;
    move_srv.request.obj_ids.push_back(obj_id);
    move_srv.request.poses.push_back(pose_msg.pose);

    tic = ros::WallTime::now();
    if(not move_obj.call(move_srv))
    {
      CNR_ERROR(logger,"call to move_obj srv not ok");
    }
    toc = ros::WallTime::now();
    time_move = (toc-tic).toSec();

    moveit_msgs::GetPlanningScene ps_srv;
    tic = ros::WallTime::now();
    if(not get_plannning_scene.call(ps_srv))
    {
      CNR_ERROR(logger,"call to get_plannning_scene srv not ok");
    }
    toc = ros::WallTime::now();
    time_pln_scn = (toc-tic).toSec();

    CNR_INFO(logger,"Time move call "<<time_move<<"\tTime pln scn "<<time_pln_scn);
    n_step++;

    lp.sleep();
  }

  return 0;
}
