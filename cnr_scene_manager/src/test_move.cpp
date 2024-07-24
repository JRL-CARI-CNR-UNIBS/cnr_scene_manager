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

  CNR_INFO(logger,"Waiting for /get_planning_scene");
  ros::service::waitForService("/get_planning_scene");

  CNR_INFO(logger,"Waiting for /cnr_scene_manager/move_objects server");
  ros::service::waitForService("/cnr_scene_manager/move_objects");

  auto get_plannning_scene = n.serviceClient<moveit_msgs::GetPlanningScene        >("/get_planning_scene"              ,true ); //persistent connection
  auto add_obj             = n.serviceClient<cnr_scene_manager_msgs::AddObjects   >("/cnr_scene_manager/add_objects"   ,false);
  auto move_obj            = n.serviceClient<cnr_scene_manager_msgs::MoveObjects  >("/cnr_scene_manager/move_objects"  ,true ); //persistent connection
  auto remove_obj          = n.serviceClient<cnr_scene_manager_msgs::RemoveObjects>("/cnr_scene_manager/remove_objects",false);

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

  ros::WallDuration(5.0).sleep();

  std::string what;

  size_t n_objs = 1;
  size_t n_objs_default = 1;

  if(not cnr::param::has("/cnr_scene_manager_test_move/n_objs",what))
    CNR_ERROR(logger,"cnr_scene_manager_test_move/n_obj not available:\n"<<what);
  else
    if(not cnr::param::get("/cnr_scene_manager_test_move/n_objs",n_objs,what,n_objs_default,true))
      CNR_ERROR(logger,"Error retrieving cnr_scene_manager_test_move/n_obj:\n"<<what);

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
  obj.object_type = "moving_sphere";
  obj.pose = pose_msg;

  cnr_scene_manager_msgs::AddObjects add_srv;
  add_srv.request.objects = std::vector<cnr_scene_manager_msgs::Object>(n_objs,obj);

  if(not add_obj.call(add_srv))
  {
    CNR_ERROR(logger,"call to add_obj srv not ok");
    return 1;
  }

  std::vector<std::string> obj_id;

  if(not add_srv.response.success)
  {
    CNR_ERROR(logger,"Adding objects failed");
    return 1;
  }
  else
  {
    for(const auto id:add_srv.response.ids)
      CNR_INFO(logger,"Object added with id %s",id.c_str());
  }

  CNR_INFO(logger,"Start moving objects");

  std::random_device rseed;
  std::mt19937 gen(rseed());
  std::uniform_real_distribution<double> rd(-1.0,1.0);

  size_t n_step = 0;
  double dt = 0.010;

  std::vector<std::vector<double>> vel(n_objs,{0.0,0.0,0.0});
  std::vector<geometry_msgs::Pose> poses(n_objs,pose_msg.pose);

  cnr_scene_manager_msgs::MoveObjects move_srv;
  move_srv.request.poses = poses;
  move_srv.request.obj_ids = add_srv.response.ids;

  ros::WallRate lp(1.0/dt);
  ros::WallTime tic, tic_cycle;
  double time_move,time_pln_scn;
  while(ros::ok())
  {
    tic_cycle = ros::WallTime::now();

    if(n_step>10)
      n_step = 0;

    for(size_t i=0;i<n_objs;i++)
    {
      if(n_step == 0)
        vel[i] = {rd(gen),rd(gen),rd(gen)};

      poses[i].position.x = poses[i].position.x+vel[i][0]*dt;
      poses[i].position.y = poses[i].position.y+vel[i][1]*dt;
      poses[i].position.z = poses[i].position.z+vel[i][2]*dt;

      move_srv.request.poses[i] = poses[i];
    }

    tic = ros::WallTime::now();
    if(not move_obj.call(move_srv))
    {
      CNR_ERROR(logger,"call to move_obj srv not ok");
    }
    time_move = (ros::WallTime::now()-tic).toSec()*1000.0;

    moveit_msgs::GetPlanningScene ps_srv;
    tic = ros::WallTime::now();
    if(not get_plannning_scene.call(ps_srv))
    {
      CNR_ERROR(logger,"call to get_plannning_scene srv not ok");
    }
    time_pln_scn = (ros::WallTime::now()-tic).toSec()*1000.0;

    n_step++;

    CNR_INFO(logger,"Time move call "<<time_move<<"ms\tTime pln scn "<<time_pln_scn<<"ms\tTime cycle "<<(ros::WallTime::now()-tic_cycle).toSec()*1000.0<<"ms");

    lp.sleep();
  }

  return 0;
}
