#include <tf/tf.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <geometric_shapes/shape_operations.h>

#include <cnr_param/cnr_param.h>
#include <cnr_logger/cnr_logger.h>
#include <cnr_tf_named_object_loader/cnr_tf_named_object_loader.h>

#include <cnr_scene_manager_msgs/AddObjects.h>
#include <cnr_scene_manager_msgs/RemoveObjects.h>


using namespace  cnr_tf_named_object_loader;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cnr_scene_manager_test");
  ros::NodeHandle n;

  std::shared_ptr<TFNamedObjectsManager> scene_manager = std::make_shared<TFNamedObjectsManager>();

  object_t obj;
  obj.id = "obj_test";
  obj.header.frame_id = "world";
  obj.header.stamp = ros::Time::now();

  obj.pose.position.x = 1;
  obj.pose.position.y = 1;
  obj.pose.position.z = 1;

  obj.pose.orientation.x = 0;
  obj.pose.orientation.y = 0;
  obj.pose.orientation.z = 0;
  obj.pose.orientation.w = 1;

  shape_msgs::SolidPrimitive shape;
  shape.type = shape_msgs::SolidPrimitive::SPHERE;
  shape.dimensions.resize(1); shape.dimensions[0] = 0.5;

  std::vector<shape_msgs::SolidPrimitive> primitive_v;
  primitive_v.push_back(shape);

  obj.primitives = primitive_v;

  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = 0;
  pose_msg.position.y = 0;
  pose_msg.position.z = 0;
  pose_msg.orientation.x = 0;
  pose_msg.orientation.y = 0;
  pose_msg.orientation.z = 0;
  pose_msg.orientation.w = 1;

  obj.primitive_poses.clear();
  obj.primitive_poses.push_back(pose_msg);

  obj.operation = obj.ADD;

  std::string w;
  tf_named_objects_t tf_objects;
  tf_objects.emplace_back(obj);

  if(not scene_manager->addNamedTFObjects(tf_objects,0.1,w))
    ROS_ERROR_STREAM("error: \n"<<w);

  return 0;
}
