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
  obj.pose.position.x = obj.pose.position.y = obj.pose.position.z = 1;
  obj.pose.orientation.x = obj.pose.orientation.y = obj.pose.orientation.z = 0;
  obj.pose.orientation.w = 1;
  obj.operation = obj.ADD;

  std::vector<shape_msgs::SolidPrimitive> prim;
  shape_msgs::SolidPrimitive sf;
  sf.type = shape_msgs::SolidPrimitive::SPHERE;
  sf.dimensions.resize(1); sf.dimensions[0] = 2;
  prim.push_back(sf);

  obj.primitives = prim;
  geometry_msgs::Pose pose_msg;

  std::string w;
  tf_named_objects_t t;
  t.emplace_back(obj);
  scene_manager->addNamedTFObjects(t,10,w);

  ros::spin();

  return 0;
}
