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

#define OBJS_NS "/object_geometries"
#define SCENE_NS "/scene_objects"
#define ADD_OBJ_SERVICE "add_objects"
#define REMOVE_OBJ_SERVICE "remove_objects"

using namespace  cnr_tf_named_object_loader;
using color_t = moveit_msgs::ObjectColor;

/**
 * @brief The object_type_t struct represents a type of object that can be loaded into the scene
 */
struct object_type_t
{
public:
  bool use_mesh_;
  color_t color_;
  std::string id_;
  shape_msgs::Mesh mesh_;                //if mesh, no primitive
  shape_msgs::SolidPrimitive primitive_; //if primitive, no mesh
  Eigen::Affine3d rt_matrix_;            // roto-translation matrix of the mesh/primitive respect to the object reference frame
};

/**
 * @brief The registered_object_t struct associates an object type with a counter of published objects.
 *  The counter is helpful to give unique names to the objects. Note that when an object is deleted, the counter is not decremented to avoid inconsistent behaviors.
 */
struct registered_object_t
{
public:
  object_type_t obj_;
  unsigned int count_ = 0;
};

/**
 * @brief logger_ cnr_logger::TraceLogger for logging
 */
cnr_logger::TraceLoggerPtr logger_;

/**
 * @brief registered_objects_ map of registered object types
 */
std::map<std::string,registered_object_t> registered_object_types_;

/**
 * @brief scene_manager_ is the cnr_tf_named_object_loader::TFNamedObjectsManager which manages the add, remove, move services in the scene
 */
std::shared_ptr<cnr_tf_named_object_loader::TFNamedObjectsManager> scene_manager_;

/**
 * @brief register_type registers a single object type given the YAML::Node
 * @param yaml_node the YAML::Node from which the object type configuration is retrieved
 * @param id the name to give to the object to register
 * @param r_obj the registered object
 * @return true if everything went well, false otherwise
 */
bool register_type(const YAML::Node& yaml_node, const std::string& id, registered_object_t& r_obj)
{
  r_obj.count_ = 0;
  r_obj.obj_.id_ = id;

  /********************* COLOR *****************************/

  std::vector<double> rgba{255,255,255,1};
  if(yaml_node["color"])
  {
    try {
      rgba = yaml_node["color"].as<std::vector<double>>();
    } catch (const YAML::BadConversion& e) {
      CNR_ERROR(logger_,"color should be an array of 4 double (r,g,b,alpha)");
      return false;
    }

    if(rgba.size() != 4)
    {
      CNR_ERROR(logger_,"color should be an array of 4 double (r,g,b,alpha)");
      return false;
    }
  }

  r_obj.obj_.color_.id = id;
  r_obj.obj_.color_.color.r = rgba.at(0);
  r_obj.obj_.color_.color.g = rgba.at(1);
  r_obj.obj_.color_.color.b = rgba.at(2);
  r_obj.obj_.color_.color.a = rgba.at(3);

  /********************* RT-MATRIX *****************************/

  r_obj.obj_.rt_matrix_ .setIdentity();
  if(yaml_node["offset_quaternion"])
  {
    std::vector<double> offset_quaternion;
    try {
      offset_quaternion = yaml_node["offset_quaternion"].as<std::vector<double>>();
    } catch (const YAML::BadConversion& e) {
      CNR_ERROR(logger_,"offset_quaternion should be an array of 4 double (i,j,k,w)");
      return false;
    }

    if(offset_quaternion.size() != 4)
    {
      CNR_ERROR(logger_,"offset_quaternion should be an array of 4 double (i,j,k,w)");
      return false;
    }

    Eigen::Quaterniond q(offset_quaternion.at(3),offset_quaternion.at(0),offset_quaternion.at(1),offset_quaternion.at(2));
    q.normalize();
    r_obj.obj_.rt_matrix_=q;
  }

  if(yaml_node["offset"])
  {
    std::vector<double> offset;
    try {
      offset = yaml_node["offset"].as<std::vector<double>>();
    } catch (const YAML::BadConversion& e) {
      CNR_ERROR(logger_,"offset should be an array of 3 double (x,y,z)");
      return false;
    }

    if(offset.size() != 3)
    {
      CNR_ERROR(logger_,"offset should be an array of 3 double (x,y,z)");
      return false;
    }

    r_obj.obj_.rt_matrix_.translation()(0)=offset.at(0);
    r_obj.obj_.rt_matrix_.translation()(1)=offset.at(1);
    r_obj.obj_.rt_matrix_.translation()(2)=offset.at(2);
  }

  /********************* MESH/PRIMITIVE *****************************/
  r_obj.obj_.use_mesh_ = false;

  if(yaml_node["mesh"])
  {
    std::string path;
    try {
      path = yaml_node["mesh"].as<std::string>();
    } catch (const YAML::BadConversion& e) {
      CNR_ERROR(logger_,"mash has to be a string (path to mesh)");
      return false;
    }

    Eigen::Vector3d scale = Eigen::Vector3d(1,1,1);
    if(yaml_node["scale"])
    {
      std::vector<double> scale_v;
      try {
        scale_v = yaml_node["scale"].as<std::vector<double>>();
      } catch (const YAML::BadConversion& e) {
        CNR_ERROR(logger_,"scale should be an array of 3 double");
        return false;
      }

      if(scale_v.size() != 3)
      {
        CNR_ERROR(logger_,"scale should be an array of 3 double");
        return false;
      }
      scale(0)=scale_v.at(0);
      scale(1)=scale_v.at(1);
      scale(2)=scale_v.at(2);
    }

    shapes::Mesh* m = shapes::createMeshFromResource(path,scale);

    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m,mesh_msg);

    r_obj.obj_.mesh_ = boost::get<shape_msgs::Mesh>(mesh_msg);
    r_obj.obj_.use_mesh_ = true;

    return true;
  }
  else if(yaml_node["box"])
  {
    std::vector<double> size;
    try {
      size = yaml_node["box"].as<std::vector<double>>();
    } catch (const YAML::BadConversion& e) {
      CNR_ERROR(logger_,"box should be an array of 3 double");
      return false;
    }

    if(size.size() != 3)
    {
      CNR_ERROR(logger_,"box should be an array of 3 double");
      return false;
    }

    r_obj.obj_.primitive_.type = shape_msgs::SolidPrimitive::BOX;
    r_obj.obj_.primitive_.dimensions.resize(3);
    r_obj.obj_.primitive_.dimensions[0] = size.at(0);
    r_obj.obj_.primitive_.dimensions[1] = size.at(1);
    r_obj.obj_.primitive_.dimensions[2] = size.at(2);

    return true;
  }
  else if(yaml_node["sphere"])
  {
    double radius;
    try {
      radius = yaml_node["sphere"].as<double>();
    } catch (const YAML::BadConversion& e) {
      CNR_ERROR(logger_,"sphere should be one double representing the radius");
      return false;
    }

    r_obj.obj_.primitive_.type = shape_msgs::SolidPrimitive::SPHERE;
    r_obj.obj_.primitive_.dimensions.resize(1);
    r_obj.obj_.primitive_.dimensions[0] = radius;

    return true;
  }
  else
  {
    CNR_ERROR(logger_,"Object geometry not recognized");
    return false;
  }
}

/**
 * @brief register_object_types registers the object types listed under the param_ns/OBJS_NS parameter
 * @param param_ns the cnr::param namespace under which OBJS_NS is defined
 * @return true if everything went well, false otherwise
 */
bool register_object_types(const std::string& param_ns)
{
  std::string w;
  std::string ns = param_ns+OBJS_NS;
  if(cnr::param::has(ns,w))
  {
    YAML::Node objects_geometries;
    if(not cnr::param::get(ns,objects_geometries,w))
    {
      CNR_ERROR(logger_, "Cannot get objects geometries "<<ns<<"\n"<<w);
      return false;
    }

    if(not objects_geometries.IsMap())
    {
      CNR_ERROR(logger_, ns<<" should be a map");
      return false;
    }

    size_t dim;
    std::string id;
    for(YAML::const_iterator it=objects_geometries.begin();it != objects_geometries.end();++it)
    {
      registered_object_t r_obj;
      YAML::Node yaml_node = *it;

      id = it->first.as<std::string>();
      dim = registered_object_types_.size();

      if(register_type(yaml_node,id,r_obj))
      {
        registered_object_types_.insert(std::pair<std::string,registered_object_t>(r_obj.obj_.id_,r_obj));

        if(registered_object_types_.size() == dim)
          CNR_WARN(logger_, "Cannot add multiple objects with ID: "<<id);
      }
      else
        CNR_ERROR(logger_, "Cannot register object with ID: "<<id);
    }
  }
  else
  {
    CNR_ERROR(logger_, ns<<" not available\n"<<w);
    return false;
  }

  return true;
}

/**
 * @brief instantiate_registered_obj creates a Moveit collision object based on the type requested
 * @param it an interator to the element of the vector registered_objects_ pointing to the registered object type to instantiate
 * @param reference_frame the reference frame to which the object pose is defined
 * @param pose_msg a message containing the object pose
 * @param obj the Moveit collision object created
 * @return true if everything went well, false otherwise
 */
bool instantiate_registered_obj(const std::map<std::string,registered_object_t>::iterator& it, const std::string& reference_frame, const geometry_msgs::Pose& pose_msg, object_t& obj)
{
  if(it == registered_object_types_.end())
    return false;

  obj.id = it->first + "/n_" + std::to_string(it->second.count_);
  it->second.count_++; //increase the counter of published objects with this type

  obj.operation = object_t::ADD;
  obj.header.frame_id = reference_frame;

  Eigen::Affine3d rt_matrix_frame2obj;
  tf::poseMsgToEigen(pose_msg,rt_matrix_frame2obj);

  geometry_msgs::Pose absolute_pose_msg;
  Eigen::Affine3d rt_matrix_absolute = rt_matrix_frame2obj*it->second.obj_.rt_matrix_;

  tf::poseEigenToMsg(rt_matrix_absolute,absolute_pose_msg);

  if(it->second.obj_.use_mesh_)
  {
    obj.meshes.resize(1);
    obj.mesh_poses.resize(1);
    obj.meshes[0] = it->second.obj_.mesh_;
    obj.mesh_poses[0] = absolute_pose_msg;
  }
  else
  {
    obj.primitives.resize(1);
    obj.primitive_poses.resize(1);
    obj.primitives[0] = it->second.obj_.primitive_;
    obj.primitive_poses[0] = pose_msg;
  }

  return true;
}

/**
 * @brief load_scene_obj reads from a YAML::Node the attributes (type, fram, position, quaternion) to instantiate a registered obeject type
 * @param yaml_node the YAML::Node from which the attributes are retrieved
 * @param obj the Moveit collision object created
 * @return true if everything went well, false otherwise
 */
bool load_scene_obj(const YAML::Node& yaml_node, object_t& obj)
{
  std::string type;
  if(yaml_node["type"])
  {
    try {
      type = yaml_node["type"].as<std::string>();
    } catch (const YAML::BadConversion& e) {
      CNR_ERROR(logger_,"type field should be a string");
      return false;
    }
  }
  else
  {
    CNR_ERROR(logger_,"Each object to load into the scene should have the 'type' field");
    return false;
  }

  std::map<std::string,registered_object_t>::iterator it = registered_object_types_.find(type);
  if(it==registered_object_types_.end())
  {
    CNR_ERROR(logger_,"Object type %s not registered",type.c_str());
    return false;
  }

  std::string reference_frame;
  if(yaml_node["frame"])
  {
    try {
      reference_frame = yaml_node["frame"].as<std::string>();
    } catch (const YAML::BadConversion& e) {
      CNR_ERROR(logger_,"frame field should be a string");
      return false;
    }
  }
  else
  {
    CNR_ERROR(logger_,"Each object to load into the scene should have the 'frame' field");
    return false;
  }

  std::vector<double> position;
  if(yaml_node["position"])
  {
    try {
      position = yaml_node["position"].as<std::vector<double>>();
    } catch (const YAML::BadConversion& e) {
      CNR_ERROR(logger_,"position field should be a vector of 3 double (x,y,z)");
      return false;
    }

    if(position.size() != 3)
    {
      CNR_ERROR(logger_,"position field should be a vector of 3 double (x,y,z)");
      return false;
    }
  }
  else
  {
    CNR_ERROR(logger_,"Each object to load into the scene should have the 'position' field");
    return false;
  }

  std::vector<double> quaternion;
  if(yaml_node["quaternion"])
  {
    try {
      quaternion = yaml_node["quaternion"].as<std::vector<double>>();
    } catch (const YAML::BadConversion& e) {
      CNR_ERROR(logger_,"quaternion field should be a vector of 4 double (i,j,k,w)");
      return false;
    }

    if(quaternion.size() != 4)
    {
      CNR_ERROR(logger_,"quaternion field should be a vector of 4 double (i,j,k,w)");
      return false;
    }
  }
  else
  {
    CNR_ERROR(logger_,"Each object to load into the scene should have the 'quaternion' field");
    return false;
  }

  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = position.at(0);
  pose_msg.position.y = position.at(1);
  pose_msg.position.z = position.at(2);
  pose_msg.orientation.x = quaternion.at(0);
  pose_msg.orientation.y = quaternion.at(1);
  pose_msg.orientation.z = quaternion.at(2);
  pose_msg.orientation.w = quaternion.at(3);

  return instantiate_registered_obj(it,reference_frame,pose_msg,obj);
}

/**
 * @brief load_scene loads the objects listed under the param_ns/SCENE_NS parameter
 * @param param_ns the cnr::param namespace under which SCENE_NS is defined
 * @return true if everything went well, false otherwise
 */
bool load_scene(const std::string& param_ns)
{
  std::string w;
  std::string ns = param_ns+SCENE_NS;
  if(cnr::param::has(ns,w))
  {
    YAML::Node scene;
    if(not cnr::param::get(ns,scene,w))
    {
      CNR_ERROR(logger_, "Cannot get scene "<<ns<<"\n"<<w);
      return false;
    }

    if(not scene.IsSequence())
    {
      CNR_ERROR(logger_, ns<<" should be a sequence");
      return false;
    }

    objects_t scene_objs;
    for(YAML::iterator it = scene.begin(); it != scene.end(); ++it)
    {
      object_t scene_obj;
      if(load_scene_obj(*it,scene_obj))
        scene_objs.push_back(std::move(scene_obj));
      else
        CNR_ERROR(logger_,"Cannot load the %s-th object into the scene",std::distance(scene.begin(),it));
    }
  }
  else
  {
    CNR_ERROR(logger_, ns<<" not available\n"<<w);
    return false;
  }

  return true;
}

/**
 * @brief add callback for ADD_OBJ_SERVICE
 * @param req
 * @param res
 * @return
 */
bool add(cnr_scene_manager_msgs::AddObjects::Request& req,
         cnr_scene_manager_msgs::AddObjects::Response& res)
{
  CNR_INFO(logger_,"=================================================================");
  CNR_INFO(logger_,"==                ADD OBJECTS REQUEST RECEIVED!               ===");
  CNR_INFO(logger_,"=================================================================");

  res.ids.resize(req.objects.size());

  std::string what;
  std::map<std::string,registered_object_t>::iterator it;
  cnr_tf_named_object_loader::tf_named_objects_t objs_to_add;

  for(unsigned int i=0; req.objects.size(); i++)
  {
    it = registered_object_types_.find(req.objects[i].object_type);
    if(it==registered_object_types_.end())
    {
      CNR_ERROR(logger_,"Object type %s not registered",req.objects[i].object_type.c_str());
      res.success = false;
      return false;
    }

    object_t cobj;
    if(instantiate_registered_obj(it,req.objects[i].pose.header.frame_id,req.objects[i].pose.pose,cobj))
    {
      res.ids[i] = cobj.id;

      cobj.operation = object_t::ADD;
      objs_to_add.push_back(std::move(cobj));
    }
    else
    {
      CNR_ERROR(logger_,"Object type %s cannot be instantiated",req.objects[i].object_type.c_str());
      res.success = false;
      return false;
    }
  }

  if(not scene_manager_->addNamedTFObjects(objs_to_add, req.timeout, what))
  {
    CNR_ERROR(logger_,"Error in adding the objects to the scene\n%s", what.c_str());
    res.success = false;
    return false;
  }

  res.success = true;
  return true;
}

bool remove(cnr_scene_manager_msgs::RemoveObjects::Request& req,
            cnr_scene_manager_msgs::RemoveObjects::Response& res)
{
  CNR_INFO(logger_,"=================================================================");
  CNR_INFO(logger_,"==             REMOVE OBJECTS REQUEST RECEIVED!               ===");
  CNR_INFO(logger_,"=================================================================");

  std::string what;
  if(req.obj_ids.empty()) //no objects means reset the scene
  {
    if(not scene_manager_->resetScene(req.timeout,what))
    {
      CNR_ERROR(logger_,"Error in resetting the scene\n%s", what.c_str());
      res.success = false;
      return false;
    }
  }
  else
  {
    if(not scene_manager_->removeNamedObjects(req.obj_ids,req.timeout,what))
    {
      CNR_ERROR(logger_,"Error in removing the objects from the scene\n%s", what.c_str());
      res.success = false;
      return false;
    }
  }

  res.success = true;
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cnr_scene_manager");
  ros::NodeHandle n;
  std::string package_name = "cnr_scene_manager";
  std::string package_path = ros::package::getPath(package_name);

  if(package_path.empty())
    throw std::invalid_argument("Failed to get path for package!");

  std::string logger_file = package_path+"/config/logger_param.yaml";
  logger_ = std::make_shared<cnr_logger::TraceLogger>("cnr_scene_manager_logger",logger_file);

  scene_manager_.reset(new cnr_tf_named_object_loader::TFNamedObjectsManager());

  ros::ServiceServer add_service = n.advertiseService(ADD_OBJ_SERVICE, add);
  ROS_INFO("Ready to add objects to the scene");

  ros::ServiceServer remove_service = n.advertiseService(REMOVE_OBJ_SERVICE, remove);
  ROS_INFO("Ready to remove objects from the scene");

  ros::spin();

  return 0;
}
