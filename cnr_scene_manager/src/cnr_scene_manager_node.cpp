#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <geometric_shapes/shape_operations.h>

#include <cnr_param/cnr_param.h>
#include <cnr_logger/cnr_logger.h>
#include <cnr_tf_named_object_loader/cnr_tf_named_object_loader.h>

#include <cnr_scene_manager_msgs/srv/add_objects.hpp>
#include <cnr_scene_manager_msgs/srv/move_objects.hpp>
#include <cnr_scene_manager_msgs/srv/remove_objects.hpp>

#define OBJS_NS "/object_geometries"
#define SCENE_NS "/scene_objects"
#define ADD_OBJ_SERVICE "~/add_objects"
#define MOVE_OBJ_SERVICE "~/move_objects"
#define REMOVE_OBJ_SERVICE "~/remove_objects"

using namespace  cnr_tf_named_object_loader;
using color_t = std_msgs::msg::ColorRGBA;

/**
 * @brief The object_type_t struct represents a type of object that can be loaded into the scene
 */
struct object_type_t
{
public:
  bool use_mesh_;
  color_t color_;
  std::string id_;
  shape_msgs::msg::Mesh mesh_;                // if mesh, no primitive
  shape_msgs::msg::SolidPrimitive primitive_; // if primitive, no mesh
  Eigen::Affine3d rt_matrix_;                 // roto-translation matrix of the mesh/primitive respect to the object reference frame
  geometry_msgs::msg::Pose rt_pose_;          // roto-translation pose msg of the mesh/primitive respect to the object reference frame
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
std::shared_ptr<TFNamedObjectsManager> scene_manager_;

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

  r_obj.obj_.color_.r = rgba.at(0);
  r_obj.obj_.color_.g = rgba.at(1);
  r_obj.obj_.color_.b = rgba.at(2);
  r_obj.obj_.color_.a = rgba.at(3);

  /********************* RT-MATRIX *****************************/

  r_obj.obj_.rt_matrix_ .setIdentity();
  if(yaml_node["quaternion"])
  {
    std::vector<double> quaternion;
    try {
      quaternion = yaml_node["quaternion"].as<std::vector<double>>();
    } catch (const YAML::BadConversion& e) {
      CNR_ERROR(logger_,"quaternion should be an array of 4 double (i,j,k,w)");
      return false;
    }

    if(quaternion.size() != 4)
    {
      CNR_ERROR(logger_,"quaternion should be an array of 4 double (i,j,k,w)");
      return false;
    }

    Eigen::Quaterniond q(quaternion.at(3),quaternion.at(0),quaternion.at(1),quaternion.at(2));
    q.normalize();
    r_obj.obj_.rt_matrix_=q;
  }

  if(yaml_node["position"])
  {
    std::vector<double> position;
    try {
      position = yaml_node["position"].as<std::vector<double>>();
    } catch (const YAML::BadConversion& e) {
      CNR_ERROR(logger_,"position should be an array of 3 double (x,y,z)");
      return false;
    }

    if(position.size() != 3)
    {
      CNR_ERROR(logger_,"position should be an array of 3 double (x,y,z)");
      return false;
    }

    r_obj.obj_.rt_matrix_.translation()(0)=position.at(0);
    r_obj.obj_.rt_matrix_.translation()(1)=position.at(1);
    r_obj.obj_.rt_matrix_.translation()(2)=position.at(2);
  }

  r_obj.obj_.rt_pose_ = tf2::toMsg(r_obj.obj_.rt_matrix_);

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

    r_obj.obj_.mesh_ = boost::get<shape_msgs::msg::Mesh>(mesh_msg);
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

    r_obj.obj_.primitive_.type = shape_msgs::msg::SolidPrimitive::BOX;
    r_obj.obj_.primitive_.dimensions.resize(3);
    r_obj.obj_.primitive_.dimensions[r_obj.obj_.primitive_.BOX_X] = size.at(0);
    r_obj.obj_.primitive_.dimensions[r_obj.obj_.primitive_.BOX_Y] = size.at(1);
    r_obj.obj_.primitive_.dimensions[r_obj.obj_.primitive_.BOX_Z] = size.at(2);

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

    r_obj.obj_.primitive_.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    r_obj.obj_.primitive_.dimensions.resize(1);
    r_obj.obj_.primitive_.dimensions[r_obj.obj_.primitive_.SPHERE_RADIUS] = radius;

    return true;
  }
  else if(yaml_node["cylinder"])
  {
    std::vector<double> size;
    try {
      size = yaml_node["cylinder"].as<std::vector<double>>();
    } catch (const YAML::BadConversion& e) {
      CNR_ERROR(logger_,"cylinder should be an array of 2 double");
      return false;
    }

    if(size.size() != 2)
    {
      CNR_ERROR(logger_,"cylinder should be an array of 2 double");
      return false;
    }

    r_obj.obj_.primitive_.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    r_obj.obj_.primitive_.dimensions.resize(2);
    r_obj.obj_.primitive_.dimensions[r_obj.obj_.primitive_.CYLINDER_HEIGHT] = size.at(0);
    r_obj.obj_.primitive_.dimensions[r_obj.obj_.primitive_.CYLINDER_RADIUS] = size.at(1);

    return true;
  }
  else if(yaml_node["cone"])
  {
    std::vector<double> size;
    try {
      size = yaml_node["cone"].as<std::vector<double>>();
    } catch (const YAML::BadConversion& e) {
      CNR_ERROR(logger_,"cone should be an array of 2 double");
      return false;
    }

    if(size.size() != 2)
    {
      CNR_ERROR(logger_,"cone should be an array of 2 double");
      return false;
    }

    r_obj.obj_.primitive_.type = shape_msgs::msg::SolidPrimitive::CONE;
    r_obj.obj_.primitive_.dimensions.resize(2);
    r_obj.obj_.primitive_.dimensions[r_obj.obj_.primitive_.CONE_HEIGHT] = size.at(0);
    r_obj.obj_.primitive_.dimensions[r_obj.obj_.primitive_.CONE_RADIUS] = size.at(1);

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

  std::string text = "REGISTERING OBJECT TYPES FROM "+ns;
  std::string header = "=========================================================================================";

  CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BG()<<header<<cnr_logger::RESET());
  CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BG()<<std::setw((header.length()-text.length())/2+text.length())<<text<<cnr_logger::RESET());
  CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BG()<<header<<cnr_logger::RESET()<<cnr_logger::RESET());

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
      CNR_ERROR(logger_, ns<<" has type "<<objects_geometries.Type()<<", while it should be a map (type "<<YAML::NodeType::Map<<")");
      return false;
    }

    size_t dim;
    std::string id;
    for(YAML::const_iterator it = objects_geometries.begin(); it != objects_geometries.end(); ++it)
    {
      registered_object_t r_obj;
      id = it->first.as<std::string>();
      if(register_type(it->second,id,r_obj))
      {
        dim = registered_object_types_.size();

        registered_object_types_.insert(std::pair<std::string,registered_object_t>(r_obj.obj_.id_,r_obj));

        if(registered_object_types_.size() == dim)
          CNR_WARN(logger_, "Cannot add multiple objects with ID: "<<id);
        else
          CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::G()<<"\t- "<<cnr_logger::W()<<id
                   <<cnr_logger::RESET()<<cnr_logger::G()<<" registered"<<cnr_logger::RESET());
      }
      else
        CNR_ERROR(logger_, "Cannot register object with ID: "<<id);
    }
    CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BG()<<header<<cnr_logger::RESET());
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
bool instantiate_registered_obj(const std::map<std::string,registered_object_t>::iterator& it, const std::string& reference_frame, const geometry_msgs::msg::Pose& pose_msg, object_t& obj)
{
  if(it == registered_object_types_.end())
    return false;

  obj.id = it->first + "/n_" + std::to_string(it->second.count_);
  it->second.count_++; //increase the counter of published objects with this type

  obj.operation = object_t::ADD;
  obj.header.frame_id = reference_frame;

  /* === Include primitive/mesh relative pose into the pose of the object's reference frame === */

  //  Eigen::Affine3d rt_matrix_frame2obj;
  //  tf::poseMsgToEigen(pose_msg,rt_matrix_frame2obj);

  //  geometry_msgs::msg::Pose absolute_pose_msg;
  //  Eigen::Affine3d rt_matrix_absolute = rt_matrix_frame2obj*it->second.obj_.rt_matrix_;

  //  tf::poseEigenToMsg(rt_matrix_absolute,absolute_pose_msg);

  //  obj.pose = absolute_pose_msg;

  /* ========================================================================================== */

  /* =========================== Otherwise keep those poses separate ========================== */

  obj.pose = pose_msg;

  /* ========================================================================================== */


  if(it->second.obj_.use_mesh_)
  {
    obj.meshes.resize(1);
    obj.mesh_poses.resize(1);
    obj.meshes[0] = it->second.obj_.mesh_;
    obj.mesh_poses[0] = it->second.obj_.rt_pose_;
  }
  else
  {
    obj.primitives.resize(1);
    obj.primitive_poses.resize(1);
    obj.primitives[0] = it->second.obj_.primitive_;
    obj.primitive_poses[0] = it->second.obj_.rt_pose_;
  }

  return true;
}

/**
 * @brief load_scene_obj reads from a YAML::Node the attributes (type, frame, position, quaternion) to instantiate a registered obeject type
 * @param yaml_node the YAML::Node from which the attributes are retrieved
 * @param obj the Moveit collision object created
 * @param color std_msgs::msg::ColorRGBA color of the object to load
 * @return true if everything went well, false otherwise
 */
bool load_scene_obj(const YAML::Node& yaml_node, object_t& obj, color_t& color)
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

  color = it->second.obj_.color_;

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

  geometry_msgs::msg::Pose pose_msg;
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
  std::string what;
  std::string ns = param_ns+SCENE_NS;
  std::string text = "LOADING SCENE OBJECTS FROM "+ns;
  std::string header = "=========================================================================================";

  CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BG()<<header<<cnr_logger::RESET());
  CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BG()<<std::setw((header.length()-text.length())/2+text.length())<<text<<cnr_logger::RESET());
  CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BG()<<header<<cnr_logger::RESET());

  if(cnr::param::has(ns,what))
  {
    YAML::Node scene;
    if(not cnr::param::get(ns,scene,what))
    {
      CNR_ERROR(logger_, "Cannot get scene "<<ns<<"\n"<<what);
      return false;
    }

    if(not scene.IsSequence())
    {
      CNR_ERROR(logger_, ns<<" has type "<<scene.Type()<<", while it should be a sequence (type "<<YAML::NodeType::Sequence<<")");
      return false;
    }

    std::vector<color_t> colors;
    tf_named_objects_t scene_objs;
    for(YAML::iterator it = scene.begin(); it != scene.end(); ++it)
    {
      color_t color;
      object_t scene_obj;
      if(load_scene_obj(*it,scene_obj,color))
      {
        colors.push_back(std::move(color));
        scene_objs.emplace_back(scene_obj);
        CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::G()<<"\t- "<<cnr_logger::W()<<scene_objs.back().id
                 <<cnr_logger::RESET()<<cnr_logger::G()<<" loaded"<<cnr_logger::RESET());
      }
      else
        CNR_ERROR(logger_,cnr_logger::RESET()<<"Cannot load the "<<std::distance(scene.begin(),it)<<"-th object into the scene"<<cnr_logger::RESET());
    }
    CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BG()<<header<<cnr_logger::RESET());

    if(not scene_manager_->addNamedTFObjects(scene_objs,10.0,colors,what))
    {
      CNR_ERROR(logger_,"Error in adding the objects to the scene\n%s", what.c_str());
      return false;
    }
  }
  else
  {
    CNR_ERROR(logger_, ns<<" not available\n"<<what);
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
void add_obj(const std::shared_ptr<cnr_scene_manager_msgs::srv::AddObjects::Request> req,
             std::shared_ptr<cnr_scene_manager_msgs::srv::AddObjects::Response> res)
{

  CNR_INFO(logger_,"=================================================================");
  CNR_INFO(logger_,"===               ADD OBJECTS REQUEST RECEIVED!               ===");
  CNR_INFO(logger_,"=================================================================");

  res->ids.resize(req->objects.size());

  std::string what;
  std::vector<color_t> colors;
  tf_named_objects_t objs_to_add;
  std::map<std::string,registered_object_t>::iterator it;

  for(unsigned int i=0; i<req->objects.size(); i++)
  {
    it = registered_object_types_.find(req->objects[i].object_type);
    if(it==registered_object_types_.end())
    {
      CNR_ERROR(logger_,"Object type %s not registered",req->objects[i].object_type.c_str());
      res->success = false;
      return;
    }

    object_t cobj;
    if(instantiate_registered_obj(it,req->objects[i].pose.header.frame_id,req->objects[i].pose.pose,cobj))
    {
      res->ids[i] = cobj.id;

      cobj.operation = object_t::ADD;
      objs_to_add.emplace_back(cobj);

      colors.push_back(it->second.obj_.color_);
    }
    else
    {
      CNR_ERROR(logger_,"Object type %s cannot be instantiated",req->objects[i].object_type.c_str());
      res->success = false;
      return;
    }
  }

  double timeout = 0.01;
  if(req->timeout)
    timeout = req->timeout;

  if(not scene_manager_->addNamedTFObjects(objs_to_add,timeout,colors,what))
  {
    CNR_ERROR(logger_,"Error in adding the objects to the scene\n%s", what.c_str());
    res->success = false;
    return;
  }

  res->success = true;
  return;
}

/**
 * @brief move callback for MOVE_OBJ_SERVICE
 * @param req
 * @param res
 * @return
 */
void move_obj(const std::shared_ptr<cnr_scene_manager_msgs::srv::MoveObjects::Request> req,
             std::shared_ptr<cnr_scene_manager_msgs::srv::MoveObjects::Response> res)
{
//  CNR_INFO(logger_,"=================================================================");
//  CNR_INFO(logger_,"===               MOVE OBJECTS REQUEST RECEIVED!              ===");
//  CNR_INFO(logger_,"=================================================================");

  if(req->obj_ids.size() != req->poses.size())
  {
    CNR_ERROR(logger_, "req->obj_id size should be euqal to req->poses size, but they are %u and %u",req->obj_ids.size(),req->poses.size());
    res->success = false;
    return;
  }

  std::map<std::string,geometry_msgs::msg::Pose> pose_map;
  std::map<std::string,std_msgs::msg::ColorRGBA> color_map;
  std::pair<std::string,geometry_msgs::msg::Pose> pair_pose;
  std::pair<std::string,std_msgs::msg::ColorRGBA> pair_color;

  size_t pos;
  std::string id, registered_obj_key;

  for(size_t i=0;i<req->obj_ids.size();i++)
  {
    id = req->obj_ids[i];
    pos = id.find("/n_");

    if (pos != std::string::npos)
      registered_obj_key = id.substr(0, pos);
    else
    {
      CNR_ERROR(logger_,"id not valid, it should be like registered_type/n_XXX");
      res->success = false;
      return;
    }

    auto it = registered_object_types_.find(registered_obj_key);
    if(it == registered_object_types_.end())
    {
      CNR_ERROR(logger_,"id not associated to a registered object type");
      res->success = false;
      return;
    }
    else
    {
      pair_color.first = id;
      pair_color.second = it->second.obj_.color_;

      color_map.insert(pair_color);

      pair_pose.first = id;
      pair_pose.second.position.x = req->poses[i].position.x;
      pair_pose.second.position.y = req->poses[i].position.y;
      pair_pose.second.position.z = req->poses[i].position.z;
      pair_pose.second.orientation.x = req->poses[i].orientation.x;
      pair_pose.second.orientation.y = req->poses[i].orientation.y;
      pair_pose.second.orientation.z = req->poses[i].orientation.z;
      pair_pose.second.orientation.w = req->poses[i].orientation.w;

      pose_map.insert(pair_pose);
    }
  }

  double timeout = 0.01;
  if(req->timeout)
    timeout = req->timeout;

  std::string what;
  if(not scene_manager_->moveNamedTFObjects(pose_map,color_map,timeout,what))
  {
    CNR_ERROR(logger_,"Error in moving the objects in the scene\n%s", what.c_str());
    res->success = false;
    return;
  }

  res->success = true;
  return;
}

void remove_obj(const std::shared_ptr<cnr_scene_manager_msgs::srv::RemoveObjects::Request> req,
                std::shared_ptr<cnr_scene_manager_msgs::srv::RemoveObjects::Response> res)
{
  CNR_INFO(logger_,"=================================================================");
  CNR_INFO(logger_,"==             REMOVE OBJECTS REQUEST RECEIVED!               ===");
  CNR_INFO(logger_,"=================================================================");

  double timeout = 0.01;
  if(req->timeout)
    timeout = req->timeout;

  std::string what;
  if(req->obj_ids.empty()) //no objects means reset the scene
  {
    if(not scene_manager_->resetScene(timeout,what))
    {
      CNR_ERROR(logger_,"Error in resetting the scene\n%s", what.c_str());
      res->success = false;
      return;
    }
  }
  else
  {
    if(not scene_manager_->removeNamedObjects(req->obj_ids,timeout,what))
    {
      CNR_ERROR(logger_,"Error in removing the objects from the scene\n%s", what.c_str());
      res->success = false;
      return;
    }
  }

  res->success = true;
  return;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cnr_scene_manager");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::string package_name = "cnr_scene_manager";
  std::string package_path = ament_index_cpp::get_package_share_directory(package_name);

  if(package_path.empty())
    throw std::invalid_argument("Failed to get path for package!");

  std::string logger_file = package_path+"/config/logger_param.yaml";
  logger_ = std::make_shared<cnr_logger::TraceLogger>("cnr_scene_manager_logger",logger_file);

  std::string param_ns;
  node->declare_parameter("param_ns", "");
  node->get_parameter("param_ns", param_ns);

  CNR_INFO(logger_,cnr_logger::RESET()<<cnr_logger::BY()<<
           "Scene objects read under param namespace: "<<param_ns<<cnr_logger::RESET());

  // Register the available object types listed under param_ns/OBJS_NS parameter
  scene_manager_.reset(new TFNamedObjectsManager(node));
  if(not register_object_types(param_ns))
  {
    CNR_ERROR(logger_,"Cannot register object types");
    rclcpp::shutdown();
    return 1;
  }

  // Load the scene objects listed under param_ns/SCENE_NS parameter
  if(not load_scene(param_ns))
  {
    CNR_ERROR(logger_,"Cannot load scene objects");
    rclcpp::shutdown();
    return 1;
  }

  // Declare services
  auto add_service = node->create_service<cnr_scene_manager_msgs::srv::AddObjects>(ADD_OBJ_SERVICE, &add_obj);
  CNR_INFO(logger_, "Ready to add objects to the scene");

  auto move_service = node->create_service<cnr_scene_manager_msgs::srv::MoveObjects>(MOVE_OBJ_SERVICE, &move_obj);
  CNR_INFO(logger_, "Ready to move objects in the scene");

  auto remove_service = node->create_service<cnr_scene_manager_msgs::srv::RemoveObjects>(REMOVE_OBJ_SERVICE, &remove_obj);
  CNR_INFO(logger_, "Ready to remove objects from the scene");

  rclcpp::WallRate loop_rate(1000);
  while(rclcpp::ok())
  {
    executor.spin_some();
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
