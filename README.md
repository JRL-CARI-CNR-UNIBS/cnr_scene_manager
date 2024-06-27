# cnr_scene_manager

The `cnr_scene_manager` package provides a tool to dynamically load and unload objects to and from the MoveitPlanningScene. It interfaces with the MoveitPlanningScene through the [cnr_tf_named_object_loader](https://github.com/CNR-STIIMA-IRAS/cnr_tf_named_object_loader).

## Dependencies

This package relies on the following dependencies:

- [cnr_param](https://github.com/CNR-STIIMA-IRAS/cnr_param): For reading parameters.
- [cnr_logger](https://github.com/CNR-STIIMA-IRAS/cnr_logger): For logging.
- [cnr_tf_named_object_loader](https://github.com/CNR-STIIMA-IRAS/cnr_tf_named_object_loader): For interfacing with the MoveitPlanningScene.

## How It Works

The core functionality is provided by the `cnr_scene_manager_node`. This node maintains a list of available object descriptions read from parameters and uses them to load the scene at startup or during runtime.

### Initialization

When launched, the node reads available object descriptions from the parameter namespace `param_ns/object_geometries`, where `param_ns` is the first-level namespace provided to the node (see the [example launch file](https://github.com/JRL-CARI-CNR-UNIBS/cnr_scene_manager/blob/master/cnr_scene_manager/launch/cnr_scene_manager.launch)). These object descriptions are stored and referred to as *registered objects*.

### Loading the Initial Scene

The initial scene is loaded based on the description provided under the parameter namespace `param_ns/scene_objects`. All objects referred to in the scene must be *registered objects*.

### Dynamic Scene Management

The node provides the following services to dynamically manage the scene during runtime:
- `cnr_scene_manager/add_objects`: Adds *registered objects* to the scene.
- `cnr_scene_manager/move_objects`: Moves *registered objects* to different poses in the scene.
- `cnr_scene_manager/remove_objects`: Removes *registered objects* from the scene.

### Run an example
You can run an example by launching ```ros2 launch cnr_scene_manager cnr_scene_manager_test.launch.py```, but you need to download [this package](https://github.com/JRL-CARI-CNR-UNIBS/cell_cartesian) first.

### Examples and Configuration

- An example of object and scene descriptions can be found in [this config file](https://github.com/JRL-CARI-CNR-UNIBS/cnr_scene_manager/blob/ros2/cnr_scene_manager/config/scene.yaml).
- The necessary services and messages are defined in [this package](https://github.com/JRL-CARI-CNR-UNIBS/cnr_scene_manager/tree/ros2/cnr_scene_manager_msgs).
