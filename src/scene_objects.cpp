#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class SceneManager {
public:
    SceneManager(rclcpp::Node::SharedPtr node) : node_(node){
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

        executor.add_node(move_group_node);
        std::thread([this]() { this->executor.spin(); }).detach();

        node_->declare_parameter<bool>("is_robot_sim", false);
        RCLCPP_INFO(LOGGER, "---------------------------INSTANTIATED!----------------------------------");
        node_->declare_parameter<float>("x_dim", 0.1);
        node_->declare_parameter<float>("y_dim", 0.1);
        node_->declare_parameter<float>("z_dim", 0.1);
        node_->declare_parameter<float>("x_pos", 0.1);
        node_->declare_parameter<float>("y_pos", 0.1);
        node_->declare_parameter<float>("z_pos", 0.1);
    }

    bool get_is_robot_sim() {
        return node_->get_parameter("use_sim_time").get_value<bool>();
    }
    float get_x_dim() {return node_->get_parameter("x_dim").get_value<float>();}
    float get_y_dim() {return node_->get_parameter("y_dim").get_value<float>();}
    float get_z_dim() {return node_->get_parameter("z_dim").get_value<float>();}
    float get_x_pos() {return node_->get_parameter("x_pos").get_value<float>();}
    float get_y_pos() {return node_->get_parameter("y_pos").get_value<float>();}
    float get_z_pos() {return node_->get_parameter("z_pos").get_value<float>();}

    moveit_msgs::msg::CollisionObject createCollisionObject(
        const std::string& id, 
        const std::string& frame_id, 
        const shape_msgs::msg::SolidPrimitive& primitive, 
        const geometry_msgs::msg::Pose& pose) 
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = id;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(pose);
        collision_object.operation = collision_object.ADD;

        RCLCPP_INFO(LOGGER, "Added an object into the world");
        return collision_object;
    }

    shape_msgs::msg::SolidPrimitive createSolidPrimitiveBOX(double x, double y, double z) {
        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions.resize(3);
        box.dimensions[0] = x; // x dimension
        box.dimensions[1] = y; // y dimension
        box.dimensions[2] = z; // z dimension
        return box;
    }

    geometry_msgs::msg::Pose createPose(double x, double y, double z, double w) {
        geometry_msgs::msg::Pose pose;
        pose.orientation.w = w;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        return pose;
    }

    void removeCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                            const std::vector<moveit_msgs::msg::CollisionObject>& collision_objects) 
    {
        std::vector<std::string> object_ids;
        for (const auto& object : collision_objects) {
            object_ids.push_back(object.id);
        }
        planning_scene_interface.removeCollisionObjects(object_ids);
    }

private:
    rclcpp::Node::SharedPtr move_group_node;
    rclcpp::executors::SingleThreadedExecutor executor;
    bool is_robot_sim;
    rclcpp::Node::SharedPtr node_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // create an instance of rclcpp:Node
    auto node = std::make_shared<rclcpp::Node>("for_check_arguments");
    SceneManager SceneManager(node);
    
    //SCENE
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Create vector to hold collision objects.
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;


    // create the wall
    auto wall_object = SceneManager.createCollisionObject(
        "wall", 
        "base_link",
        SceneManager.createSolidPrimitiveBOX(2.0, 0.1, 2.0),
        SceneManager.createPose(-0.25,-0.4,0, 1.0)
    );
    // create the table
    auto table_object = SceneManager.createCollisionObject(
        "table", 
        "base_link",
        SceneManager.createSolidPrimitiveBOX(0.85,1.8,1.0),
        SceneManager.createPose(0.3,0.35,-0.501, 1.0)
    );
    // create machine
    auto machine_object = SceneManager.createCollisionObject(
        "machine", 
        "base_link",
        SceneManager.createSolidPrimitiveBOX(0.6,0.15,0.4),
        SceneManager.createPose(0.2,0.85,0.2, 1.0)
    );


    // push the objects into the vector
    collision_objects.push_back(wall_object);
    collision_objects.push_back(table_object);
    collision_objects.push_back(machine_object);
    planning_scene_interface.addCollisionObjects(collision_objects);
    // Wait for MoveGroup to recieve and process the collision object message.
    rclcpp::sleep_for(std::chrono::seconds(2));



    // read the atributte 'is_robot_sim' of the object
    if(SceneManager.get_is_robot_sim()) {
        RCLCPP_INFO(LOGGER, "\n\n\n Robot is simulated \n\n\n");

    } else {
        RCLCPP_INFO(LOGGER, "\n\n\n Robot is real \n\n\n");

    }



    while (rclcpp::ok()) {
        // Create a new collision object using the parameters
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = "XXX";
        collision_object.header.frame_id = "base_link";
        collision_object.primitives.push_back(SceneManager.createSolidPrimitiveBOX(SceneManager.get_x_dim(), SceneManager.get_y_dim(), SceneManager.get_z_dim()));
        collision_object.primitive_poses.push_back(SceneManager.createPose(SceneManager.get_x_pos(), SceneManager.get_y_pos(), SceneManager.get_z_pos(), 1.0));
        collision_object.operation = collision_object.ADD;
    
        // Add the new object
        planning_scene_interface.applyCollisionObject(collision_object);
    
        // Sleep a little bit
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    
        // Remove the object
        planning_scene_interface.removeCollisionObjects({"XXX"});
    
        rclcpp::spin_some(node);
    }

    // remove the objects from the scene
    SceneManager.removeCollisionObjects(planning_scene_interface, collision_objects);

    // rclcpp::shutdown();
    return 0;
}
