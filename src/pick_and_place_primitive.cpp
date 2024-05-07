#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class RobotArm {
public:
    RobotArm(rclcpp::Node::SharedPtr node) : node_(node){
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

        executor.add_node(move_group_node);
        std::thread([this]() { this->executor.spin(); }).detach();

        move_group_arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, PLANNING_GROUP_ARM);
        joint_model_group_arm = move_group_arm->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
        
        move_group_gripper = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, PLANNING_GROUP_GRIPPER);
        joint_model_group_gripper = move_group_gripper->getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);
        
        node_->declare_parameter<bool>("is_robot_sim", false);
        RCLCPP_INFO(LOGGER, "---------------------------INSTANTIATED!----------------------------------");
    }

    bool get_is_robot_sim() {
        return node_->get_parameter("is_robot_sim").as_bool();
    }

    void move_joint_space(float angle_0, float angle_1, float angle_2, float angle_3, float angle_4, float angle_5) {
        RCLCPP_INFO(LOGGER, "---------------------------MOVE JOINT STATE!----------------------------------");
        moveit::core::RobotStatePtr current_state = move_group_arm->getCurrentState(10);

        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group_arm, joint_group_positions);

        joint_group_positions[0] = angle_0; // Shoulder Pan
        joint_group_positions[1] = angle_1; // Shoulder Lift
        joint_group_positions[2] = angle_2; // Elbow
        joint_group_positions[3] = angle_3; // Wrist 1
        joint_group_positions[4] = angle_4; // Wrist 2
        joint_group_positions[5] = angle_5; // Wrist 3

        move_group_arm->setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group_arm->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success) {
            move_group_arm->execute(my_plan);
        } else {
            RCLCPP_ERROR(LOGGER, "Joint space movement plan failed!");
        }
    }

    void move_end_effector(double x, double y, double z, double roll_deg, double pitch_deg, double yaw_deg) {
        RCLCPP_INFO(LOGGER, "---------------------------MOVE END EFFECTOR!----------------------------------");
        geometry_msgs::msg::Pose target_pose;
        tf2::Quaternion q;
        q.setRPY(roll_deg * M_PI/180.0, pitch_deg * M_PI/180.0, yaw_deg * M_PI/180.0);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        move_group_arm->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group_arm->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success) {
            move_group_arm->execute(my_plan);
        } else {
            RCLCPP_ERROR(LOGGER, "End effector movement plan failed!");
        }
    }

    void move_waypoint(double delta, const std::string& direction) {
        RCLCPP_INFO(LOGGER, "---------------------------MOVE WAYPOINT!----------------------------------");

        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose target_pose = move_group_arm->getCurrentPose().pose;

        if(direction == "x") {
            target_pose.position.x += delta;
        } else if(direction == "y") {
            target_pose.position.y += delta;
        } else if(direction == "z") {
            target_pose.position.z += delta;
        } else {
            RCLCPP_ERROR(LOGGER, "Invalid direction!");
            return;
        }

        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;

        double fraction = move_group_arm->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if(fraction >= 0.0) {
            move_group_arm->execute(trajectory);
        } else {
            RCLCPP_ERROR(LOGGER, "Waypoint movement plan failed!");
        }
    }

    void cmd_arm(const std::string& target_name) {
        RCLCPP_INFO(LOGGER, "---------------------------ARM COMMAND!----------------------------------");
        RCLCPP_INFO(LOGGER, target_name.c_str());

        move_group_arm->setNamedTarget(target_name);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
        bool success_arm = (move_group_arm->plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);

        if(success_arm) {
            move_group_arm->execute(my_plan_arm);
        } else {
            RCLCPP_ERROR(LOGGER, "Arm plan failed!");
        }
    }

    void cmd_gripper(const std::string& target_name) {
        RCLCPP_INFO(LOGGER, "---------------------------GRIPPER COMMAND!----------------------------------");
        RCLCPP_INFO(LOGGER, target_name.c_str());

        move_group_gripper->setNamedTarget(target_name);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
        bool success_gripper = (move_group_gripper->plan(my_plan_gripper) == moveit::core::MoveItErrorCode::SUCCESS);

        if(success_gripper) {
            move_group_gripper->execute(my_plan_gripper);
        } else {
            RCLCPP_ERROR(LOGGER, "Gripper plan failed!");
        }
    }

    void move_gripper_space(double distance) {
        RCLCPP_INFO(LOGGER, "---------------------------MOVE GRIPPER JOINTS!----------------------------------");

        std::vector<double> joint_group_positions;

        move_group_gripper->getCurrentState()->copyJointGroupPositions(move_group_gripper->getCurrentState()->getRobotModel()->getJointModelGroup(move_group_gripper->getName()), joint_group_positions);

        joint_group_positions[0] = distance; // rg2_gripper_finger_left_joint
        joint_group_positions[1] = 0; // rg2_gripper_thumb_left_joint
        joint_group_positions[2] = 0; // rg2_gripper_finger_right_joint
        joint_group_positions[3] = 0; // rg2_gripper_thumb_right_joint

        move_group_gripper->setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group_gripper->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success) {
            move_group_gripper->execute(my_plan);
        } else {
            RCLCPP_ERROR(LOGGER, "Gripper space movement plan failed!");
        }
    }

    void print_end_effector_position() {
        geometry_msgs::msg::PoseStamped current_pose = move_group_arm->getCurrentPose();
        RCLCPP_INFO(LOGGER, ">>>>>>>>>>>>>>>>>> Position: (%f, %f, %f)", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    }
    
    void move2xy(double x, double y) {
        // Obtener la posición actual del efector final
        geometry_msgs::msg::PoseStamped current_pose = move_group_arm->getCurrentPose();

        // Calcular la diferencia en x y y
        double dx = x - current_pose.pose.position.x;
        double dy = y - current_pose.pose.position.y;

        // Definir el tamaño del paso
        double step_size = 0.01;

        // Calcular el número de pasos
        int num_steps_x = std::abs(dx / step_size);
        int num_steps_y = std::abs(dy / step_size);

        // Mover el efector final a la posición objetivo en pasos
        for (int i = 0; i < num_steps_x; ++i) {
            move_waypoint(dx > 0 ? step_size : -step_size, "x");
        }
        for (int i = 0; i < num_steps_y; ++i) {
            move_waypoint(dy > 0 ? step_size : -step_size, "y");
        }
    }
private:
    rclcpp::Node::SharedPtr move_group_node;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm;
    const moveit::core::JointModelGroup* joint_model_group_arm;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper;
    const moveit::core::JointModelGroup* joint_model_group_gripper;
    rclcpp::executors::SingleThreadedExecutor executor;
    bool is_robot_sim;
    rclcpp::Node::SharedPtr node_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // create an instance of rclcpp:Node
    auto node = std::make_shared<rclcpp::Node>("for_check_arguments");
    RobotArm robotArm(node);
    
    // read the atributte 'is_robot_sim' of the object
    if(robotArm.get_is_robot_sim()) {
        RCLCPP_INFO(LOGGER, "\n\n\n Robot is simulated \n\n\n");
        // Shoulder Pan | Shoulder Lift | Elbow | Wrist 1 | Wrist 2 | Wrist 3
        robotArm.move_joint_space(3.47, -1.64, -1.59, -1.47, 1.56, -5.70);
        robotArm.cmd_gripper("gripper_open");

        // robotArm.move_end_effector(0.2, -0.34, 0.28, 180.0, 0.0, 0.0);
        robotArm.move_waypoint(-0.1, "z");
        robotArm.cmd_gripper("gripper_close");

        robotArm.move_waypoint(0.1, "z");

        robotArm.move_joint_space(6.1, -1.64, -1.59, -1.47, 1.56, -5.70);
        robotArm.cmd_gripper("gripper_open");
    } else {
        RCLCPP_INFO(LOGGER, "\n\n\n Robot is real \n\n\n");
        // Shoulder Pan | Shoulder Lift | Elbow | Wrist 1 | Wrist 2 | Wrist 3
        robotArm.move_joint_space(3.87, -1.75, -1.46, -1.48, 1.55, -5.56);
        robotArm.cmd_gripper("gripper_open");

        // robotArm.move_end_effector(0.2, -0.34, 0.28, 180.0, 0.0, 0.0);
        robotArm.move_waypoint(-0.05, "z");
        robotArm.cmd_gripper("gripper_close");

        robotArm.move_waypoint(0.15, "z");

        robotArm.move_joint_space(0.73, -1.75, -1.46, -1.48, 1.55, -5.56);
        robotArm.cmd_gripper("gripper_open");
    }
    robotArm.cmd_arm("home");

    rclcpp::shutdown();
    return 0;
}

//name:
//- shoulder_pan_joint
//- shoulder_lift_joint
//- elbow_joint
//- wrist_1_joint
//- wrist_2_joint
//- wrist_3_joint
//- rg2_gripper_finger_left_joint
//position:
