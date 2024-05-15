#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~SCENE OBJECTs~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
    
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~JOINTs~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
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

    void move_single_joint(const std::string& joint_name, double degree) {
        RCLCPP_INFO(LOGGER, "---------------------------MOVE JOINT %s!----------------------------------", joint_name.c_str());
        moveit::core::RobotStatePtr current_state = move_group_arm->getCurrentState(10);

        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group_arm, joint_group_positions);

        double angle = degree * M_PI / 180.0;
        if(joint_name == "shoulder_pan_joint") {
            joint_group_positions[0] = angle;
        } else if(joint_name == "shoulder_lift_joint") {
            joint_group_positions[1] = angle;
        } else if(joint_name == "elbow_joint") {
            joint_group_positions[2] = angle;
        } else if(joint_name == "wrist_1_joint") {
            joint_group_positions[3] = angle;
        } else if(joint_name == "wrist_2_joint") {
            joint_group_positions[4] = angle;
        } else if(joint_name == "wrist_3_joint") {
            joint_group_positions[5] = angle;
        } else {
            RCLCPP_ERROR(LOGGER, "Invalid joint name!");
            return;
        }

        move_group_arm->setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group_arm->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success) {
            move_group_arm->execute(my_plan);
        } else {
            RCLCPP_ERROR(LOGGER, "Joint space movement plan failed!");
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

        // Set planning time to 5 seconds
        move_group_gripper->setPlanningTime(5.0);
        // Set number of planning attempts to ..
        move_group_gripper->setNumPlanningAttempts(10);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group_gripper->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success) {
            move_group_gripper->execute(my_plan);
        } else {
            RCLCPP_ERROR(LOGGER, "Gripper space movement plan failed!");
        }
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END EFFECTOR~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

        // Set planning time to 10 seconds
        move_group_arm->setPlanningTime(10.0);
        // Set number of planning attempts to ..
        move_group_arm->setNumPlanningAttempts(20);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group_arm->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success) {
            move_group_arm->execute(my_plan);
        } else {
            RCLCPP_ERROR(LOGGER, "End effector movement plan failed!");
        }
    }
    
    void position_end_effector(double x, double y, double z) {
        RCLCPP_INFO(LOGGER, "---------------------------SET END EFFECTOR POSITION!----------------------------------");
        geometry_msgs::msg::Pose target_pose;
        // Get current end effector pose
        geometry_msgs::msg::Pose current_pose = move_group_arm->getCurrentPose().pose;
        target_pose.orientation.x = current_pose.orientation.x;
        target_pose.orientation.y = current_pose.orientation.y;
        target_pose.orientation.z = current_pose.orientation.z;
        target_pose.orientation.w = current_pose.orientation.w;

        // Set target position
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        move_group_arm->setPoseTarget(target_pose);

        // Set planning time to 10 seconds
        move_group_arm->setPlanningTime(10.0);
        // Set number of planning attempts to ..
        move_group_arm->setNumPlanningAttempts(100);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group_arm->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success) {
            move_group_arm->execute(my_plan);
        } else {
            RCLCPP_ERROR(LOGGER, "End effector movement plan failed!");
        }
    }

    void orientation_end_effector(double roll_deg, double pitch_deg, double yaw_deg) {
        RCLCPP_INFO(LOGGER, "---------------------------SET END EFFECTOR ORIENTATION!----------------------------------");
        geometry_msgs::msg::Pose target_pose;
        // Get current end effector pose
        geometry_msgs::msg::Pose current_pose = move_group_arm->getCurrentPose().pose;
        target_pose.position.x = current_pose.position.x;
        target_pose.position.y = current_pose.position.y;
        target_pose.position.z = current_pose.position.z;

        // Set target orientation
        tf2::Quaternion q;
        q.setRPY(roll_deg * M_PI/180.0, pitch_deg * M_PI/180.0, yaw_deg * M_PI/180.0);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        move_group_arm->setPoseTarget(target_pose);

        // Set planning time to 10 seconds
        move_group_arm->setPlanningTime(10.0);
        // Set number of planning attempts to ..
        move_group_arm->setNumPlanningAttempts(20);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group_arm->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success) {
            move_group_arm->execute(my_plan);
        } else {
            RCLCPP_ERROR(LOGGER, "End effector movement plan failed!");
        }
    }

    void move_waypoint(double delta, const std::string& direction) {
        RCLCPP_INFO(LOGGER, "---------------------------MOVE WAYPOINT %s!----------------------------------", direction.c_str());
        
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

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~COMMANDs~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

        // Set planning time to 5 seconds
        move_group_gripper->setPlanningTime(7.0);
        // Set number of planning attempts to ..
        move_group_gripper->setNumPlanningAttempts(15);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
        bool success_gripper = (move_group_gripper->plan(my_plan_gripper) == moveit::core::MoveItErrorCode::SUCCESS);

        if(success_gripper) {
            move_group_gripper->execute(my_plan_gripper);
        } else {
            RCLCPP_ERROR(LOGGER, "Gripper plan failed!");
        }
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~HELPERs~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    void print_end_effector_position() {
        RCLCPP_INFO(LOGGER, "---------------------------END EFFECTOR POSE!----------------------------------");

        geometry_msgs::msg::PoseStamped current_pose = move_group_arm->getCurrentPose();
        RCLCPP_INFO(LOGGER, ">>>>>>>>>>>>>>>>>> Position: (%f, %f, %f)", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
        RCLCPP_INFO(LOGGER, ">>>>>>>>>>>>>>>>>> Quaternion: (%f, %f, %f, %f)", current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
        // Use quat2rpy() to convert quaternion to RPY
        double roll, pitch, yaw;
        std::tie(roll, pitch, yaw) = quat2rpy(current_pose.pose.orientation, true);
        RCLCPP_INFO(LOGGER, ">>>>>>>>>>>>>>>>>> Euler°: (%f, %f, %f)", roll, pitch, yaw);
    }
    
    std::tuple<double, double, double> quat2rpy(geometry_msgs::msg::Quaternion q, bool to_degrees) {
        tf2::Quaternion tf2_quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(tf2_quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        if (to_degrees) {
            roll = roll * 180.0 / M_PI;
            pitch = pitch * 180.0 / M_PI;
            yaw = yaw * 180.0 / M_PI;
        }

        return std::make_tuple(roll, pitch, yaw);
    }
    
    std::vector<double> getArmJointPositions() {
        RCLCPP_INFO(LOGGER, ">>>>>>>>>>>>>>>>>> Current Joint Positions:");
        std::vector<double> joint_values = move_group_arm->getCurrentJointValues();
        for(std::size_t i = 0; i < joint_values.size(); ++i) {
            RCLCPP_INFO(LOGGER, "Joint %zu: %f", i, joint_values[i]);
        }
        // angle_0 = Shoulder Pan
        // angle_1 = Shoulder Lift
        // angle_2 = Elbow
        // angle_3 = Wrist 1
        // angle_4 = Wrist 2
        // angle_5 = Wrist 3
        return joint_values;
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~PROs~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    void move2pos(double absolute_coord, const std::string& axis) {
        RCLCPP_INFO(LOGGER, "---------------------------MOVE 2 POS %s!----------------------------------", axis.c_str());

        std::vector<geometry_msgs::msg::Pose> waypoints;
        // obtain the current position of the end effector
        geometry_msgs::msg::PoseStamped current_pose = move_group_arm->getCurrentPose();
        geometry_msgs::msg::Pose target_pose = move_group_arm->getCurrentPose().pose;

        // calculate the difference depending on the axis
        double difference = 0.0;
        if(axis == "x") {
            difference = absolute_coord - current_pose.pose.position.x;
        } else if(axis == "y") {
            difference = absolute_coord - current_pose.pose.position.y;
        } else if(axis == "z") {
            difference = absolute_coord - current_pose.pose.position.z;
        } else {
            RCLCPP_ERROR(LOGGER, "Invalid axis!");
        }

        // set the step size
        double step_size = 0.01;
        // calculate the number of steps
        int num_steps = std::abs(difference / step_size);

        // move the end effector to the target position in steps
        for (int i = 0; i < num_steps; ++i) {
            if (axis == "x") {
                target_pose.position.x += difference > 0 ? step_size : -step_size;
            } 
            else if (axis == "y") {
                target_pose.position.y += difference > 0 ? step_size : -step_size;
            } 
            else if (axis == "z") {
                target_pose.position.z += difference > 0 ? step_size : -step_size;
            } 
            else {
                RCLCPP_ERROR(LOGGER, "Invalid axis: %s", axis.c_str());
                return;
            }
            waypoints.push_back(target_pose);
        }

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
    
    //SCENE
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Create vector to hold collision objects.
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;


    // create the wall
    auto wall_object = robotArm.createCollisionObject(
        "wall", 
        "base_link",
        robotArm.createSolidPrimitiveBOX(2.0, 0.1, 2.0),
        robotArm.createPose(-0.25,-0.4,0, 1.0)
    );
    // create the table
    auto table_object = robotArm.createCollisionObject(
        "table", 
        "base_link",
        robotArm.createSolidPrimitiveBOX(0.85,1.8,1.0),
        robotArm.createPose(0.3,0.35,-0.501, 1.0)
    );
    // create machine
    auto machine_object = robotArm.createCollisionObject(
        "machine", 
        "base_link",
        robotArm.createSolidPrimitiveBOX(0.6,0.15,0.4),
        robotArm.createPose(0.2,0.85,0.2, 1.0)
    );


    // push the objects into the vector
    collision_objects.push_back(wall_object);
    collision_objects.push_back(table_object);
    collision_objects.push_back(machine_object);
    planning_scene_interface.addCollisionObjects(collision_objects);
    // Wait for MoveGroup to recieve and process the collision object message.
    rclcpp::sleep_for(std::chrono::seconds(2));

    robotArm.cmd_arm("crab");
    float cube_pos_x_ = 0.310;
    float cube_pos_y_ = 0.340;
    // read the atributte 'is_robot_sim' of the object
    if(robotArm.get_is_robot_sim()) {
        RCLCPP_INFO(LOGGER, "\n\n\n Robot is simulated \n\n\n");
        // robotArm.move_end_effector(cube_pos_x_, cube_pos_y_, 0.35, -180.0, 0.0, 0.0);
        robotArm.print_end_effector_position();
        // robotArm.orientation_end_effector(-180.0, 0.0, 0.0);
        robotArm.move2pos(0.35, "z");
        robotArm.move2pos(cube_pos_x_, "x");
        robotArm.move2pos(cube_pos_y_, "y");
        robotArm.cmd_gripper("gripper_open");
        robotArm.print_end_effector_position();
        
        robotArm.move_waypoint(-0.05, "z");
        robotArm.cmd_gripper("gripper_close");
        //robotArm.move_gripper_space(0.69);
        robotArm.move2pos(0.20, "z");
        // robotArm.move_waypoint(0.10, "z");
        robotArm.move_single_joint("shoulder_pan_joint", 180.0);

        // robotArm.move_end_effector(-0.5, -0.02, -0.15, -180.0, 0.0, 0.0);
        robotArm.move2pos(-0.5, "x");
        robotArm.move2pos(-0.02, "y");
        robotArm.move2pos(-0.15, "z");
        robotArm.print_end_effector_position();
        robotArm.cmd_gripper("gripper_open");
    } 
    else {
        RCLCPP_INFO(LOGGER, "\n\n\n Robot is real \n\n\n");
        robotArm.move_end_effector(cube_pos_x_, cube_pos_y_, 0.25, -180.0, 0.0, 0.0);
        robotArm.cmd_gripper("gripper_open");
        robotArm.print_end_effector_position();
        
        robotArm.move_waypoint(-0.08, "z");
        //robotArm.cmd_gripper("gripper_close");
        robotArm.move_gripper_space(0.69);

        robotArm.move_waypoint(0.10, "z");

        robotArm.move_joint_space(6.1, -1.64, -1.59, -1.47, 1.56, -5.70);
        robotArm.cmd_gripper("gripper_open");
    }
    robotArm.cmd_arm("home");

    rclcpp::shutdown();
    return 0;
}