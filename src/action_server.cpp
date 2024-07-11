#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <rclcpp_action/rclcpp_action.hpp> // for action server
#include "robot_ur3e_manipulation/action/deliver_coffee_action.hpp"
// # goal definition
// bool coffe_order
// ---
// # result definition
// bool order_success
// ---
// # feedback definition
// string order_status

using CoffeeAction = robot_ur3e_manipulation::action::DeliverCoffeeAction;
using GoalHandleCoffee = rclcpp_action::ServerGoalHandle<CoffeeAction>;
using std::placeholders::_1;
using std::placeholders::_2;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";
static const rclcpp::Logger ACTION = rclcpp::get_logger("action_server");

class RobotArm {
public:
    RobotArm(rclcpp::Node::SharedPtr node) : node_(node){
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

        executor.add_node(move_group_node);
        executor.add_node(node_);
        //std::thread([this]() { this->executor.spin(); }).detach();
        executor_thread = std::thread([this]() { this->executor.spin(); });
        executor_thread.detach();

        move_group_arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, PLANNING_GROUP_ARM);
        joint_model_group_arm = move_group_arm->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
        
        move_group_gripper = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, PLANNING_GROUP_GRIPPER);
        joint_model_group_gripper = move_group_gripper->getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);
        
        // create a subscriber to the topic: /cup_abs_position :which has Point info
        sub_cup_abs_position = node_->create_subscription<geometry_msgs::msg::Point>(
            "/cup_abs_position", 10, std::bind(&RobotArm::clbk_cup_abs_position, this, std::placeholders::_1)
        );

        // create an action server: robot_ur3e_manipulation_as
        action_server_ = rclcpp_action::create_server<CoffeeAction>(
            node_,
            "robot_ur3e_manipulation_as",
            std::bind(&RobotArm::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RobotArm::handle_cancel, this, std::placeholders::_1),
            std::bind(&RobotArm::handle_accepted, this, std::placeholders::_1)
        );

        node_->declare_parameter<bool>("is_robot_sim", false);
        RCLCPP_INFO(LOGGER, "---------------------------INSTANTIATED!----------------------------------");
        start_moveit_scene();
    }
    
    ~RobotArm() {
        RCLCPP_INFO(ACTION, "COFFEE PROGRAM IS SHUTING DOWN");
        executor.cancel();
        if (executor_thread.joinable()) {
            executor_thread.join();
        }
    }

    bool get_is_robot_sim() {
        return node_->get_parameter("is_robot_sim").as_bool();
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~SCENE OBJECTs~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    void start_moveit_scene() {
        RCLCPP_INFO(LOGGER, "---------------------------STARTing SCENE!----------------------------------");
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        // Clear the scene
        std::vector<std::string> object_ids = {"wall", "table", "machine"};
        planning_scene_interface.removeCollisionObjects(object_ids);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        // Create vector to hold collision objects.
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    
        
        // create the wall
        auto wall_object = createCollisionObject(
            "wall", 
            "base_link",
            createSolidPrimitiveBOX(2.0, 0.1, 2.0),
            createPose(-0.25,-0.4,0, 1.0)
        );
        // create the table
        auto table_object = createCollisionObject(
            "table", 
            "base_link",
            createSolidPrimitiveBOX(0.85,1.8,1.0),
            createPose(0.3,0.35,-0.501, 1.0)
        );
        // create machine
        auto machine_object = createCollisionObject(
            "machine", 
            "base_link",
            createSolidPrimitiveBOX(0.6,0.15,0.4),
            createPose(0.2,0.85,0.2, 1.0)
        );


        // push the objects into the vector
        collision_objects.push_back(wall_object);
        collision_objects.push_back(table_object);
        collision_objects.push_back(machine_object);
        planning_scene_interface.addCollisionObjects(collision_objects);
        // Wait for MoveGroup to recieve and process the collision object message.
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

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

    void removeCollisionObjects(const std::vector<std::string>& object_ids) {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        planning_scene_interface.removeCollisionObjects(object_ids);
        RCLCPP_INFO(LOGGER, "Removed specified objects from the scene.");
    }
    
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~CUP POSITION~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    void clbk_cup_abs_position(const geometry_msgs::msg::Point::SharedPtr msg) {
        //RCLCPP_INFO(LOGGER, "I heard: 'Cup position: (%.3f, %.3f, %.3f)'", msg->x, msg->y, msg->z);
        cube_pos_x_ = msg->x;
        cube_pos_y_ = msg->y;
        cube_pos_z_ = msg->z;
    }

    // tuple for getting the cup position
    std::tuple<float, float, float> get_cup_position() {
        return std::make_tuple(cube_pos_x_, cube_pos_y_, cube_pos_z_);
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ACTION SERVER~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~    

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CoffeeAction::Goal> goal) {
        RCLCPP_INFO(ACTION, "COFFEE - Received goal request with bool value: %s", goal->coffe_order ? "true" : "false");
        (void)uuid;
        if(goal->coffe_order) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } 
        else {
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCoffee> goal_handle) {
        RCLCPP_INFO(ACTION, "COFFEE - Received request to cancel goal");
        (void)goal_handle;
        // accept all cancel requests
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleCoffee> goal_handle) {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&RobotArm::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleCoffee> goal_handle) {
        RCLCPP_INFO(ACTION, "COFFEE - Executing goal");
        auto feedback = std::make_shared<CoffeeAction::Feedback>();
        auto result = std::make_shared<CoffeeAction::Result>();
        
        if(get_is_robot_sim()) {
            RCLCPP_INFO(ACTION, "\n\n\n Robot is simulated \n\n\n");
            move_gripper_space(-0.17);
            feedback->order_status = "COFFEE -> STAGE 1!"; goal_handle->publish_feedback(feedback);
            feedback->order_status = "COFFEE -> Moving to crab position"; goal_handle->publish_feedback(feedback);
            cmd_arm("crab_pose");
            // move_joint_space(-0.000045, -2.144726, 1.697500, -1.123655, -1.570612, -1.570653);
            
            // feedback->order_status = "COFFEE -> Moving to x=0.31"; goal_handle->publish_feedback(feedback);
            // move2pos(0.31, "x");
            // feedback->order_status = "COFFEE -> Moving to y=0.34"; goal_handle->publish_feedback(feedback);
            // move2pos(0.34, "y");
            feedback->order_status = "COFFEE -> Moving to x=0.31 y=0.34 z=0.35"; goal_handle->publish_feedback(feedback);
            move_with_orientation_constraint(0.31, 0.34, 0.35);
            feedback->order_status = "COFFEE -> Opening gripper"; goal_handle->publish_feedback(feedback);
            cmd_gripper("gripper_open");
            print_end_effector_position();

            feedback->order_status = "COFFEE -> STAGE 2!"; goal_handle->publish_feedback(feedback);
            // feedback->order_status = "COFFEE -> Moving to x=0.31 y=0.34 z=0.26"; goal_handle->publish_feedback(feedback);
            // move_with_orientation_constraint(0.31, 0.34, 0.26);            
            feedback->order_status = "COFFEE -> Moving to z=0.26"; goal_handle->publish_feedback(feedback);
            move2pos(0.26, "z");            
            feedback->order_status = "COFFEE -> Closing gripper"; goal_handle->publish_feedback(feedback);
            // move_gripper_space(-0.17);
            cmd_gripper("gripper_close");
            // feedback->order_status = "COFFEE -> Moving to x=0.31 y=0.34 z=0.35"; goal_handle->publish_feedback(feedback);
            // move_with_orientation_constraint(0.31, 0.34, 0.35);            
            feedback->order_status = "COFFEE -> Moving to z=0.35"; goal_handle->publish_feedback(feedback);
            move2pos(0.35, "z");            
            feedback->order_status = "COFFEE -> Moving to shoulder_pan_joint=135.0"; goal_handle->publish_feedback(feedback);
            move_single_joint("shoulder_pan_joint", 135.0);

            float cup_pos_x, cup_pos_y, cup_pos_z;
            std::tie(cup_pos_x, cup_pos_y, cup_pos_z) = get_cup_position();
            feedback->order_status = "COFFEE -> STAGE 3!"; goal_handle->publish_feedback(feedback);
            std::string cube_pos_x_str = std::to_string(cup_pos_x-0.015).substr(0, std::to_string(cup_pos_x-0.015).find(".") + 4);
            std::string cube_pos_y_str = std::to_string(cup_pos_y).substr(0, std::to_string(cup_pos_y).find(".") + 4);
            std::string cube_pos_z_str = std::to_string(cup_pos_z+0.30).substr(0, std::to_string(cup_pos_z+0.30).find(".") + 4);
            feedback->order_status = "COFFEE -> Delivering to point (" + cube_pos_x_str + ", " + cube_pos_y_str + ", " + cube_pos_z_str + ")"; goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(ACTION, "COFFEE - Delivering to point (%s, %s, %s)", cube_pos_x_str.c_str(), cube_pos_y_str.c_str(), cube_pos_z_str.c_str());
            move_with_orientation_constraint(cup_pos_x-0.015, cup_pos_y, cup_pos_z+0.30);
            print_end_effector_position();
            move_with_orientation_constraint(cup_pos_x-0.015, cup_pos_y, cup_pos_z+0.27);            
            // feedback->order_status = "COFFEE -> Moving to y=cup_pos_y"; goal_handle->publish_feedback(feedback);
            // move2pos(cup_pos_y, "y");
            // feedback->order_status = "COFFEE -> Moving to z=cup_pos_z+0.33"; goal_handle->publish_feedback(feedback);
            // move2pos(cup_pos_z+0.33, "z");
            // feedback->order_status = "COFFEE -> Moving to x=cup_pos_x"; goal_handle->publish_feedback(feedback);
            // move2pos(cup_pos_x, "x");
            feedback->order_status = "COFFEE -> Opening gripper"; goal_handle->publish_feedback(feedback);
            move_gripper_space(0.4);
            cmd_gripper("gripper_open");
            move_with_orientation_constraint(cup_pos_x-0.015, cup_pos_y, cup_pos_z+0.30);

            // Check if goal is succeeded or aborted
            if (rclcpp::ok()) {
                result->order_success = true;
                RCLCPP_INFO(ACTION, "COFFEE - Goal succeeded\n");
                goal_handle->succeed(result);
            }
            cmd_arm("home");
        } 
        else {
            RCLCPP_INFO(ACTION, "\n\n\n Robot is real \n\n\n");
            move_gripper_space(0.0);
            feedback->order_status = "COFFEE -> STAGE 1!"; goal_handle->publish_feedback(feedback);
            feedback->order_status = "COFFEE -> Moving to crab position"; goal_handle->publish_feedback(feedback);
            cmd_arm("crab_pose");
            // move_joint_space(-0.000045, -2.144726, 1.697500, -1.123655, -1.570612, -1.570653);
            
            // feedback->order_status = "COFFEE -> Moving to x=0.21"; goal_handle->publish_feedback(feedback);
            // move2pos(0.21, "x");
            // feedback->order_status = "COFFEE -> Moving to y=0.34"; goal_handle->publish_feedback(feedback);
            // move2pos(0.34, "y");
            feedback->order_status = "COFFEE -> Moving to x=0.21 y=0.34 z=0.35"; goal_handle->publish_feedback(feedback);
            move_with_orientation_constraint(0.21, 0.34, 0.35);
            feedback->order_status = "COFFEE -> Opening gripper"; goal_handle->publish_feedback(feedback);
            cmd_gripper("gripper_open");
            print_end_effector_position();

            feedback->order_status = "COFFEE -> STAGE 2!"; goal_handle->publish_feedback(feedback);
            feedback->order_status = "COFFEE -> Moving to x=0.21 y=0.34 z=0.28"; goal_handle->publish_feedback(feedback);
            move_with_orientation_constraint(0.21, 0.34, 0.28);
            // feedback->order_status = "COFFEE -> Moving to z=0.28"; goal_handle->publish_feedback(feedback);
            // move2pos(0.28, "z");            
            feedback->order_status = "COFFEE -> Closing gripper"; goal_handle->publish_feedback(feedback);
            move_gripper_space(0.0);
            feedback->order_status = "COFFEE -> Moving to x=0.21 y=0.34 z=0.35"; goal_handle->publish_feedback(feedback);
            move_with_orientation_constraint(0.21, 0.34, 0.35);
            // feedback->order_status = "COFFEE -> Moving to z=0.35"; goal_handle->publish_feedback(feedback);
            // move2pos(0.35, "z");
            feedback->order_status = "COFFEE -> Moving to shoulder_pan_joint=135.0"; goal_handle->publish_feedback(feedback);
            move_single_joint("shoulder_pan_joint", 135.0);

            float cup_pos_x, cup_pos_y, cup_pos_z;
            std::tie(cup_pos_x, cup_pos_y, cup_pos_z) = get_cup_position();
            feedback->order_status = "COFFEE -> STAGE 3!"; goal_handle->publish_feedback(feedback);
            std::string cube_pos_x_str = std::to_string(cup_pos_x).substr(0, std::to_string(cup_pos_x).find(".") + 4);
            std::string cube_pos_y_str = std::to_string(cup_pos_y).substr(0, std::to_string(cup_pos_y).find(".") + 4);
            std::string cube_pos_z_str = std::to_string(cup_pos_z+0.33).substr(0, std::to_string(cup_pos_z+0.33).find(".") + 4);
            feedback->order_status = "COFFEE -> Delivering to point (" + cube_pos_x_str + ", " + cube_pos_y_str + ", " + cube_pos_z_str + ")"; goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(ACTION, "COFFEE - Delivering to point (%.3f, %.3f, %.3f)", cup_pos_x, cup_pos_y, cup_pos_z+0.33);
            move_with_orientation_constraint(cup_pos_x, cup_pos_y, cup_pos_z+0.33);
            print_end_effector_position();
            move_with_orientation_constraint(cup_pos_x, cup_pos_y, cup_pos_z+0.30);
            // feedback->order_status = "COFFEE -> Moving to y=cup_pos_y"; goal_handle->publish_feedback(feedback);
            // move2pos(cup_pos_y, "y");
            // feedback->order_status = "COFFEE -> Moving to z=cup_pos_z+0.33"; goal_handle->publish_feedback(feedback);
            // move2pos(cup_pos_z+0.33, "z");
            // feedback->order_status = "COFFEE -> Moving to x=cup_pos_x"; goal_handle->publish_feedback(feedback);
            // move2pos(cup_pos_x, "x");
            feedback->order_status = "COFFEE -> Opening gripper"; goal_handle->publish_feedback(feedback);
            cmd_gripper("gripper_open");
            move_with_orientation_constraint(cup_pos_x, cup_pos_y, cup_pos_z+0.33);

            // Check if goal is succeeded or aborted
            if (rclcpp::ok()) {
                result->order_success = true;
                RCLCPP_INFO(ACTION, "COFFEE - Goal succeeded\n");
                goal_handle->succeed(result);
            }
            cmd_arm("home");
        }
        
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

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~COMMANDs~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    void cmd_arm(const std::string& target_name) {
        RCLCPP_INFO(LOGGER, "---------------------------ARM COMMAND!----------------------------------");
        RCLCPP_INFO(LOGGER, target_name.c_str());

        move_group_arm->setNamedTarget(target_name);

        // Set a longer execution timeout (e.g., 30 seconds)
        move_group_gripper->setGoalTolerance(30.0);

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
        RCLCPP_INFO(LOGGER, ">>>>>>>>>>>>>>>>>> Position: (%.3f, %.3f, %.3f)", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
        RCLCPP_INFO(LOGGER, ">>>>>>>>>>>>>>>>>> Quaternion: (%.3f, %.3f, %.3f, %.3f)", current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
        // Use quat2rpy() to convert quaternion to RPY
        double roll, pitch, yaw;
        std::tie(roll, pitch, yaw) = quat2rpy(current_pose.pose.orientation, true);
        RCLCPP_INFO(LOGGER, ">>>>>>>>>>>>>>>>>> Euler°: (%.3f, %.3f, %.3f)", roll, pitch, yaw);
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
    
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~PROs~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    void move_with_orientation_constraint(double x, double y, double z) {
        moveit_msgs::msg::OrientationConstraint ocm; // create an orientation constraint message
        ocm.link_name = "hand_ee";
        ocm.header.frame_id = "base_link";
        ocm.orientation.x = 1.0;
        ocm.orientation.y = 0.0;
        ocm.orientation.z = 0.0;
        ocm.orientation.w = 0.0;
        ocm.absolute_x_axis_tolerance = 0.05;
        ocm.absolute_y_axis_tolerance = 0.05;
        ocm.absolute_z_axis_tolerance = 0.05;
        ocm.weight = 1.0;                          // set the importance
        moveit_msgs::msg::Constraints constraints; // create a constraint message
        constraints.orientation_constraints.push_back(ocm); // add the orientation constraint to the constraint message
        RCLCPP_INFO(LOGGER, "---------------------------MOVE WITH ORIENTATION "
                            "CONSTRAINT!----------------------------------");

        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose target_pose = move_group_arm->getCurrentPose().pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_arm->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory, constraints);

        if (fraction >= 0.0) {
            move_group_arm->execute(trajectory);
        } else {
            RCLCPP_ERROR(LOGGER, "Waypoint movement plan failed!");
        }
    }    

    void move2pos(double absolute_coord, const std::string& axis) {
        RCLCPP_INFO(LOGGER, "---------------------------MOVE 2 POS %s!----------------------------------", axis.c_str());
    
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double step_size = 0.01;
        double fraction = 0.0;
    
        do {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            geometry_msgs::msg::PoseStamped current_pose = move_group_arm->getCurrentPose();
            geometry_msgs::msg::Pose target_pose = current_pose.pose;
    
            double difference = 0.0;
            if(axis == "x") {
                difference = absolute_coord - current_pose.pose.position.x;
            } else if(axis == "y") {
                difference = absolute_coord - current_pose.pose.position.y;
            } else if(axis == "z") {
                difference = absolute_coord - current_pose.pose.position.z;
            } else {
                RCLCPP_ERROR(LOGGER, "Invalid axis!");
                return;
            }
    
            int num_steps = std::abs(difference / step_size);
    
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
            fraction = move_group_arm->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
            if(fraction >= 0.0) {
                move_group_arm->execute(trajectory);
                // Add a delay here to ensure the robot has enough time to finish executing the trajectory
                rclcpp::sleep_for(std::chrono::seconds(1));
            } else {
                RCLCPP_ERROR(LOGGER, "Waypoint movement plan failed!");
            }
        } while (fraction < 1.0);
    }

private:
    rclcpp::Node::SharedPtr move_group_node;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm;
    const moveit::core::JointModelGroup* joint_model_group_arm;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper;
    const moveit::core::JointModelGroup* joint_model_group_gripper;
    rclcpp::executors::MultiThreadedExecutor executor;
    std::thread executor_thread;
    bool is_robot_sim;
    rclcpp::Node::SharedPtr node_;
    // for cup position
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_cup_abs_position;
    float cube_pos_x_;
    float cube_pos_y_;
    float cube_pos_z_;
    // for action server
    rclcpp_action::Server<CoffeeAction>::SharedPtr action_server_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // create an instance of rclcpp:Node
    auto node = std::make_shared<rclcpp::Node>("for_check_arguments");
    RobotArm robotArm(node);

    while(rclcpp::ok()) {
        // wait some time in ms
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }

    rclcpp::shutdown();
    return 0;
}
