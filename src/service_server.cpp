#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <std_msgs/msg/string.hpp>// we are going to emulate an action, so we need a feedback string
#include "robot_ur3e_manipulation/srv/deliver_coffee_service.hpp"
// # goal definition
// bool coffe_order
// ---
// # result definition
// bool order_success

using CoffeeService = robot_ur3e_manipulation::srv::DeliverCoffeeService;
using std::placeholders::_1;
using std::placeholders::_2;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";
static const rclcpp::Logger SERVICE = rclcpp::get_logger("service_server");

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

        // create an service server: robot_ur3e_manipulation_ss
        service_server_ = node_->create_service<CoffeeService>(
            "robot_ur3e_manipulation_ss", std::bind(&RobotArm::handle_service, this, _1, _2)
        );
        // feedback topic
        feedback_pub_ = node_->create_publisher<std_msgs::msg::String>("robot_ur3e_manipulation_feedback", 10);

        node_->declare_parameter<bool>("is_robot_sim", false);
        RCLCPP_INFO(LOGGER, "---------------------------INSTANTIATED!----------------------------------");
    }
    
    ~RobotArm() {
        RCLCPP_INFO(SERVICE, "COFFEE PROGRAM IS SHUTING DOWN");
        executor.cancel();
        if (executor_thread.joinable()) {
            executor_thread.join();
        }
    }

    bool get_is_robot_sim() {
        return node_->get_parameter("is_robot_sim").as_bool();
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

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~SERVICE SERVER~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    void handle_service(const std::shared_ptr<CoffeeService::Request> request, std::shared_ptr<CoffeeService::Response> response) {
        RCLCPP_INFO(SERVICE, "COFFEE - Received request with bool value: %s", request->coffe_order ? "true" : "false");

        if(request->coffe_order) {
            move_gripper_space(-0.17);
            RCLCPP_INFO(SERVICE, "COFFEE - Executing goal");
            
            send_feedback("COFFEE -> STAGE 1!");
            send_feedback("COFFEE -> Moving to crab position");
            // cmd_arm("crab_pose");
            move_joint_space(-0.000045, -2.144726, 1.697500, -1.123655, -1.570612, -1.570653);
            send_feedback("COFFEE -> Moving to x=0.31");
            move2pos(0.31, "x");
            send_feedback("COFFEE -> Moving to y=0.34");
            move2pos(0.34, "y");
            send_feedback("COFFEE -> Opening gripper");
            cmd_gripper("gripper_open");
            print_end_effector_position();

            send_feedback("COFFEE -> STAGE 2!");
            send_feedback("COFFEE -> Moving to z=0.26");
            move2pos(0.26, "z");
            send_feedback("COFFEE -> Closing gripper");
            move_gripper_space(-0.17);
            send_feedback("COFFEE -> Moving to z=0.35");
            move2pos(0.35, "z");
            send_feedback("COFFEE -> Moving to shoulder_pan_joint=135.0");
            move_single_joint("shoulder_pan_joint", 135.0);

            send_feedback("COFFEE -> STAGE 3!");
            std::string cube_pos_x_str = std::to_string(cube_pos_x_).substr(0, std::to_string(cube_pos_x_).find(".") + 4);
            std::string cube_pos_y_str = std::to_string(cube_pos_y_).substr(0, std::to_string(cube_pos_y_).find(".") + 4);
            std::string cube_pos_z_str = std::to_string(cube_pos_z_).substr(0, std::to_string(cube_pos_z_).find(".") + 4);
            send_feedback("COFFEE -> Delivering to point (" + cube_pos_x_str + ", " + cube_pos_y_str + ", " + cube_pos_z_str + ")");
            send_feedback("COFFEE -> Moving to y=cube_pos_y_");
            move2pos(cube_pos_y_, "y");
            send_feedback("COFFEE -> Moving to z=cube_pos_z_+0.33");
            move2pos(cube_pos_z_+0.33, "z");
            send_feedback("COFFEE -> Moving to x=cube_pos_x_");
            move2pos(cube_pos_x_, "x");
            send_feedback("COFFEE -> Opening gripper");
            cmd_gripper("gripper_open");
            print_end_effector_position();
            
            if (rclcpp::ok()) {
                response->order_success = true;
                RCLCPP_INFO(SERVICE, "COFFEE - Order succeeded\n");
            }
            cmd_arm("home");
        }
        else
        {
            response->order_success = false;
        }
        
    }

    void send_feedback(const std::string& feedback) {
        auto feedback_msg = std::make_shared<std_msgs::msg::String>();
        feedback_msg->data = feedback;
        feedback_pub_->publish(*feedback_msg);
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
    // for service server
    rclcpp::Service<CoffeeService>::SharedPtr service_server_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_pub_;
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
