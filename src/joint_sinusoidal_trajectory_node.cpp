//~ Copyright (C) 2023 Łukasz Woliński
//~ You may use, distribute and modify this code under the terms of the BSD-3-Clause License.

#include <iostream>
#include <memory>
#include <thread>
#include <string>
#include <eigen3/Eigen/Dense>
#include "trajectory_generator/joint_sinusoidal_trajectory.hpp"

#include "rrlib_interfaces/action/ptp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class JointSinusoidalTrajectoryNode : public rclcpp::Node
{
public:
    using PTP = rrlib_interfaces::action::PTP;
    using GoalHandlePTP = rclcpp_action::ServerGoalHandle<PTP>;
    JointSinusoidalTrajectoryNode();
    
private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr publisher_;
    rclcpp_action::Server<PTP>::SharedPtr action_server_;
    
    rrlib::JointSinusoidalTrajectory trajectory_;
    
    rclcpp_action::GoalResponse HandleGoal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const PTP::Goal> goal);
        
    rclcpp_action::CancelResponse HandleCancel(
        const std::shared_ptr<GoalHandlePTP> goal_handle);
    
    void HandleAccepted(const std::shared_ptr<GoalHandlePTP> goal_handle);
    
    void Execute(const std::shared_ptr<GoalHandlePTP> goal_handle);
};

JointSinusoidalTrajectoryNode::JointSinusoidalTrajectoryNode()
: Node("joint_sinusoidal_trajectory")
{
    using namespace std::placeholders;
    
    RCLCPP_INFO(this->get_logger(), "Starting the node.");
    
    // create a publisher for the joint trajectory point
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("jnt_sin_traj", 10);
    
    // create an action server to handle requests for the joint trajectory generation
    this->action_server_ = rclcpp_action::create_server<PTP>(
        this,
        "ptp_motion",
        std::bind(&JointSinusoidalTrajectoryNode::HandleGoal, this, _1, _2),
        std::bind(&JointSinusoidalTrajectoryNode::HandleCancel, this, _1),
        std::bind(&JointSinusoidalTrajectoryNode::HandleAccepted, this, _1));
}

rclcpp_action::GoalResponse JointSinusoidalTrajectoryNode::HandleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PTP::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received the PTP motion request");
    (void)uuid;
    
    size_t DOF;
    Eigen::VectorXd q_start, q_end;
    double vel_max, acc_max;
    
    if (goal->dt <= 0.0 or goal->vel_max <= 0.0 or goal->acc_max <= 0.0)
    {
        RCLCPP_INFO(this->get_logger(), "Request rejected. The values of vel_max, acc_max, and dt shall be greater than 0.0.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    else
    {
        vel_max = goal->vel_max;
        acc_max = goal->acc_max;
        // dt is not assigned to a local variable, because it is not passed to trajectory_.SetParameters function
        // dt is assigned to a local variable in the Execute function
    }
    
    if (goal->start_position.size() != goal->end_position.size())
    {
        RCLCPP_INFO(this->get_logger(), "Request rejected. The vectors q_start and q_end shall have the same size.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    else
    {
        DOF = goal->start_position.size();
        q_start = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(goal->start_position.data(), DOF);
        q_end = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(goal->end_position.data(), DOF);
    }
    
    trajectory_.SetParameters(q_start, q_end, DOF, vel_max, acc_max);
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointSinusoidalTrajectoryNode::HandleCancel(
    const std::shared_ptr<GoalHandlePTP> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void JointSinusoidalTrajectoryNode::HandleAccepted(const std::shared_ptr<GoalHandlePTP> goal_handle)
{
    using namespace std::placeholders;
    // according to the tutorial, the callback for accepting the goal needs to return quickly to avoid blocking the executor, therefore a new thread is required
    std::thread{std::bind(&JointSinusoidalTrajectoryNode::Execute, this, _1), goal_handle}.detach();
}
    
void JointSinusoidalTrajectoryNode::Execute(const std::shared_ptr<GoalHandlePTP> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Generating the trajectory.");
    const auto goal = goal_handle->get_goal();
    double dt = goal->dt;                       // time increment
    double t = 0.0;                             // starting time
    double t_end = trajectory_.GetMotionTime(); // end time
    int N = (int) ceil(t_end / dt);             // number of steps
    
    rclcpp::Rate loop_rate(1.0/dt);
    auto feedback = std::make_shared<PTP::Feedback>();
    auto result = std::make_shared<PTP::Result>();

    for (int k = 1; (k < N) && rclcpp::ok(); ++k)
    {
        // Check if there is a cancel request
        if (goal_handle->is_canceling())
        {
            result->success = false;
            result->message = "Trajectory cancelled.";
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled.");
            return;
        }
        
        // compute the joint positions, velocities, and accelerations (the trajectory point):
        Eigen::VectorXd q;
        Eigen::VectorXd dqdt;
        Eigen::VectorXd d2qdt2;
        trajectory_.PositionVelocityAcceleration(t, &q, &dqdt, &d2qdt2);
        
        // publish the computed joint trajectory point
        trajectory_msgs::msg::JointTrajectoryPoint response;
        std::vector<double> positions(q.data(), q.data() + q.size());
        response.positions = positions;
        std::vector<double> velocities(dqdt.data(), dqdt.data() + dqdt.size());
        response.velocities = velocities;
        std::vector<double> accelerations(d2qdt2.data(), d2qdt2.data() + d2qdt2.size());
        response.accelerations = accelerations;
        response.time_from_start.sec = (int) floor(t);
        response.time_from_start.nanosec = (int) ( (t - floor(t)) * pow(10.0, 9.0) ); // TODO: more legit way?
        publisher_->publish(response);
        
        // Publish feedback
        feedback->percent_complete = t / trajectory_.GetMotionTime() * 100.0;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Trajectory percent complete: %lf", feedback->percent_complete);
        
        t += dt;

        loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok())
    {
        result->success = true;
        result->message = "Trajectory finished.";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointSinusoidalTrajectoryNode>());
    rclcpp::shutdown();
    return 0;
}
