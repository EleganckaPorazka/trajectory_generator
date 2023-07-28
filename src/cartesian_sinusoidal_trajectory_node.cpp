//~ Copyright (C) 2023 Łukasz Woliński
//~ You may use, distribute and modify this code under the terms of the BSD-3-Clause License.

#include <iostream>
#include <memory>
#include <thread>
#include <string>
#include <eigen3/Eigen/Dense>
#include "trajectory_generator/cartesian_sinusoidal_trajectory.hpp"

#include "rrlib_interfaces/action/cart.hpp"
#include "rrlib_interfaces/msg/cartesian_trajectory_point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace rrlib
{
class CartesianSinusoidalTrajectoryNode : public rclcpp::Node
{
public:
    using CART = rrlib_interfaces::action::CART;
    using GoalHandleCART = rclcpp_action::ServerGoalHandle<CART>;
    CartesianSinusoidalTrajectoryNode();
    
private:
    rclcpp::Publisher<rrlib_interfaces::msg::CartesianTrajectoryPoint>::SharedPtr publisher_;
    rclcpp_action::Server<CART>::SharedPtr action_server_;
    
    CartesianSinusoidalTrajectory trajectory_;
    
    rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const CART::Goal> goal);
        
    rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<GoalHandleCART> goal_handle);
    
    void HandleAccepted(const std::shared_ptr<GoalHandleCART> goal_handle);
    
    void Execute(const std::shared_ptr<GoalHandleCART> goal_handle);
};

CartesianSinusoidalTrajectoryNode::CartesianSinusoidalTrajectoryNode()
: Node("cartesian_sinusoidal_trajectory")
{
    using namespace std::placeholders;
    
    RCLCPP_INFO(this->get_logger(), "Starting the node.");
    
    // create a publisher for the joint trajectory point
    publisher_ = this->create_publisher<rrlib_interfaces::msg::CartesianTrajectoryPoint>("cart_sin_traj", 10);
    
    // create an action server to handle requests for the joint trajectory generation
    this->action_server_ = rclcpp_action::create_server<CART>(
        this,
        "cartesian_motion",
        std::bind(&CartesianSinusoidalTrajectoryNode::HandleGoal, this, _1, _2),
        std::bind(&CartesianSinusoidalTrajectoryNode::HandleCancel, this, _1),
        std::bind(&CartesianSinusoidalTrajectoryNode::HandleAccepted, this, _1));
}

rclcpp_action::GoalResponse CartesianSinusoidalTrajectoryNode::HandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const CART::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received the Cartesian space motion request.");
    (void)uuid;
    
    if (goal->motion_type != "LIN" or goal->motion_type != "CIRC")
    {
        RCLCPP_INFO(this->get_logger(), "Request rejected. The supported motion types are 'LIN' and 'CIRC'.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
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
    
    Eigen::VectorXd x_start, x_end, x_aux;
    if (goal->start_pose.size() != 7)
    {
        RCLCPP_INFO(this->get_logger(), "Request rejected. The vector x_start shall have 7 elements.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    else
    {
        x_start = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(goal->start_pose.data(), 7);
    }
    
    if (goal->end_pose.size() != 7)
    {
        RCLCPP_INFO(this->get_logger(), "Request rejected. The vectors x_end shall have 7 elements.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    else
    {
        x_end = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(goal->end_pose.data(), 7);
    }
    
    if (goal->motion_type == "LIN")
    {
        x_aux = Eigen::VectorXd::Zero(7); // in LIN motion we will not use it, but initialize it to zeros just in case
        trajectory_.SetParameters(x_start, x_end, vel_max, acc_max);
    }
    else if (goal->motion_type == "CIRC")
    {
        if (goal->aux_pose.size() != 7)
        {
            RCLCPP_INFO(this->get_logger(), "Request rejected. The vectors x_end shall have 7 elements.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        else
        {
            x_aux = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(goal->aux_pose.data(), 7);
        }
        trajectory_.SetParameters(x_start, x_end, vel_max, acc_max);
        //TODO:
        //trajectory_.SetParameters(x_start, x_end, x_aux, vel_max, acc_max);
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CartesianSinusoidalTrajectoryNode::HandleCancel(const std::shared_ptr<GoalHandleCART> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CartesianSinusoidalTrajectoryNode::HandleAccepted(const std::shared_ptr<GoalHandleCART> goal_handle)
{
    using namespace std::placeholders;
    // according to the tutorial, the callback for accepting the goal needs to return quickly to avoid blocking the executor, therefore a new thread is required
    std::thread{std::bind(&CartesianSinusoidalTrajectoryNode::Execute, this, _1), goal_handle}.detach();
}
    
void CartesianSinusoidalTrajectoryNode::Execute(const std::shared_ptr<GoalHandleCART> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Generating the trajectory.");
    const auto goal = goal_handle->get_goal();
    double dt = goal->dt;                       // time increment
    double t = 0.0;                             // starting time
    double t_end = trajectory_.GetMotionTime(); // end time
    int N = (int) ceil(t_end / dt) + 1;         // number of steps
    
    rclcpp::Rate loop_rate(1.0/dt);
    auto feedback = std::make_shared<CART::Feedback>();
    auto result = std::make_shared<CART::Result>();

    for (int k = 0; (k < N) && rclcpp::ok(); ++k)
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
        
        // compute the end effector positions, velocities, and accelerations:
        Eigen::VectorXd x;
        Eigen::VectorXd v;
        Eigen::VectorXd dvdt;
        trajectory_.PositionVelocityAcceleration(t, &x, &v, &dvdt);
        
        // publish the computed end effector trajectory point
        rrlib_interfaces::msg::CartesianTrajectoryPoint response;
        std::vector<double> positions(x.data(), x.data() + x.size());
        response.positions = positions;
        std::vector<double> velocities(v.data(), v.data() + v.size());
        response.velocities = velocities;
        std::vector<double> accelerations(dvdt.data(), dvdt.data() + dvdt.size());
        response.accelerations = accelerations;
        response.time_from_start.sec = (int) floor(t);
        response.time_from_start.nanosec = (int) ( (t - floor(t)) * pow(10.0, 9.0) ); // TODO: more legit way?
        publisher_->publish(response);
        
        // Publish feedback
        feedback->percent_complete = ((double) k) / ((double) (N - 1)) * 100.0;
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

} // namespace rrlib

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rrlib::CartesianSinusoidalTrajectoryNode>());
    rclcpp::shutdown();
    return 0;
}
