//~ Copyright (C) 2023 Łukasz Woliński
//~ You may use, distribute and modify this code under the terms of the BSD-3-Clause License.

#include <iostream>
#include <memory>
#include <string>
#include <eigen3/Eigen/Dense>
#include "trajectory_generator/joint_sinusoidal_trajectory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/float64.hpp" //temporary, because deprecated?

using std::placeholders::_1;

class JointSinusoidalTrajectoryNode : public rclcpp::Node
{
public:
    JointSinusoidalTrajectoryNode();
    //~ void SetParameters( const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_end, size_t DOF, double vel_max, double acc_max );
    void SetParametersTest();
    
private:
    void TopicCallback(const std_msgs::msg::Float64 & msg);
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr publisher_;
    
    rrlib::JointSinusoidalTrajectory trajectory_;
    
};

JointSinusoidalTrajectoryNode::JointSinusoidalTrajectoryNode()
: Node("joint_sinusoidal_trajectory")
{
    RCLCPP_INFO(this->get_logger(), "Starting the node.");
    
    // create a subscriber for the local time
    subscription_ = this->create_subscription<std_msgs::msg::Float64>("jnt_sin_local_time", 10, std::bind(&JointSinusoidalTrajectoryNode::TopicCallback, this, _1));
    // create a publisher for the joint trajectory point
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("jnt_sin_traj", 10);
    
    SetParametersTest();
}

//~ void JointSinusoidalTrajectoryNode::SetParameters( const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_end, size_t DOF, double vel_max, double acc_max )
//~ {
    //~ if (q_start.size() != DOF or q_end.size() != DOF or vel_max <= 0.0 or acc_max <= 0.0)
    //~ {
        //~ std::string message;
        //~ message = "\nParameters are not set. Please, provide the correct values:\n-> q_start and q_end need to be of the same size,\n-> v_max has to be greater than 0,\n-> a max has to be greater than 0.\n";
        //~ std::cout << message;
        //~ return;
    //~ }
    
    //~ trajectory_.SetParameters(q_start, q_end, DOF, vel_max, acc_max);
//~ }

void JointSinusoidalTrajectoryNode::SetParametersTest()
{
    /* test function to set the joint trajectory generator parameters */
    Eigen::VectorXd q_start = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd q_end(7);
    q_end << 0.0, 0.0, 0.0, M_PI/2.0, 0.0, -M_PI/2.0, 0.0;
    size_t DOF = 7;
    double vel_max = 100.0 / 180.0 * M_PI;
    double acc_max = 500.0 / 180.0 * M_PI;
    trajectory_.SetParameters(q_start, q_end, DOF, vel_max, acc_max);
}

void JointSinusoidalTrajectoryNode::TopicCallback(const std_msgs::msg::Float64 & msg)
{
    double t = msg.data;
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
    response.time_from_start.nanosec = (int) (t - floor(t)) * pow(10.0, 9.0); // TODO: check?
    publisher_->publish(response);

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointSinusoidalTrajectoryNode>());
    rclcpp::shutdown();
    return 0;
}
