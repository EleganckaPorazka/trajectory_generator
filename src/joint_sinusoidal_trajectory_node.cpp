class JointSinusoidalTrajectoryNode
{
public:
    JointSinusoidalTrajectoryNode();
    JointSinusoidalTrajectoryNode( const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_end, double vel_max, double acc_max );
    void setParameters( const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_end, double vel_max, double acc_max );
    double getMotionTime();
    void positionVelocityAcceleration( double t, Eigen::VectorXd* q, Eigen::VectorXd* dqdt, Eigen::VectorXd* d2qdt2 );
    
private:
    JointSinusoidalTrajectory trajectory;
    
};

JointSinusoidalTrajectoryNode::JointSinusoidalTrajectoryNode()
{
}

JointSinusoidalTrajectoryNode::JointSinusoidalTrajectoryNode( const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_end, double vel_max, double acc_max )
{
    setParameters(q_start, q_end, vel_max, acc_max);
}

void JointSinusoidalTrajectoryNode::setParameters( const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_end, double vel_max, double acc_max )
{
    if (q_start.size() != q_end.size() or vel_max <= 0.0 or acc_max <= 0.0)
    {
        std::string message;
        message = "\nParameters are not set. Please, provide the correct values:\n-> q_start and q_end need to be of the same size,\n-> v_max has to be greater than 0,\n-> a max has to be greater than 0.\n";
        std::cout << message;
        return;
    }
    
    trajectory.setParameters(q_start, q_end, vel_max, acc_max);
}

double JointSinusoidalTrajectoryNode::getMotionTime()
{
    return trajectory.getMotionTime();
}

void JointSinusoidalTrajectoryNode::positionVelocityAcceleration( double t, Eigen::VectorXd* q, Eigen::VectorXd* dqdt, Eigen::VectorXd* d2qdt2 )
{
    if (trajectory.areParametersOK() != true)
    {
        std::cout << "The JointSinusoidalTrajectory class parameters are not viable for computing the trajectory.\n";
        *q = Eigen::VectorXd::Zero( trajectory.getDOF() );
        *dqdt = Eigen::VectorXd::Zero( trajectory.getDOF() );
        *d2qdt2 = Eigen::VectorXd::Zero( trajectory.getDOF() );
        return;
    }
    
    trajectory.positionVelocityAcceleration(t, q, dqdt, d2qdt2);
}
