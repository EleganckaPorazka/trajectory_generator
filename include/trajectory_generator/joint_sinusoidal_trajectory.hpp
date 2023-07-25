//~ Copyright (C) 2023 Łukasz Woliński
//~ You may use, distribute and modify this code under the terms of the BSD-3-Clause License.

#ifndef JNT_SIN_TRAJ_H
#define JNT_SIN_TRAJ_H

#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>

namespace rrlib
{
class JointSinusoidalTrajectory
{
public:
    JointSinusoidalTrajectory();
    void SetParameters( const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_end, size_t DOF, double vel_max, double acc_max );
    bool AreParametersOK();
    double GetMotionTime();
    size_t GetDOF();
    void MotionProfile( double t, double* u, double* dudt, double* d2udt2 );
    void PositionVelocityAcceleration( double t, Eigen::VectorXd* q, Eigen::VectorXd* dqdt, Eigen::VectorXd* d2qdt2 );
    
private:
    Eigen::VectorXd q_start_, q_end_;   // start and end points (in radians)
    size_t DOF_;                        // number of degrees of freedom of the joint trajectory
    double vel_max_;                    // maximum velocity: a scalar value, greater than 0 (in rad/s)
    double acc_max_;                    // maximum velocity: a scalar value, greater than 0 (in rad/s^2)
    double t_acc_;                      // acceleration (and deceleration) duration
    double t_end_;                      // motion time
    bool PARAMETERS_OK_;                // a flag to be set to true if the parameters are viable for the trajectory computation, and false otherwise; TODO: is this necessary? remove it?
};

JointSinusoidalTrajectory::JointSinusoidalTrajectory()
{
    /* Default parameters are zero. User shall use setParameters to assign them the proper values before computing the trajectory.*/
    q_start_ = Eigen::Vector3d::Zero();
    q_end_ = Eigen::Vector3d::Zero();
    DOF_ = 0;
    vel_max_ = 0.0;
    acc_max_ = 0.0;
    t_acc_ = 0.0;
    t_end_ = 0.0;
    PARAMETERS_OK_ = false;
}

void JointSinusoidalTrajectory::SetParameters( const Eigen::VectorXd &q_start, const Eigen::VectorXd &q_end, size_t DOF, double vel_max, double acc_max )
{
    q_start_ = q_start;
    q_end_ = q_end;
    DOF_ = DOF;
    vel_max_ = vel_max;
    acc_max_ = acc_max;
    
    /* axis synchronization -- picking the longest motion time (motion time of the axis which has the biggest distance to cover) */
    Eigen::VectorXd distance = ( q_end_ - q_start_ ).cwiseAbs();
    Eigen::VectorXd t_acc_temp(DOF_);
    Eigen::VectorXd t_end_temp(DOF_);
    
    for (size_t j = 0; j < DOF_; j++)
    {
        if ( distance(j) > 2.0 * (vel_max_ * vel_max_) / acc_max_ )
        {
            t_acc_temp(j) = 2.0 * vel_max_ / acc_max_;
        }
        else // sinusoidal "trapezoid" degenerates to "triangle"
        {
            t_acc_temp(j) = sqrt( 2.0 * distance(j) / acc_max_ );
        }
    }
    
    t_acc_ = t_acc_temp.maxCoeff();
    
    for (size_t j = 0; j < DOF_; j++)
    {
        if ( distance(j) > 2.0 * (vel_max_ * vel_max_) / acc_max_ )
        {
            t_end_temp(j) = distance(j) / vel_max_ + t_acc_;
        }
        else // sinusoidal "trapezoid" degenerates to "triangle"
        {
            t_end_temp(j) = 2.0 * t_acc_temp(j);
        }
    }
    
    t_end_ = t_end_temp.maxCoeff();
    
    PARAMETERS_OK_ = true;
}

bool JointSinusoidalTrajectory::AreParametersOK()
{
    return PARAMETERS_OK_;
}

double JointSinusoidalTrajectory::GetMotionTime()
{
    return t_end_;
}

size_t JointSinusoidalTrajectory::GetDOF()
{
    return DOF_;
}

void JointSinusoidalTrajectory::MotionProfile( double t, double* u, double* dudt, double* d2udt2 )
{
    double eta = t / t_end_; // normalize time to <0, 1>
    double p = t_acc_ / t_end_;
    
    if (eta < 0.0)
    {
        *u = 0.0;
        *dudt = 0.0;
        *d2udt2 = 0.0;
    }
    else if (eta < p)
    {
        *u = ( (eta * eta) / (2.0 * p) + p / (4.0 * M_PI * M_PI) * ( cos(2.0 * M_PI * eta / p) - 1.0 ) ) / (1.0 - p);
        *dudt = ( eta / p - sin(2.0 * M_PI * eta / p) / (2.0 * M_PI) ) / ( (1.0 - p) * t_end_ );
        *d2udt2 = ( 1.0 / p - cos(2.0 * M_PI * eta / p) / p ) / ( (1.0 - p) * t_end_ * t_end_ );
    }
    else if (eta < 1.0 - p)
    {
        *u = ( eta - p / 2.0 ) / ( 1.0 - p );
        *dudt = 1.0 / ( (1.0 - p) * t_end_ );
        *d2udt2 = 0.0;
    }
    else if (eta < 1.0)
    {
        *u = ( ( 2.0 * eta - eta * eta + 2.0 * p - 1.0 - 2.0 * p * p ) / (2.0 * p) + p / (4.0 * M_PI * M_PI) * ( 1.0 - cos( 2.0 * M_PI / p * (eta - 1.0 + p) ) ) ) / (1.0 - p);
        *dudt = ( (1.0 - eta) / p + sin(2.0 * M_PI * (eta - 1.0 + p) / p) / (2.0 * M_PI) ) / ( (1.0 - p) * t_end_ );
        *d2udt2 = ( -1.0 / p + cos(2.0 * M_PI * (eta - 1.0 + p) / p) / p ) / ( (1.0 - p) * t_end_ * t_end_ );
    }
    else
    {
        *u = 1.0;
        *dudt = 0.0;
        *d2udt2 = 0.0;
    }
}

void JointSinusoidalTrajectory::PositionVelocityAcceleration( double t, Eigen::VectorXd* q, Eigen::VectorXd* dqdt, Eigen::VectorXd* d2qdt2 )
{
    double u, dudt, d2udt2;
    
    MotionProfile(t, &u, &dudt, &d2udt2);
    
    *q = q_start_ + ( q_end_ - q_start_ ) * u;
    *dqdt = ( q_end_ - q_start_ ) * dudt;
    *d2qdt2 = ( q_end_ - q_start_ ) * d2udt2;
}

} // namespace rrlib

#endif // JNT_SIN_TRAJ_H
