//~ Copyright (C) 2023 Łukasz Woliński
//~ You may use, distribute and modify this code under the terms of the BSD-3-Clause License.

#ifndef CART_SIN_TRAJ_H
#define CART_SIN_TRAJ_H

#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>
#include "helper_functions.hpp"

// TODO: (1) end effector orientation change, (2) circular trajectory

namespace rrlib
{
class CartesianSinusoidalTrajectory
{
public:
    CartesianSinusoidalTrajectory();
    void SetParameters( const Eigen::VectorXd &x_start, const Eigen::VectorXd &x_end, double vel_max, double acc_max );
    double GetMotionTime();
    void MotionProfile( double t, double* u, double* dudt, double* d2udt2 );
    void PositionVelocityAcceleration( double t, Eigen::VectorXd* x, Eigen::VectorXd* v, Eigen::VectorXd* dvdt );
    
private:
    Eigen::VectorXd x_start_, x_end_;   // start and end poses: x = [position (in meters); orientation (unit quaternions)]
    double vel_max_;                    // maximum velocity: a scalar value, greater than 0 (in m/s)
    double acc_max_;                    // maximum velocity: a scalar value, greater than 0 (in m/s^2)
    double t_acc_;                      // acceleration (and deceleration) duration
    double t_end_;                      // motion time
};

CartesianSinusoidalTrajectory::CartesianSinusoidalTrajectory()
{
    /* Default parameters are zero. User shall use setParameters to assign them the proper values before computing the trajectory.*/
    x_start_ = Eigen::VectorXd::Zero(7);
    x_end_ = Eigen::VectorXd::Zero(7);
    vel_max_ = 0.0;
    acc_max_ = 0.0;
    t_acc_ = 0.0;
    t_end_ = 0.0;
}

void CartesianSinusoidalTrajectory::SetParameters( const Eigen::VectorXd &x_start, const Eigen::VectorXd &x_end, double vel_max, double acc_max )
{
    x_start_ = x_start;
    x_end_ = x_end;
    vel_max_ = vel_max;
    acc_max_ = acc_max;
    
    /* Compute the acceleration time t_acc_ and total motion time t_end_ */
    double l = ( x_end_.head(3) - x_start_.head(3) ).norm();
    
    if ( l > 2.0 * (vel_max_ * vel_max_) / acc_max_ )
    {
        t_acc_ = 2.0 * vel_max_ / acc_max_;
    }
    else // sinusoidal "trapezoid" degenerates to "triangle"
    {
        t_acc_ = sqrt( 2.0 * l / acc_max_ );
    }
    
    if ( l > 2.0 * (vel_max_ * vel_max_) / acc_max_ )
    {
        t_end_ = l / vel_max_ + t_acc_;
    }
    else // sinusoidal "trapezoid" degenerates to "triangle"
    {
        t_end_ = 2.0 * t_acc_;
    }
}

double CartesianSinusoidalTrajectory::GetMotionTime()
{
    return t_end_;
}

void CartesianSinusoidalTrajectory::MotionProfile( double t, double* u, double* dudt, double* d2udt2 )
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

void CartesianSinusoidalTrajectory::PositionVelocityAcceleration( double t, Eigen::VectorXd* x, Eigen::VectorXd* v, Eigen::VectorXd* dvdt )
{
    double u, dudt, d2udt2;
    
    MotionProfile(t, &u, &dudt, &d2udt2);
    
    Eigen::Vector3d position, trans_vel, trans_acc, rot_vel, rot_acc;
    Eigen::Vector4d orientation;
    Eigen::VectorXd pose(7), vel(6), acc(6);
    
    /* translational motion */
    position = x_start_.head(3) + ( x_end_.head(3) - x_start_.head(3) ) * u;
    trans_vel = ( x_end_.head(3) - x_start_.head(3) ) * dudt;
    trans_acc = ( x_end_.head(3) - x_start_.head(3) ) * d2udt2;
    
    /*TODO: rotational motion*/
    orientation = x_start_.tail(4);
    rot_vel = Eigen::Vector3d::Zero();
    rot_acc = Eigen::Vector3d::Zero();
    
    pose.head(3) = position;
    pose.tail(4) = orientation;
    vel.head(3) = trans_vel;
    vel.tail(3) = rot_vel;
    acc.head(3) = trans_acc;
    acc.tail(3) = rot_acc;
    
    *x = pose;
    *v = vel;
    *dvdt = acc;
}

} // namespace rrlib

#endif // CART_SIN_TRAJ_H
