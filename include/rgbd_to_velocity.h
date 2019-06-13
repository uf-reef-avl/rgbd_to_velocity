//
// Created by humberto on 5/20/19.
//

#ifndef RGBD_TO_VELOCITY_H
#define RGBD_TO_VELOCITY_H
#endif //TURTLE_NAV_TURTLE_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <eigen3/Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <random>
#include <cmath>
#include "../../reef_msgs/include/reef_msgs/matrix_operation.h"
#include "../../reef_msgs/include/reef_msgs/dynamics.h"
#include <reef_msgs/DeltaToVel.h>

namespace rgbd_to_velocity {

    class RgbdToVelocity {
    private:
        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;
        ros::Publisher vel_publisher_;
        ros::Publisher velocity_init_frame_publisher_;
        ros::Publisher velocity_camera_frame_publisher_;
        ros::Publisher velocity_level_body_publisher_;
        ros::Subscriber pose_subscriber_;



    public:
        RgbdToVelocity();//Constructor
        ~RgbdToVelocity();//Destructor
        double mocap_noise_std;
        double alpha;
        double current_time_stamp;
        double previous_time_stamp;
        double pitch;
        double roll;
        double yaw;
        double DT;
        double beta_0;
        double delta_quaternion_magnitude;
        double euler_angle;
        double x_vel_covariance;
        double y_vel_covariance;
        int counterOfSamples;

        //Variable section
        Eigen::Quaterniond quat;
        nav_msgs::Odometry odom_msg;
        Eigen::Vector3d euler_deg;
        geometry_msgs::PoseStamped pose;
        reef_msgs::DeltaToVel vel_msg;
        void CtoYawPitchRoll213(Eigen::Matrix3d C);
        void CtoYawPitchRoll321(Eigen::Matrix3d C);
        Eigen::Matrix3d quaternionToRotation(Eigen::Vector3d quaternionVectorPart, double quaternionScalarPart);
        Eigen::Vector4d multiplyQuat(Eigen::MatrixXd q,Eigen::MatrixXd p);

        std::mt19937 engine;
        std::normal_distribution<double> distribution;

        Eigen::MatrixXd inv_previous_quaternion_init_to_body;
        Eigen::MatrixXd current_quaternion_init_to_body;
        Eigen::Matrix3d C_from_init_to_camera_level_frame;
        Eigen::Matrix3d C_from_camera_level_frame_to_NED_level_frame;
        Eigen::Matrix3d covariance_matrix_in_init;
        Eigen::Matrix3d covariance_matrix_in_body_level;

        Eigen::Vector3d beta;
        Eigen::Vector3d previous_velocity_init;
        Eigen::Vector3d estimated_velocity_init;
        Eigen::Vector3d filtered_velocity_init;
        Eigen::Vector3d filtered_velocity_body_leveled_frame;
        Eigen::Vector3d previous_position_init;
        Eigen::Vector3d current_position_init;
        Eigen::VectorXd quaternion_body_to_camera;
        Eigen::Vector3d translation_body_to_camera;
        Eigen::Vector3d axis_vector;
        Eigen::Vector3d angular_vel_body_to_n_in_body_frame;
        Eigen::Vector3d angular_vel_body_to_n_in_inertial_frame;
        Eigen::Vector3d extra_linear_vel_of_camera_in_body_frame;


        //Declare all callbacks here
        void poseCallback(const nav_msgs::OdometryConstPtr& msg);


    };
}
