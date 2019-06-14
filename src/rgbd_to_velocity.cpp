//
// Created by humberto on 5/20/19.
//
#include <eigen3/Eigen/Core>
#include "../include/rgbd_to_velocity.h"
#include <geometry_msgs/Twist.h>

namespace rgbd_to_velocity {
    //Constructor
    RgbdToVelocity::RgbdToVelocity(): private_nh_("~"), nh_("") {
        pose_subscriber_ = nh_.subscribe("cam_to_init", 1, &RgbdToVelocity::poseCallback, this);
        velocity_init_frame_publisher_ = nh_.advertise<reef_msgs::DeltaToVel>("rgbd_to_velocity/init_frame", 1, true);
//    velocity_camera_frame_publisher_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("mocap_velocity/camera_frame", 1, true);
        velocity_level_body_publisher_ = nh_.advertise<reef_msgs::DeltaToVel>("rgbd_to_velocity/body_level_frame", 1, true);

        C_from_camera_level_frame_to_NED_level_frame << 0, 0, 1,
                -1,0,0,
                0,-1,0;

        previous_position_init << 0,0,0;
        previous_velocity_init << 0,0,0;
        previous_time_stamp = 0;
        previous_position_init << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;

        inv_previous_quaternion_init_to_body  = Eigen::MatrixXd(4,1);
        inv_previous_quaternion_init_to_body <<  0,0,0,1;
        counterOfSamples = 0;

        private_nh_.param<double>("alpha", alpha, 1.0);
        ROS_WARN_STREAM("alpha = ");
        ROS_WARN_STREAM(alpha);

        private_nh_.param<double>("x_vel_covariance", x_vel_covariance, 0.01);
        ROS_WARN_STREAM("RGBD_x_vel_covariance = ");
        ROS_WARN_STREAM(x_vel_covariance);
        private_nh_.param<double>("y_vel_covariance", y_vel_covariance, 0.01 );
        ROS_WARN_STREAM("RGBD_y_vel_covariance = ");
        ROS_WARN_STREAM(y_vel_covariance);


        quaternion_body_to_camera = Eigen::VectorXd(4);
        reef_msgs::importMatrixFromParamServer(private_nh_,quaternion_body_to_camera,"body_to_camera_quat");

        translation_body_to_camera = Eigen::VectorXd(3);
        reef_msgs::importMatrixFromParamServer(private_nh_,translation_body_to_camera,"body_to_camera_trans");
    }

    //Destructor
    RgbdToVelocity::~RgbdToVelocity() {}

    void RgbdToVelocity::poseCallback(const nav_msgs::OdometryConstPtr& msg) {

        odom_msg = *msg;

//        quat.x() =  odom_msg.pose.pose.orientation.x;
//        quat.y() =  odom_msg.pose.pose.orientation.y;
//        quat.z() =  odom_msg.pose.pose.orientation.z;
//        quat.w() =  odom_msg.pose.pose.orientation.w;
//
//        euler_deg = quat.toRotationMatrix().eulerAngles(1, 0, 2); //converted to degrees here already

        counterOfSamples++;
        current_time_stamp = odom_msg.header.stamp.toSec();
        DT = current_time_stamp - previous_time_stamp;
        if(DT >= (1.0/30.0)) {

            current_position_init << odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z;
            //-------------------------------Block to compute Rotation
            beta <<  odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z;  // Vector quaternion part// ;
            beta_0 = odom_msg.pose.pose.orientation.w;

            current_quaternion_init_to_body  = Eigen::MatrixXd(4,1);
            current_quaternion_init_to_body<< beta, beta_0;

            Eigen::MatrixXd C_init_to_camera_frame(3,3);
            //Cayley transform. Convert quaternion message into a C matrix.
            C_init_to_camera_frame =  quaternionToRotation(beta,beta_0);



            CtoYawPitchRoll213(C_init_to_camera_frame);
            //Form a 2-rotation.
            C_from_init_to_camera_level_frame << cos(yaw), 0, -sin(yaw),
                    0,        1,      0,
                    sin(yaw),        0,    cos(yaw);

            Eigen::MatrixXd C_init_to_NED_camera_level_frame(3,3);
            C_init_to_NED_camera_level_frame = C_from_camera_level_frame_to_NED_level_frame*C_from_init_to_camera_level_frame;

            Eigen::MatrixXd C_from_camera_frame_to_image_frame(3,3);
            C_from_camera_frame_to_image_frame << -1, 0, 0,
                                                   0,-1,0,
                                                   0,0,1;

            //Cayley transform. Convert quaternion message into a C matrix.
            Eigen::MatrixXd C_body_to_camera_frame(3,3);
            C_body_to_camera_frame = quaternionToRotation(quaternion_body_to_camera.head(3),quaternion_body_to_camera(3));


            Eigen::MatrixXd C_from_camera_level_to_body_NED_frame(3,3);
            C_from_camera_level_to_body_NED_frame = C_body_to_camera_frame.transpose()*C_from_camera_frame_to_image_frame*C_init_to_camera_frame*C_init_to_NED_camera_level_frame.transpose();
            CtoYawPitchRoll321(C_from_camera_level_to_body_NED_frame);

            Eigen::MatrixXd C_from_camera_level_to_body_level_NED_frame(3,3);
            C_from_camera_level_to_body_level_NED_frame <<      cos(yaw),      sin(yaw),   0,
                                                               -sin(yaw),     cos(yaw),    0,
                                                                0,              0,         1 ;

            //Estimating angular velocity.
//          Eigen::Vector4d delta_quaternion(4,1);
//          delta_quaternion = multiplyQuat(current_quaternion_init_to_body,inv_previous_quaternion_init_to_body);
//
//          delta_quaternion_magnitude = sqrt(pow(delta_quaternion(0),2) + pow(delta_quaternion(1),2) +pow(delta_quaternion(2),2));
//          euler_angle = 2*atan2(delta_quaternion_magnitude,delta_quaternion(3));
//
//
//          if(delta_quaternion_magnitude>0){
//              //Normalize axis vector, such that it is just a direction vector.
//              axis_vector = delta_quaternion.head(3) / delta_quaternion_magnitude;
//          }
//
//
//          angular_vel_body_to_n_in_body_frame = axis_vector * euler_angle/(DT);
//
//          extra_linear_vel_of_camera_in_body_frame = angular_vel_body_to_n_in_body_frame.cross(translation_body_to_camera);
//
//          angular_vel_body_to_n_in_inertial_frame = C_init_to_camera_frame.transpose() * extra_linear_vel_of_camera_in_body_frame;
            estimated_velocity_init = (current_position_init - previous_position_init) * (1 / DT);

            //Alpha-Beta filter. Alpha is a measured of how much the new estimated velocity is used.
            filtered_velocity_init = alpha * estimated_velocity_init + (1 - alpha) * previous_velocity_init;

            //Now that we have used the current position and current time, we make it the previous position and previous time.
            previous_position_init = current_position_init;
            previous_time_stamp = current_time_stamp;

            // Assign the new estimated velocity to the previous velocity here.
            previous_velocity_init = filtered_velocity_init;
//          inv_previous_quaternion_init_to_body << -current_quaternion_init_to_body(0), -current_quaternion_init_to_body(1), -current_quaternion_init_to_body(2),current_quaternion_init_to_body(3);

            //Publish velocities in init frame
            vel_msg.vel.header.stamp = odom_msg.header.stamp;
            vel_msg.vel.twist.twist.linear.x = filtered_velocity_init(0);
            vel_msg.vel.twist.twist.linear.y = filtered_velocity_init(1);
            vel_msg.vel.twist.twist.linear.z = filtered_velocity_init(2);

            velocity_init_frame_publisher_.publish(vel_msg);

            //Publish velocities in camera frame
//          filtered_velocity_body_leveled_frame = C_body_to_camera_frame * C_init_to_camera_frame * filtered_velocity_init;
//          vel_msg.vel.header.stamp = pose.header.stamp;
//          vel_msg.vel.twist.twist.linear.x = filtered_velocity_body_leveled_frame(0);
//          vel_msg.vel.twist.twist.linear.y = filtered_velocity_body_leveled_frame(1);
//          vel_msg.vel.twist.twist.linear.z = filtered_velocity_body_leveled_frame(2);
//
//          velocity_camera_frame_publisher_.publish(vel_msg);
//
//          //Publish velocities in NED camera leveled frame
            filtered_velocity_body_leveled_frame =  C_from_camera_level_to_body_level_NED_frame*C_init_to_NED_camera_level_frame*filtered_velocity_init;
            vel_msg.vel.header.stamp = odom_msg.header.stamp;
            vel_msg.vel.twist.twist.linear.x = filtered_velocity_body_leveled_frame(0);
            vel_msg.vel.twist.twist.linear.y = filtered_velocity_body_leveled_frame(1);
            vel_msg.vel.twist.twist.linear.z = filtered_velocity_body_leveled_frame(2);
            covariance_matrix_in_init <<   x_vel_covariance, 0,                   0 ,
                                                0,               y_vel_covariance ,    0,
                                                0,                0,                   0;


            covariance_matrix_in_body_level = C_from_camera_level_to_body_level_NED_frame*C_init_to_NED_camera_level_frame* covariance_matrix_in_init * (C_from_camera_level_to_body_level_NED_frame*C_init_to_NED_camera_level_frame).transpose();

            //Next we fill out the message
            Eigen::Vector3d sigmas_level;
            sigmas_level = covariance_matrix_in_body_level.diagonal().array().sqrt();
            vel_msg.vel.twist.covariance[0] = covariance_matrix_in_body_level(0,0);
            vel_msg.vel.twist.covariance[7] = covariance_matrix_in_body_level(1,1);
            vel_msg.vel.twist.covariance[14] = covariance_matrix_in_body_level(2,2);
            vel_msg.S_upper_bound[0] = filtered_velocity_body_leveled_frame(0)  + 3*sigmas_level(0);
            vel_msg.S_upper_bound[1] = filtered_velocity_body_leveled_frame(1)  + 3*sigmas_level(1);
            vel_msg.S_upper_bound[2] = filtered_velocity_body_leveled_frame(2)  + 3*sigmas_level(2);
            vel_msg.S_lower_bound[0] = filtered_velocity_body_leveled_frame(0)  - 3*sigmas_level(0);
            vel_msg.S_lower_bound[1] = filtered_velocity_body_leveled_frame(1)  - 3*sigmas_level(1);
            vel_msg.S_lower_bound[2] = filtered_velocity_body_leveled_frame(2)  - 3*sigmas_level(2);
            velocity_level_body_publisher_.publish(vel_msg);
            counterOfSamples = 0;
        }
    }

    void RgbdToVelocity::CtoYawPitchRoll321(Eigen::Matrix3d C){
        pitch = -asin(C(0,2));
        roll =  atan2(C(0,2),C(2,2));
        yaw =  atan2(C(0,1),C(0,0));
    }

    void RgbdToVelocity::CtoYawPitchRoll213(Eigen::Matrix3d C){
        yaw =  atan2(C(2,0),C(2,2));

        pitch = -asin(C(2,1));

        roll =  atan2(C(0,1),C(1,1));
    }

    Eigen::Matrix3d RgbdToVelocity::quaternionToRotation(Eigen::Vector3d quaternionVectorPart, double quaternionScalarPart)
    {
        //-------------------------------Block to compute Rotation Matrix
        double beta_0;
        Eigen::Vector3d beta;
        beta = quaternionVectorPart;
        beta_0 = quaternionScalarPart;

        Eigen::MatrixXd beta_skew(3,3);
        //Form skew symmetric matrix.
        beta_skew << 0,        -beta(2), beta(1),
                beta(2),  0,        -beta(0),
                -beta(1), beta(0),  0;

        Eigen::MatrixXd I(3,3);
        //Define the 3x3 identity matrix that will be used in the Cayley transform.
        I.setIdentity(3,3);

        Eigen::Matrix3d C_from_a_to_b_through;
        //Cayley transform. Convert quaternion message into a C matrix.
        C_from_a_to_b_through =  I - 2*beta_0*beta_skew + 2*beta_skew*beta_skew;
        return C_from_a_to_b_through;
    }

    Eigen::Vector4d RgbdToVelocity::multiplyQuat(Eigen::MatrixXd q,Eigen::MatrixXd p) {
        double p4;
        p4 = p(3);
        Eigen::MatrixXd I(3,3);
        I.setIdentity();
        Eigen::MatrixXd p_skew(3,3);
        //Form skew symmetric matrix.
        p_skew << 0,        -p(2), p(1),
                p(2),  0,        -p(0),
                -p(1), p(0),  0;

        Eigen::MatrixXd A(4,4);  //See trawney eq 10.

        A.block<3,3>(0,0) = p4*I + p_skew;
        A.block<4,1>(0,3) = p;
        A.block<1,4>(3,0) = -p.transpose();
        A(3,3) = p4;
        Eigen::Vector4d product(4,1);
        product = A * q; //quaternion multiplication q*p

        return product;

    }





}
