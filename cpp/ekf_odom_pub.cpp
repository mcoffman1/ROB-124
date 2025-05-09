// Include various libraries
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

// Create odometry data publishers
ros::Publisher odom_data_pub;
ros::Publisher odom_data_pub_quat;
nav_msgs::Odometry odomNew;
nav_msgs::Odometry odomOld;

// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;

// Robot physical constants
const double TICKS_PER_REVOLUTION = 620; // For reference purposes
const double WHEEL_RADIUS = 0.033;        // Wheel radius in meters
const double WHEEL_BASE = 0.17;            // Center of left tire to center of right tire
const double TICKS_PER_METER = 3100;       // Original was 2800

// Distance both wheels have traveled
double distanceLeft = 0;
double distanceRight = 0;

// Flag to see if initial pose has been received
bool initialPoseReceived = false;

// TF broadcaster
tf2_ros::TransformBroadcaster* tf_broadcaster_ptr = nullptr;

// Get initial_2d message from either RViz clicks or a manual pose publisher
void set_initial_2d(const geometry_msgs::PoseStamped& rvizClick) {
    odomOld.pose.pose.position.x = rvizClick.pose.position.x;
    odomOld.pose.pose.position.y = rvizClick.pose.position.y;
    odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z;
    initialPoseReceived = true;
}

// Calculate the distance the left wheel has traveled since the last cycle
void Calc_Left(const std_msgs::Int16& leftCount) {
    static int lastCountL = 0;
    if (leftCount.data != 0 && lastCountL != 0) {
        int leftTicks = leftCount.data - lastCountL;

        if (leftTicks > 10000) {
            leftTicks = 0 - (65535 - leftTicks);
        } else if (leftTicks < -10000) {
            leftTicks = 65535 - leftTicks;
        }
        distanceLeft = static_cast<double>(leftTicks) / TICKS_PER_METER;
    }
    lastCountL = leftCount.data;
}

// Calculate the distance the right wheel has traveled since the last cycle
void Calc_Right(const std_msgs::Int16& rightCount) {
    static int lastCountR = 0;
    if (rightCount.data != 0 && lastCountR != 0) {
        int rightTicks = rightCount.data - lastCountR;

        if (rightTicks > 10000) {
            rightTicks = 0 - (65535 - rightTicks);
        } else if (rightTicks < -10000) {
            rightTicks = 65535 - rightTicks;
        }
        distanceRight = static_cast<double>(rightTicks) / TICKS_PER_METER;
    }
    lastCountR = rightCount.data;
}

// Publish a nav_msgs::Odometry message in quaternion format
void publish_quat(const ros::Time& current_time) {
    tf2::Quaternion q;
    q.setRPY(0, 0, odomNew.pose.pose.orientation.z);

    nav_msgs::Odometry quatOdom;
    quatOdom.header.stamp = current_time;
    quatOdom.header.frame_id = "odom";
    quatOdom.child_frame_id = "base_link";
    quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
    quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
    quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
    quatOdom.pose.pose.orientation.x = q.x();
    quatOdom.pose.pose.orientation.y = q.y();
    quatOdom.pose.pose.orientation.z = q.z();
    quatOdom.pose.pose.orientation.w = q.w();
    quatOdom.twist.twist = odomNew.twist.twist;

    for (int i = 0; i < 36; i++) {
        if (i == 0 || i == 7 || i == 14) {
            quatOdom.pose.covariance[i] = 0.01;
        } else if (i == 21 || i == 28 || i == 35) {
            quatOdom.pose.covariance[i] = 0.1;
        } else {
            quatOdom.pose.covariance[i] = 0.0;
        }
    }

    odom_data_pub_quat.publish(quatOdom);

    // Publish the transform
    if (tf_broadcaster_ptr) {
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = quatOdom.pose.pose.position.x;
        odom_trans.transform.translation.y = quatOdom.pose.pose.position.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = quatOdom.pose.pose.orientation;
        tf_broadcaster_ptr->sendTransform(odom_trans);
    }
}

// Update odometry information
void update_odom() {
    double cycleDistance = (distanceRight + distanceLeft) / 2.0;
    double cycleAngle = asin((distanceRight - distanceLeft) / WHEEL_BASE);

    double avgAngle = cycleAngle / 2.0 + odomOld.pose.pose.orientation.z;

    if (avgAngle > M_PI) {
        avgAngle -= 2.0 * M_PI;
    } else if (avgAngle < -M_PI) {
        avgAngle += 2.0 * M_PI;
    }

    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle) * cycleDistance;
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle) * cycleDistance;
    odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;

    if (std::isnan(odomNew.pose.pose.position.x) || std::isnan(odomNew.pose.pose.position.y)) {
        odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
        odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
        odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
    }

    if (odomNew.pose.pose.orientation.z > M_PI) {
        odomNew.pose.pose.orientation.z -= 2.0 * M_PI;
    } else if (odomNew.pose.pose.orientation.z < -M_PI) {
        odomNew.pose.pose.orientation.z += 2.0 * M_PI;
    }

    // Velocity calculation
    ros::Time current_time = ros::Time::now();
    odomNew.header.stamp = current_time;

    double dt = (odomNew.header.stamp - odomOld.header.stamp).toSec();
    if (dt > 0.0001) {
        odomNew.twist.twist.linear.x = cycleDistance / dt;
        odomNew.twist.twist.angular.z = cycleAngle / dt;
    } else {
        odomNew.twist.twist.linear.x = 0.0;
        odomNew.twist.twist.angular.z = 0.0;
    }

    // Publish euler odometry
    odom_data_pub.publish(odomNew);

    // Save old values for next cycle
    odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
    odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
    odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
    odomOld.header.stamp = odomNew.header.stamp;

    // Publish quaternion odometry
    publish_quat(current_time);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ekf_odom_pub");
    ros::NodeHandle node;

    // Initialize publishers
    odom_data_pub = node.advertise<nav_msgs::Odometry>("odom_data_euler", 100);
    odom_data_pub_quat = node.advertise<nav_msgs::Odometry>("odom_data_quat", 100);

    // Initialize TF broadcaster
    tf_broadcaster_ptr = new tf2_ros::TransformBroadcaster();

    // Initialize odometry fields
    odomNew.header.frame_id = "odom";
    odomNew.pose.pose.position.z = 0.0;
    odomNew.pose.pose.orientation.x = 0.0;
    odomNew.pose.pose.orientation.y = 0.0;
    odomNew.twist.twist.linear.x = 0.0;
    odomNew.twist.twist.linear.y = 0.0;
    odomNew.twist.twist.linear.z = 0.0;
    odomNew.twist.twist.angular.x = 0.0;
    odomNew.twist.twist.angular.y = 0.0;
    odomNew.twist.twist.angular.z = 0.0;
    odomOld.pose.pose.position.x = initialX;
    odomOld.pose.pose.position.y = initialY;
    odomOld.pose.pose.orientation.z = initialTheta;

    // Subscribe to encoder and initial pose topics
    ros::Subscriber subRight = node.subscribe("right_ticks", 100, Calc_Right, ros::TransportHints().tcpNoDelay());
    ros::Subscriber subLeft = node.subscribe("left_ticks", 100, Calc_Left, ros::TransportHints().tcpNoDelay());
    ros::Subscriber subInitPose = node.subscribe("initial_2d", 1, set_initial_2d);

    ros::Rate loop_rate(30);

    while (ros::ok()) {
        if (initialPoseReceived) {
            update_odom();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete tf_broadcaster_ptr; // Clean up

    return 0;
}
