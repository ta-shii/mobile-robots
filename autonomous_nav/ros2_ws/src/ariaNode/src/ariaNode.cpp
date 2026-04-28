/*
*   A basic node for ros2 that runs with ariaCoda
*   To run use 'ros2 run ariaNode ariaNode -rp /dev/ttyUSB0'
*
*   Author: Kieran Quirke-Brown
*   Date: 12/01/2024
*
*   Part 3 addition: publishes /odom and odom→base_link TF so that
*   slam_toolbox and Nav2 can localise the robot.
*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <signal.h>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "Aria/Aria.h"

bool stopRunning = false;

using namespace std::chrono_literals;

/*
*   Basic ROS node that updates velocity of pioneer robot, Aria doesn't like
*   being spun as a node therefore we just use a single subscriber.
*
*   Extended for Part 3: also publishes /odom and broadcasts odom→base_link TF
*   by reading wheel-encoder position from the ArRobot object.
*/
class ariaNode : public rclcpp::Node {
    public:
        ariaNode(float* forwardSpeed, float* rotationSpeed, ArRobot* ariaRobot)
            : Node("aria_node") {
            currentForwardSpeed  = forwardSpeed;
            currentRotationSpeed = rotationSpeed;
            robot = ariaRobot;

            cmdVelSub = create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10,
                std::bind(&ariaNode::cmdVelCallback, this, std::placeholders::_1)
            );

            odomPub       = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
            tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        // Called every loop iteration – reads AriaCoda pose and publishes odom + TF.
        void publishOdom() {
            auto now = this->get_clock()->now();

            // AriaCoda units: position mm, angle degrees, velocity mm/s, rotvel deg/s
            double x   = robot->getX()      / 1000.0;          // mm  → m
            double y   = robot->getY()      / 1000.0;          // mm  → m
            double th  = robot->getTh()     * M_PI / 180.0;    // deg → rad
            double vx  = robot->getVel()    / 1000.0;          // mm/s  → m/s
            double vth = robot->getRotVel() * M_PI / 180.0;    // deg/s → rad/s

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, th);

            // --- TF broadcast: odom → base_link ---
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp    = now;
            tf.header.frame_id = "odom";
            tf.child_frame_id  = "base_link";
            tf.transform.translation.x = x;
            tf.transform.translation.y = y;
            tf.transform.translation.z = 0.0;
            tf.transform.rotation.x = q.x();
            tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z();
            tf.transform.rotation.w = q.w();
            tfBroadcaster->sendTransform(tf);

            // --- /odom topic ---
            nav_msgs::msg::Odometry odom;
            odom.header.stamp    = now;
            odom.header.frame_id = "odom";
            odom.child_frame_id  = "base_link";
            odom.pose.pose.position.x  = x;
            odom.pose.pose.position.y  = y;
            odom.pose.pose.position.z  = 0.0;
            odom.pose.pose.orientation = tf.transform.rotation;
            odom.twist.twist.linear.x  = vx;
            odom.twist.twist.angular.z = vth;
            odomPub->publish(odom);
        }

    private:
        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
            double linearSpeed  = msg->linear.x;
            double angularSpeed = msg->angular.z;
            *currentForwardSpeed  = linearSpeed;
            *currentRotationSpeed = angularSpeed;
            RCLCPP_DEBUG(this->get_logger(), "message received.");
        }

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
        float* currentForwardSpeed;
        float* currentRotationSpeed;
        ArRobot* robot;
};

void my_handler(int s) {
    printf("Caught signal %d\n", s);
    stopRunning = true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    Aria::init();
    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();
    ArRobot* robot = new ArRobot();

    signal(SIGINT, my_handler);

    ArRobotConnector robotConnector(&parser, robot);
    if(!robotConnector.connectRobot()) {
        ArLog::log(ArLog::Terse, "simpleConnect: Could not connect to the robot.");
        if(parser.checkHelpAndWarnUnparsed()) {
            Aria::logOptions();
            Aria::exit(1);
        }
    }

    robot->setAbsoluteMaxTransVel(400);

    float forwardSpeed  = 0.0;
    float rotationSpeed = 0.0;

    robot->runAsync(true);
    robot->enableMotors();

    auto aNode = std::make_shared<ariaNode>(&forwardSpeed, &rotationSpeed, robot);
    RCLCPP_DEBUG(aNode->get_logger(), "Before Spin!...");

    while (!stopRunning) {
        rclcpp::spin_some(aNode);
        robot->lock();
        robot->setVel(forwardSpeed * 500);
        robot->setRotVel(rotationSpeed * 50);
        robot->unlock();
        aNode->publishOdom();
    }

    RCLCPP_DEBUG(aNode->get_logger(), "After Spin");
    robot->disableMotors();
    robot->stopRunning();
    robot->waitForRunExit();
    RCLCPP_DEBUG(aNode->get_logger(), "ending Aria node");
    Aria::exit(0);
    return 0;
}
