#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// Class to handle publishing and subscription
class SubscribeAndPublish
{
public:
  void SubscribeAndPublish()
  {
    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // drive /ball_chaser/command_robot service with a handle_drive_request callback function
    service_ = n_.advertiseService("/ball_chaser/command_robot", &SubscribeAndPublish::handle_drive_request, this);

    //Topic you want to subscribe
    //sub_ = n_.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback, this);
  }

  // Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
  // This function should publish the requested linear x and angular velocities to the robot wheel joints
  // After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
  bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res) {
    ROS_INFO("Drive to Target request is received, linear_x: %1.2f angular_z: %1.2f", (float)req.linear_x, (float)req.angular_z);

    // Creating a motor_command object of type geomery_msgs::Twist
    geometry_msgs::Twist motor_command;

    // Set wheel velocities, forward [0.5, 0.0]
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;

    // Publish angles to drive the robot
    pub_.publish(motor_command);

    // Returning a response message
    res.msg_feedback = "motor velocities set to linear x: " + std::to_string(req.linear_x) + " angular z: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::ServiceServer service_;

};//End of class SubscribeAndPublish

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");
    
    SubscribeAndPublish SAPObject;
    SAPObject.SubscribeAndPublish();

    ROS_INFO("Service is ready...");

    // Handle ROS communication events
    ros::spin();

    return 0;
}