#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <vector>

// Class handling comms
class MessageTasks
{
public:
    void SubscribeAndPublish()
    {
        // Define a client service capable of requesting services from command_robot
        client_ = n_.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

        // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
        sub_ = n_.subscribe("/camera/rgb/image_raw", 10, &MessageTasks::process_image_callback, this);

    }

    // This callback function continuously executes and reads the image data
    void process_image_callback(const sensor_msgs::Image& img)
    {

        int white_pixel = 255;
        int left_center_cut = img.step / 3;
        int center_right_cut = (2 * img.step) / 3;
        int line_width = -1;

        float max_forward_vel = 0.5;
        float max_turn_vel = 0.5;     
        std::vector<int> arr;

        for(int i=0; i < img.height; i++) {  
            int row_pixels = i * img.step;
            for(int j=0; j < img.step; j+=3) {
                int total_pixels = row_pixels + j;
                // check if pixel is white and assign the line position to line_width
                if ((img.data[total_pixels] == white_pixel) && (img.data[total_pixels + 1] == white_pixel) && (img.data[total_pixels + 2] == white_pixel)) {
                    arr.push_back(j);
                }
            }
        }

        if (arr.empty() == false) {
            int sum_of_array = 0;
            for (auto it = arr.begin(); it != arr.end(); ++it) {
                sum_of_array += *it;
            }
            line_width = sum_of_array / arr.size();
        }

        std::string test_str = "found value: " + std::to_string(line_width) + " left divide: " + std::to_string(left_center_cut) + " right divide: " + std::to_string(center_right_cut);
        ROS_INFO_STREAM(test_str);


        if (line_width >= 0 && line_width < left_center_cut) {
            // move left
            ROS_INFO_STREAM("Moving the robot left");
            this->drive_robot(0.0, max_turn_vel);
        } else if (line_width >= left_center_cut && line_width < center_right_cut) {
            // move forward
            ROS_INFO_STREAM("Moving the robot forward");
            this->drive_robot(max_forward_vel, 0.0);
        } else if (line_width >= center_right_cut && line_width <= img.step) {
            // move right
            ROS_INFO_STREAM("Moving the robot right");
            this->drive_robot(0.0, -max_turn_vel);
        } else { 
            // stop, white ball not there
            ROS_INFO_STREAM("Stopping the robot");
            this->drive_robot(0.0, 0.0);
        }

        // Wait 0.1 seconds before issuing another command 
        ros::Duration(0.1).sleep();

        // TODO: Loop through each pixel in the image and check if there's a bright white one
        // Then, identify if this pixel falls in the left, mid, or right side of the image
        // Depending on the white ball position, call the drive_bot function and pass velocities to it
        // Request a stop when there's no white ball seen by the camera
    }

    // This function calls the command_robot service to drive the robot in the specified direction
    void drive_robot(float lin_x, float ang_z)
    {
        // Request centered joint angles [1.57, 1.57]
        ball_chaser::DriveToTarget srv;
        srv.request.linear_x = lin_x;
        srv.request.angular_z = ang_z;

        // Call the safe_move service and pass the requested joint angles
        if (!client_.call(srv))
            ROS_ERROR("Failed to call service command_robot");
    }

private:
    ros::NodeHandle n_; 
    ros::Subscriber sub_;
    ros::ServiceClient client_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "process_image");
    
    MessageTasks SubscribeObject;
    SubscribeObject.SubscribeAndPublish();

    ros::spin();

    return 0;
}