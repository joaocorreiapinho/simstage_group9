#include <iostream>

#include <cstdlib>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884L
#endif na



class ReactiveController
{
private:
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber laser_sub;

    double front_obstacle_distance;
    double right_obstacle_distance;
    double left_obstacle_distance;
    bool robot_stopped;

    geometry_msgs::Twist calculateCommand()
    {
        auto msg = geometry_msgs::Twist();

        // Desired distance to the right wall
		float desired_right_wall_distance = 0.7;

        // Wall distance controller parameters
        int k_p = 1.0;
        int k_i = 0.1;
        float dt = 0.1;

        static float error;
        static float integral_error;

        float proportional;
        float integral;
        float angular_input;

        // Switch case variables
        static bool init_flag = false;
        static int swtich_case = 0;
		static int count = 0;

        if(!init_flag){
        	// INIT
        	swtich_case = 0;
        } else if (front_obstacle_distance > 0.8){
        	// PID
        	swtich_case = 1;
        } else{
        	// MANUEVER
        	swtich_case = 2;
        }
        

        switch(swtich_case) {

            case 0 :        // Initialisation
            				if(count < 30){
                                msg.linear.x = 0.0;
                                count++;
                            } else if(front_obstacle_distance > 0.8){
                                msg.linear.x = 0.8;
                            } else {
                                init_flag = true;
                            }
                            
                            break;       // and exits the switch
          
            case 1 :       	// PI controller

        					// Calculate proportional part
        					error = desired_right_wall_distance - right_obstacle_distance;
        					proportional =  error*k_p;

        					// Calculate integral part
        					integral_error += error*dt;
        					integral = integral_error*k_i;

        					// Calculate final angular velocity input
        					angular_input = integral + proportional;

        					// Write velocities for the next time step
        					msg.angular.z = angular_input;

        					if(front_obstacle_distance < 1.0){
        						msg.linear.x = 0.4;
        					} else{
        						msg.linear.x = 0.6;
        					}
                            break;

            case 2 :		// Avoidance manuever
                    		msg.linear.x = 0.00001;
                    		msg.angular.z = M_PI*5;
                    		
                    		error = 0;
                    		integral_error = 0;
                            break;
        }
        
        // Wirte message to the robot
        return msg;

    }
    


    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        right_obstacle_distance = *std::min_element(&msg->ranges [0],&msg->ranges [50]);
        front_obstacle_distance = *std::min_element(&msg->ranges [51], &msg->ranges [70]);
	    left_obstacle_distance = *std::min_element(&msg->ranges [71],&msg->ranges [120]);

	    
        //ROS_INFO("Min distance to obstacle: %f", obstacle_distance);
    }


public:
    ReactiveController(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Create a publisher object, able to push messages
        this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 5);

        // Create a subscriber for laser scans 
        this->laser_sub = n.subscribe("base_scan", 10, &ReactiveController::laserCallback, this);

    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            // Calculate the command to apply
            auto msg = calculateCommand();

            // Publish the new command
            this->cmd_vel_pub.publish(msg);

            ros::spinOnce();

            // And throttle the loop
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "reactive_controller");


    // Create our controller object and run it
    auto controller = ReactiveController();
    controller.run();
}