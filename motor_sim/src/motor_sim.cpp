#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

//Parameters
float R = 6.0;
float L = 0.3;
float k1 = 0.04;
float k2 = k1;
float J = 0.00008;
float b = 0.00025;
float m = 0.0;

//Initial values
float current_dot = 0;
float current = 0;
float omega_dot = 0;
float omega = 0;
float input, dt, counter;

void MotorCallback(const std_msgs::Float32::ConstPtr& data){

    //Input data 
    input = data->data;
}

void stop(){
    
}

int main(int argc, char  **argv){

    //Node declaration
    ros::init(argc, argv, "motor_sim");
    ros::NodeHandle nh;

    //Publisher set up
    ros::Publisher motor_sim_pub = nh.advertise<std_msgs::Float32>("/motor_output",10);
    //Subscriber set up
    ros::Subscriber motor_sim_sub = nh.subscribe("/motor_input",10, &MotorCallback);

    //Loop Rate declaration
    ros::Rate loop_rate(100);


    //Time interval (1/100hz)
    dt = 0.01;

    std_msgs::Float32 output;

    while (ros::ok())
    {
        //Run node
        
        //Current dot ODE 
        current_dot = -(R/L)*current-(k1/L)*omega+(1/L)*(input);
        //Current dot integration
        current += current_dot*dt;
        
        //Omega dot ODE
        omega_dot = (k2/J)*current-(b/J)*omega-(1/J)*m;
        //Omega dot integration
        omega += omega_dot*dt;

        
        //output variable setup
        output.data = omega;

        motor_sim_pub.publish(output);

        counter += 1;

        //Cycle counter for 10s of data
        if (counter > 1000){
            break;
        }

        std::cout << input << omega << std::endl;

        ros::spinOnce();

        loop_rate.sleep();
    }
    

}