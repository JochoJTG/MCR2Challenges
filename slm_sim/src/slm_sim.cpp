#include <ros/ros.h>
#include <cmath>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>


//SLM parameters
float k = 0.01;
float m = 0.75;
float l = 0.36 ;
float g = 9.8;
float tau = 0.0;
float x1 = 0.0;
float x2 = 0.0;
float a = l/2;
float J = (4.0/3.0) * m*a*a;
float x2_dot,dt,result;



void Callback(const std_msgs::Float32::ConstPtr& data){

    //Input torque data 
    tau = data->data;
}


float to_pi(float x1){

    result = fmod((x1 + M_PI),(2*M_PI));

    if (result < 0){
        result += 2 * M_PI;
    }
    return result - M_PI;

}

int main(int argc, char **argv){

    //Node set up
    ros::init(argc, argv, "slm_sim");
    ros::NodeHandle nh;

    //Publishers set up
    ros::Publisher slm_pub = nh.advertise<sensor_msgs::JointState>("/joint_states",10);
    ros::Publisher states_pub = nh.advertise<geometry_msgs::Vector3>("/states",10);
    //Subscribers setup
    ros::Subscriber slm_sub = nh.subscribe("/tau",10, &Callback);

    //Rate set up
    ros::Rate loop_rate(100);
    

    geometry_msgs::Vector3 qstates;
    sensor_msgs::JointState states;
    
    states.name.resize(1);               // A 1 unit size vector
    states.position.resize(1);           // A 1 unit size vector
    states.velocity.resize(1);           // A 1 unit size vector
    states.name[0] = "joint2";           // Joint name

    dt = 0.01; 

    while(ros::ok()){


        
        x1 += x2 * dt;  

        x1 = to_pi(x1);

        x2_dot = (1/(J+m*a*a)) * (tau - (m*g*a)*cos(x1) - k*x2);

        x2 += x2_dot * dt;

        //Time Stamp
        states.header.stamp = ros::Time::now(); 
        //Position and velocities values on jpint_states
        states.position[0] = x1;    
        states.velocity[0] = x2; 

        //joint_states topic publish
        slm_pub.publish(states);

        //x1 and x2 to staates topic
        qstates.x = x1;
        qstates.y = x2;

        //states topic publish
        states_pub.publish(qstates);

        ros::spinOnce();

        loop_rate.sleep();
    }

}
