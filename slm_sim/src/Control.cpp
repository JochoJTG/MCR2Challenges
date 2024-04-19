#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>

float r,r_dot,r_ddot,e,e_dot, torque;
float r_prev, r_dprev,v,dt;

//Initial Parameters
float k = 0.01;
float m = 0.75;
float l = 0.36 ;
float g = 9.8;
float tau = 0.0;
float x1 = 0.0;
float x2 = 0.0;
float a = l/2;
float J = (4.0/3.0) * m*a*a;


//Control Parameters
float kp = 4;
float kd = 3.3;


void StateCallback(const geometry_msgs::Vector3::ConstPtr& data){

    //x1 and x2 callback
    x1 = data->x;
    x2 = data->y;
}

void RefdataCallback(const std_msgs::Float32::ConstPtr& refdata){

    //reference data callback
    r = refdata->data;

}

int main(int argc, char **argv){

    //Node Set up
    ros::init(argc, argv, "Controller");
    ros::NodeHandle nh;

    //Publishers set up
    ros::Publisher tau_pub = nh.advertise<std_msgs::Float32>("/tau",10);
    ros::Publisher error_pub = nh.advertise<geometry_msgs::Vector3>("errors",10);
    ros::Publisher ref_pub = nh.advertise<geometry_msgs::Vector3>("ref",10);

    //Subscribers set up
    ros::Subscriber refdata_pub = nh.subscribe<std_msgs::Float32>("/refdata",10,&RefdataCallback);
    ros::Subscriber state_sub = nh.subscribe<geometry_msgs::Vector3>("/states",10,&StateCallback);
    
    //Rate set up
    ros::Rate loop_rate(100);

    dt = (1.0/100.0);

    std_msgs::Float32 moment;
    geometry_msgs::Vector3 errors;
    geometry_msgs::Vector3 rs;

    while(ros::ok()){

        //Differentiate r
        r_dot = (r - r_prev)/dt;
        r_ddot = (r_dot - r_dprev)/dt;

        //Calculate errors
        e = r - x1;
        e_dot = r_dot - x2;

        //Control 
        v = kd * e_dot + kp * e;

        //torque output
        torque = (J+m*a*a)*(r_ddot + v) +  (m*g*a)*cos(x1) ;

    
        moment.data = torque;
        tau_pub.publish(moment);

        errors.x = e;
        errors.y = e_dot;

        error_pub.publish(errors);

        rs.x = r_dot;
        rs.y = r_ddot;

        ref_pub.publish(rs);

        r_prev = r;
        r_dprev = r_dot;

        ros::spinOnce();

        loop_rate.sleep();
    }

}
