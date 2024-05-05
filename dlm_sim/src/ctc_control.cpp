#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <cmath>

//DLM parameters
float m1 = 1.5;
float m2 = 1.5;
float M1 = 1;
float M2 = 2;

float a = 0.2;
float d = 0.2;
float l1 = 0.4;
float l2 = 0.4;

//Initial Values
float q1 = 0.10;
float q2 = 0.1;
float q1_dot = 0.0;
float q2_dot =  0.0;
float q1_ddot = 0.0;
float q2_ddot =  0.0;

float g = 9.81; //Gravity

float dt,r1_dot,r2_dots;

float kp = 100;
float kd = 20;




Eigen::Vector2f torque;
Eigen::Vector2f e;
Eigen::Vector2f e_dot;
Eigen::Vector2f u;
Eigen::Vector2f r;



Eigen::Vector2f r_dot;
Eigen::Vector2f r_ddot;
Eigen::Vector2f r_prev;
Eigen::Vector2f r_dprev;
Eigen::Vector2f q;


Eigen::Vector2f q_dot;

Eigen::Matrix2f M;
Eigen::Matrix2f C;
Eigen::Vector2f G;
Eigen::Vector2f t;



Eigen::Matrix2f M_Compute(float q1, float q2);
Eigen::Matrix2f C_Compute(float q1, float q2,float q1_dot, float q2_dot);
Eigen::Vector2f G_Compute(float q1, float q2);


 
void state_feedBack(const geometry_msgs::Quaternion::ConstPtr& q_feedback){
    q1 = q_feedback -> x;
    q2 = q_feedback -> y;
    q1_dot = q_feedback -> z;
    q2_dot = q_feedback -> w;
}

void ref_data(const geometry_msgs::Vector3::ConstPtr& ref){
    
    r(0) = ref->x;
    r(1) = ref->y;
}
void refd_data(const geometry_msgs::Vector3::ConstPtr& ref){
    
    r_dot(0) = ref->x;
    r_dot(1) = ref->y;
}

void refdd_data(const geometry_msgs::Vector3::ConstPtr& ref){
    
    r_ddot(0) = ref->x;
    r_ddot(1) = ref->y;
}




int main(int argc, char **argv){

    //Node Set up
    ros::init(argc, argv, "ctc_control");
    ros::NodeHandle nh;

    //Publisher set up
    ros::Publisher ctrl_pub = nh.advertise<geometry_msgs::Vector3>("/ctrl_dlm",10);
    ros::Publisher errors_pub = nh.advertise<geometry_msgs::Quaternion>("/errors_topic",10);
    ros::Publisher ref_pub = nh.advertise<geometry_msgs::Quaternion>("/ref_topic",10);
    ros::Publisher u_pub = nh.advertise<geometry_msgs::Vector3>("/utopic",10);
    


    //Subscriber set up
    ros::Subscriber dlm_dyn_fb = nh.subscribe<geometry_msgs::Quaternion>("/joint_feedback",10, &state_feedBack); 
    ros::Subscriber ref_sub = nh.subscribe<geometry_msgs::Vector3>("/ref_data",10, &ref_data); 
    ros::Subscriber refd_sub = nh.subscribe<geometry_msgs::Vector3>("/refd_data",10, &refd_data); 
    ros::Subscriber refdd_sub = nh.subscribe<geometry_msgs::Vector3>("/refdd_data",10, &refdd_data); 

    //Rate setup
    ros::Rate loop_rate(100);

    dt = 0.01;

    q << 0.10,
        0.1;

    r << 1 + 0.3*sin(0),
        0.5 + 0.7 * sin(0);


    geometry_msgs::Vector3 ctrl_values;
    geometry_msgs::Vector3 u_values;
    geometry_msgs::Quaternion errors;
    geometry_msgs::Quaternion references;


    while (ros::ok()){

        e = r - q;
        e_dot = r_dot - q_dot;

        u = -kp * e -  kd * e_dot;


        torque = M_Compute(q1,q2) * (r_ddot - u) + (C_Compute(q1,q2,q1_dot,q2_dot) * q_dot) + G_Compute(q1,q2);



        q(0) = q1;
        q(1) = q2;
        q_dot(0) = q1_dot;
        q_dot(1) = q2_dot;

        ctrl_values.x = torque(0);
        ctrl_values.y = torque(1);

        errors.x = e(0);
        errors.y = e(1);
        errors.z = e_dot(0);
        errors.w = e_dot(1);

        u_values.x = u(0);
        u_values.y = u(1);



        ctrl_pub.publish(ctrl_values);
        u_pub.publish(u_values);
        ref_pub.publish(references);
        errors_pub.publish(errors);


        ros::spinOnce();

        loop_rate.sleep();

    }
    

}

Eigen::Vector2f G_Compute(float q1 , float q2){


    G << m1*a*cos(q1) + M1*l1*cos(q1) + m2*(l1*cos(q1) + d * cos(q1 + q2)) + M2*(l1*cos(q1) + l2 * cos(q1 + q2)),

        cos(q1+q2) * (m2*d + M2 * l2);

    return g * G;
}

Eigen::Matrix2f M_Compute(float q1,float q2){

    M << (m1 * pow(a,2) + M1 * pow(l1,2) + m2*(pow(l1,2) + pow(d,2) + 2*l1*d*cos(q2)) + M2*(pow(l1,2) + pow(l2,2) + 2*l1*l2*cos(q2))),  (m2*d*(l1*cos(q2) + d) + M2*l2*(l1*cos(q2)+l2)),

        (m2*d*(l1*cos(q2) + d) + M2*l2*(l1*cos(q2)+l2)), (m2*pow(d,2) + M2*pow(l2,2));

    return M;
}

Eigen::Matrix2f C_Compute(float q1, float q2,float q1_dot, float q2_dot){



    C << -2 * l1 * sin(q2) * (m2 * d +M2*l2) * q2_dot , -l1 * sin(q2) * (m2 * d +M2*l2) * q2_dot,

        l1 * sin(q2) * (m2 * d + M2*l2) * q1_dot, 0;

    return C;
}

