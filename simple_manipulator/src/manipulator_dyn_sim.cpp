#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <cmath>


//DLM parameters
float m1 = 3.0;
float m2 = 3.0;
float M1 = 1.5;
float M2 = 1.5;

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

//Torques 
float tau1 = 0; 
float tau2 = 0;

float dt,result;

//Vectors and matrix initialization
Eigen::Vector2f q;
Eigen::Vector2f q_dot;
Eigen::Vector2f q_ddot;

Eigen::Matrix2f M;
Eigen::Matrix2f C;
Eigen::Vector2f G;
Eigen::Vector2f t;
    

Eigen::Vector2f states;

//Functions initialization

Eigen::Matrix2f M_Compute(float q1, float q2);
Eigen::Matrix2f C_Compute(float q1, float q2,float q1_dot, float q2_dot);
Eigen::Vector2f G_Compute(float q1, float q2);

float to_pi(float x1){

    result = fmod((x1 + M_PI),(2*M_PI));

    if (result < 0){
        result += 2 * M_PI;
    }
    return result - M_PI;

}


int main(int argc, char **argv){

    //Node Set up
    ros::init(argc, argv, "manipulator_dyn_sim");
    ros::NodeHandle nh;

    //Publisher set up
    ros::Publisher joints_pub = nh.advertise<sensor_msgs::JointState>("/joint_states",10);

    //Rate setup
    ros::Rate loop_rate(100);
    dt = (1.0/200.0);

    t << 0 , 0;

    sensor_msgs::JointState estados;

    estados.name.resize(2);               // A 2 unit size vector
    estados.position.resize(2);           // A 2 unit size vector
    estados.velocity.resize(2);           // A 2 unit size vector
    estados.name[0] = "joint2";           // Joint name
    estados.name[1] = "joint3";           // Joint name


    std::cout << "Node is running" << std::endl;

    while (ros::ok()){

        //angular velocity integration 
        q1 += q1_dot * dt;
        q2 += q2_dot * dt;

        //Wrap to PI
        q1 = to_pi(q1);
        q2 = to_pi(q2);

        //ODE 
        q_ddot = M_Compute(q1,q2).inverse() * (t - (C_Compute(q1,q2,q1_dot,q2_dot) * q_dot ) - G_Compute(q1,q2));

        std::cout << "****" << std::endl;
        std::cout << q_ddot << std::endl;
        std::cout << "----" << std::endl;
        std::cout << M << std::endl;
        std::cout << "----" << std::endl;
        std::cout << G << std::endl;
        std::cout << "----" << std::endl;
        std::cout << C << std::endl;


        q1_ddot = q_ddot(0);
        q2_ddot = q_ddot(1);

        //Angular acceleration integration
        q1_dot += q1_ddot * dt;
        q2_dot += q2_ddot * dt;

        //Time Stamp
        estados.header.stamp = ros::Time::now(); 

        q(0) = q1;
        q(1) = q2;

        q_dot(0) = q1_dot;
        q_dot(1) = q2_dot;

        //Position and velocities values on joint_states
        estados.position[0] = q(0);    
        estados.velocity[0] = q_dot(0); 

        estados.position[1] = q(1);    
        estados.velocity[1] = q_dot(1); 
        
        //joint_state publish
        joints_pub.publish(estados);

        

        loop_rate.sleep();
         
    }
    
}


//ODE Matrices computations

Eigen::Vector2f G_Compute(float q1 , float q2){


    G << m1*a*cos(q1) + M1*l1*cos(q1) + m2*(l1*cos(q1) + d * cos(q1 + q2)) + M2*(l1*cos(q1) + l1 * cos(q1 + q2)),

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

