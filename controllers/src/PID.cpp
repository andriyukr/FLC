#include "controllers/PID.h"

void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    tf::Quaternion q(odometry_msg->pose.pose.orientation.x, odometry_msg->pose.pose.orientation.y, odometry_msg->pose.pose.orientation.z, odometry_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double x, y, z;
    m.getEulerZYX(z, y, x);
    pose << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, odometry_msg->pose.pose.position.z, z;
    velocity << odometry_msg->twist.twist.linear.x, odometry_msg->twist.twist.linear.y, odometry_msg->twist.twist.linear.z, odometry_msg->twist.twist.angular.z;
    //cout << "[PID] yaw = " << (180 * z / M_PI) << endl;

    new_odometry = true;
}

void trajectoryCallback(const QuaternionStamped& trajectory_msg){
    pose_d << trajectory_msg.quaternion.x, trajectory_msg.quaternion.y, trajectory_msg.quaternion.z, trajectory_msg.quaternion.w;
    //cout << "[PID] position_d: " << pose_d.transpose() << endl;
}

void trajectoryVelocityCallback(const QuaternionStamped& velocity_msg){
    velocity_d << velocity_msg.quaternion.x, velocity_msg.quaternion.y, velocity_msg.quaternion.z, velocity_msg.quaternion.w;
    //cout << "[PID] velocity_d: " << velocity_d.transpose() << endl;
}

void dynamicReconfigureCallback(controllers::setPIDConfig &config, uint32_t level){
    k_p = config.k_p;
    k_i = config.k_i;
    k_d = config.k_d;
}

// Constructor
PID::PID(int argc, char** argv){
    ros::init(argc, argv, "PID");
    ros::NodeHandle node_handle;

    odometry_subscriber = node_handle.subscribe("/uav/odometry", 1, odometryCallback);
    trajectory_subscriber = node_handle.subscribe("/uav/trajectory", 1, trajectoryCallback);
    trajectory_velocity_subscriber = node_handle.subscribe("/uav/trajectory_velocity", 1, trajectoryVelocityCallback);

    velocity_publisher = node_handle.advertise<geometry_msgs::Quaternion>("/uav/command_velocity", 1);

    pose << 0, 0, 0, 0;
    pose_d << 0, 0, 0, 0;
    velocity << 0, 0, 0, 0;
    velocity_d << 0, 0, 0, 0;

    if(argc > 1){
        k_p = atof(argv[1]);
        k_i = atof(argv[2]);
        k_d = atof(argv[3]);
    }
    else{
        k_p = 1.0;
        k_i = 0.1;
        k_d = 0.004;
    }

    error_i << 0, 0, 0, 0;

    new_odometry = false;
}

// Destructor
PID::~PID(){
    ros::shutdown();
    exit(0);
}

double PID::denormalizeAngle(double a1, double a2){
    if(abs(a2 - a1) > M_PI){
        if(a2 < a1)
            a1 -= 2 * M_PI;
        else
            a1 += 2 * M_PI;
    }
    return a1;
}

void PID::run(){
    dynamic_reconfigure::Server<controllers::setPIDConfig> server;
    dynamic_reconfigure::Server<controllers::setPIDConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    double dt = (double)1/100;
    ros::Rate rate(100);
    double time = 0;
    int c = 0;
    geometry_msgs::Quaternion velocity_msg;

    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        if(pose_d(2) > -10 && new_odometry){ // command
            pose(3) = denormalizeAngle(pose(3), pose_d(3));

            //ros::Time begin = ros::Time::now();

            error = pose_d - pose;

            error_i += error * dt;
            error_d = velocity_d - velocity;

            velocity_msg.x = k_p * error(0) + k_i * error_i(0) + k_d * error_d(0);
            velocity_msg.y = k_p * error(1) + k_i * error_i(1) + k_d * error_d(1);
            velocity_msg.z = k_p * error(2) + k_i * error_i(2) + k_d * error_d(2);
            velocity_msg.w = 0; //k_p * error(3);

            //time += (ros::Time::now() - begin).toSec() * 1000;
            //c++;

            velocity_publisher.publish(velocity_msg);

            //cout << "[PID]: time = " << (time/c) << endl;
        }

        new_odometry = false;
    }
}

int main(int argc, char** argv){
    cout << "[PID] PID position controller is running..." << endl;

    PID* controller = new PID(argc, argv);

    controller->run();
}
