#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/QuaternionStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <dynamic_reconfigure/server.h>
#include <controllers/setPIDConfig.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>

using namespace geometry_msgs;
using namespace std;
using namespace ros;
using namespace Eigen;

using Eigen::Vector4d;

// Subscribers
ros::Subscriber odometry_subscriber;
ros::Subscriber trajectory_subscriber;
ros::Subscriber trajectory_velocity_subscriber;

// Publishers
ros::Publisher velocity_publisher;

// Actual state
Vector4d pose;
Vector4d velocity;
Vector4d pose_d;
Vector4d velocity_d;

Vector4d error;
Vector4d error_old;
Vector4d error_i;
Vector4d error_d;

// Gains
double k_p;
double k_i;
double k_d;

bool new_odometry;

class PID{
        public:
          PID(int, char**);
          ~PID();
          void run();
        private:
          double denormalizeAngle(double a1, double a2);
};
