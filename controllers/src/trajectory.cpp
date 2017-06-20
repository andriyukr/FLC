#include "controllers/trajectory.h"

void dynamicReconfigureCallback(controllers::setTrajectoryConfig &config, uint32_t level){
    trajectory_type = config.trajectory;
    if(level == 0){
        waypoint = 0;
        if(trajectory_type == 6)
            t = M_PI/2;
        else
            if(trajectory_type == 7)
                t = 1;
            else
                t = 0;
    }

    speed = config.speed;

    pose_d << config.x_d, config.y_d, config.z_d, config.yaw_d / 180 * M_PI;
}

// Constructor
Trajectory::Trajectory(int argc, char** argv){
    ros::init(argc, argv, "Trajectory");
    ros::NodeHandle node_handle;

    trajectory_publisher = node_handle.advertise<geometry_msgs::QuaternionStamped>("/uav/trajectory", 1);
    velocity_publisher = node_handle.advertise<geometry_msgs::QuaternionStamped>("/uav/trajectory_velocity", 1);

    pose_d << 0, 0, 0, 0;
    trajectory_type = 0;
    speed = 1;

    string line;
    ifstream myfile("/home/ste/catkin_ws/src/controllers/policy/waypoints.txt");
    if(myfile.is_open()){

      int points;
      while(getline(myfile, line))
            ++points;
      waypoints = MatrixXd(points, 3);

      ifstream myfile("/home/ste/catkin_ws/src/controllers/policy/waypoints.txt");
      for(int i = 0; getline(myfile, line); ++i){
          string delimiter = "\t";

          size_t pos = 0;
          for(int j = 0; (pos = line.find(delimiter)) != string::npos; ++j) {
              waypoints(i, j) = atof(line.substr(0, pos).c_str());
              line.erase(0, pos + delimiter.length());
          }
          waypoints(i, 2) = atof(line.c_str());
      }
      myfile.close();
    }
    else
        cout << "Unable to open file: /home/andriy/catkin_ws/src/controllers/policy/waypoints.txt";

    //cout << "[Trajectory] waypoints:\n" << waypoints << endl;

    waypoint = 0;
}

// Destructor
Trajectory::~Trajectory(){
    ros::shutdown();
    exit(0);
}

void Trajectory::run(){
    dynamic_reconfigure::Server<controllers::setTrajectoryConfig> server;
    dynamic_reconfigure::Server<controllers::setTrajectoryConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    Vector4d trajectory;
    Vector4d velocity;
    t = 0;
    double R = sqrt(2) / 2;
    Vector3d w1;
    Vector3d w2;

    double dt = (double)1/100;
    ros::Rate rate(100);
    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        t += speed * dt;

        Matrix3d R1;
        Matrix3d R2;
        switch(trajectory_type){
        case 0: // no command
            trajectory << 0, 0, -100, 0;
            velocity << 0, 0, 0, 0;
            break;
        case 1: // hower
            trajectory << 0, 0, 1, 0;
            velocity << 0, 0, 0, 0;
            break;
        case 2: // user
            trajectory = pose_d;
            velocity << 0, 0, 0, 0;
            break;
        case 3: // waypoints
            if(waypoint < waypoints.rows() - 1){
                cout << "[Trajectory]: waypoint = " << waypoint << endl;
                w1 << waypoints.row(waypoint).transpose();
                w2 << waypoints.row(waypoint + 1).transpose();
                double d = distance(w1, w2);
                trajectory << (1 - t / d) * w1 + t / d * w2, 0;
                if(t >= d){
                    t = 0;
                    waypoint++;
                }
            }
            else
                trajectory << waypoints.bottomRows(1).transpose(), 0;
            velocity << 0, 0, 0, 0;
            break;
        case 4: // smooth waypoints
            trajectory << 0, 0, 0, 0;
            velocity << 0, 0, 0, 0;
            break;
        case 5: // circle
            trajectory << 2 * sin(t), 2 * cos(t), pose_d(2), 0;
            velocity << speed * cos(t), -speed * sin(t), 0, 0;
            break;
        case 6: // smooth 8
            trajectory << sqrt(2) / 2 * 4 / (3 - cos(2 * t)) * cos(t), -sqrt(2) / 2 * 4 / (3 - cos(2 * t)) * cos(t), 4 / (3 - cos(2 * t)) * sin(2 * t) / 2 + pose_d(2) + 0.5, 0;
            velocity << 4*speed*sin(t)/(cos(2*t) - 3) - 8*speed*cos(t)*sin(2*t)/pow(cos(2*t) - 3, 2), -4*speed*pow(sin(2*t), 2)/pow(cos(2*t) - 3, 2) - 4*speed*cos(2*t)/(cos(2*t) - 3), 0, 0;
            break;
        case 7: // aggressive 8
            t = fmod(t - 0 * speed * dt / 3, 4 + 2 * M_PI * R);
            if(t < 2){
                trajectory << (R - R * t), (R - R * t), 0, 0;
                velocity << -R, -R, 0, 0;
            }else
                if(t < 2 + M_PI * R){
                    trajectory << (-R - sin((t - 2) / R) * R), (-cos((t - 2) / R) * R), 0, 0;
                    velocity << -cos((t - 2) / R), sin((t - 2) / R), 0, 0;
                }else
                    if(t < 4 + M_PI * R){
                        trajectory << (-R + R * (t - (2 + M_PI * R))), (R - R * (t - (2 + M_PI * R))), 0, 0;
                        velocity << R, -R, 0, 0;
                    }else
                        if(t < 4 + 2 * M_PI * R){
                            trajectory << (R + sin((t - (4 + M_PI * R)) / R) * R), (-cos((t - (4 + M_PI * R)) / R) * R), 0, 0;
                            velocity << cos((M_PI * R - t + 4) / R), -sin((M_PI * R - t + 4) / R), 0, 0;
                        }
            trajectory = 1 * trajectory;
            velocity = 1 * speed * velocity;
            R1 << cos(-M_PI / 4), -sin(-M_PI / 4), 0, sin(-M_PI / 4), cos(-M_PI / 4), 0, 0, 0, 1;
            R2 << cos(M_PI / 18), 0, sin(M_PI / 18), 0, 1, 0, -sin(M_PI / 18), 0, cos(M_PI / 18);
            trajectory.head(3) = R1 * R2 * trajectory.head(3);
            velocity.head(3) = R1 * R2 * velocity.head(3);
            trajectory(2) += pose_d(2);
            break;
        case 8: // rectangle
            t = fmod(t - (1 - 1.0/1) * speed * dt, 8);
            if(t < 2){
                trajectory << 1, 1 - t, 0, 0;
                velocity << 0, -1, 0, 0;
            }
            else
                if(t < 4){
                    trajectory << 1 - (t - 2), -1, 0, 0;
                    velocity << -1, 0, 0, 0;
                }
                else
                    if(t < 6){
                        trajectory << -1, -1 + (t - 4), 0, 0;
                        velocity << 0, 1, 0, 0;
                    }
                    else
                        if(t < 8){
                            trajectory << -1 + (t - 6), 1, 0, 0;
                            velocity << 1, 0, 0, 0;
                        }
            trajectory = 1 * trajectory;
            velocity = speed * velocity;
            R1 << cos(M_PI / 18), 0, sin(M_PI / 18), 0, 1, 0, -sin(M_PI / 18), 0, cos(M_PI / 18);
            trajectory.head(3) = R1 * trajectory.head(3);
            velocity.head(3) = R1 * velocity.head(3);
            trajectory(2) += pose_d(2);
            break;
        case 9: // waypoints-square
            t = fmod(t - (1 - 1.0/1) * speed * dt, 4);
            if(t < 1)
                trajectory << 1, -1, 0.2, 0;
            else
                if(t < 2)
                    trajectory << -1, -1, -0.2, 0;
                else
                    if(t < 3)
                        trajectory << -1, 1, -0.2, 0;
                    else
                        if(t < 4)
                            trajectory << 1, 1, 0.2, 0;
            /*if(t < 1)
                trajectory << 1, 1, -0.2, 0;
            else
                if(t < 2)
                    trajectory << 1, 0, -0.2, 0;
                else
                    if(t < 3)
                        trajectory << 1, -1, -0.2, 0;
                    else
                        if(t < 4)
                            trajectory << 0, -1, 0, 0;
                        else
                            if(t < 5)
                                trajectory << -1, -1, 0.2, 0;
                            else
                                if(t < 6)
                                    trajectory << -1, 0, 0.2, 0;
                                else
                                    if(t < 7)
                                        trajectory << -1, 1, 0.2, 0;
                                    else
                                        if(t < 8)
                                            trajectory << 0, 1, 0, 0;*/
            trajectory = 1 * trajectory;
            velocity << 0, 0, 0, 0;
            //R1 << cos(-M_PI / 18), 0, sin(-M_PI / 18), 0, 1, 0, -sin(-M_PI / 18), 0, cos(-M_PI / 18);
            //trajectory.head(3) = R1 * trajectory.head(3);
            trajectory(2) += pose_d(2);
            break;
        }

        geometry_msgs::QuaternionStamped trajectory_msg;
        trajectory_msg.header.stamp = ros::Time::now();
        trajectory_msg.quaternion.x = trajectory(0);
        trajectory_msg.quaternion.y = trajectory(1);
        trajectory_msg.quaternion.z = trajectory(2);
        trajectory_msg.quaternion.w = trajectory(3);
        trajectory_publisher.publish(trajectory_msg);

        geometry_msgs::QuaternionStamped velocity_msg;
        velocity_msg.header.stamp = ros::Time::now();
        velocity_msg.quaternion.x = velocity(0);
        velocity_msg.quaternion.y = velocity(1);
        velocity_msg.quaternion.z = velocity(2);
        velocity_msg.quaternion.w = velocity(3);
        velocity_publisher.publish(velocity_msg);

        //cout << "[Trajectory]: trajectory = " << trajectory.transpose() << endl;

        //results << ros::Time::now() << ", " << position(0) << ", " << position(1) << ", " << position(2) << ", " << position_real(0) << ", " << position_real(1) << ", " << position_real(2) << ", " << position_d(0) << ", " << position_d(1) << ", " << position_d(2) << endl;

        //cout << "[Trajectory] v_x = " << velocity_msg.x << ", v_y = " << velocity_msg.y << ", v_z = " << velocity_msg.z << endl;
    }
}

double Trajectory::distance(Vector3d v1, Vector3d v2){
    return sqrt(pow(v1(0) - v2(0), 2) + pow(v1(1) - v2(1), 2) + pow(v1(2) - v2(2), 2));
}

int main(int argc, char** argv){
    cout << "[Trajectory] Trajectory generator is running..." << endl;

    Trajectory* controller = new Trajectory(argc, argv);

    controller->run();
}
