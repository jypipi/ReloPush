#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>

#include <zmq.hpp>
#include <string>
#include <iostream>

#include <vector>
#include <base64.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher* robot_pose_reset_ptr;
ros::Publisher* object_marker_pub_ptr;
ros::Publisher* goal_marker_pub_ptr;
ros::Publisher* boundary_pub_ptr;




namespace ReloPush {

    ////////////////// String <-> Binary ////////////////
    std::string float2binarystr(float f_in)
    {
        std::string message(reinterpret_cast<char*>(&f_in), sizeof(float));
        return message;
    }

    float binarystr2float(std::string str_in)
    {
        float receivedValue;
        memcpy(&receivedValue, str_in.data(), sizeof(float));
        return receivedValue;
    }

    std::string bool2binarystr(bool b_in)
    {
        std::string out_str;
        if(b_in)
            out_str="t";
        else
            out_str="f";

        return out_str;
    }

    bool binarystr2bool(std::string str_in)
    {
        if(str_in=="t")
            return true;
        else if(str_in=="f")
            return false;
        else {
            //??????
        }
    }

    /// Split a string by a delimiter
    std::vector<std::string> split(std::string s, std::string delimiter)
    {
        size_t pos_start = 0, pos_end, delim_len = delimiter.length();
        std::string token;
        std::vector<std::string> res;

        while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos)
        {
          token = s.substr(pos_start, pos_end - pos_start);
          pos_start = pos_end + delim_len;
          res.push_back(token);
        }

        res.push_back(s.substr(pos_start));
        return res;
    }

    class trajectory_elem
    {
    public:
        float x;
        float y;
        float yaw;
        float ref_vel; //reference velocity
        float time;
        bool is_pushing;

        trajectory_elem()
        {
            x=0;
            y=0;
            yaw=0;
            ref_vel=0;
            time=-1;
            is_pushing = false;
        }

        trajectory_elem(std::string& serialized_waypoint)
        {
            deserialize(serialized_waypoint);
        }


        trajectory_elem(float x_in, float y_in, float yaw_in, float ref_vel_in, float time_in, bool is_pushing_in)
            : x(x_in), y(y_in), yaw(yaw_in), ref_vel(ref_vel_in), time(time_in), is_pushing(is_pushing_in)
        {}

        std::string serialize()
        {
            std::string var_delim = ",,,";
            std::string temp_str=""; // string for one waypoint
            temp_str += float2binarystr(x);
            temp_str += var_delim;
            temp_str += float2binarystr(y);
            temp_str += var_delim;
            temp_str += float2binarystr(yaw);
            temp_str += var_delim;
            temp_str += float2binarystr(ref_vel);
            temp_str += var_delim;
            temp_str += float2binarystr(time);
            temp_str += var_delim;
            temp_str += bool2binarystr(is_pushing);

            return temp_str;
        }

        void deserialize(std::string& str_pose)
        {
            std::string header_delim = "!!!";
            std::string elem_delim = ";$;";
            std::string var_delim = ",,,";

            // split variables
            auto var_sp = split(str_pose,var_delim); // x y yaw vel time
            if(var_sp.size()==6)
            {
                float x_in = binarystr2float(var_sp[0]);
                float y_in = binarystr2float(var_sp[1]);
                float yaw_in = binarystr2float(var_sp[2]);
                float vel_in = binarystr2float(var_sp[3]);
                float time_in = binarystr2float(var_sp[4]);
                bool is_pushing_in = binarystr2bool(var_sp[5]);

                x=x_in; y=y_in; yaw=yaw_in; ref_vel=vel_in; time=time_in; is_pushing=is_pushing_in;
            }
            else if(var_sp.size()==3)
            {
                float x_in = binarystr2float(var_sp[0]);
                float y_in = binarystr2float(var_sp[1]);
                float yaw_in = binarystr2float(var_sp[2]);

                x=x_in; y=y_in; yaw=yaw_in;
            }
            else
            {
                std::cout << "Cannot deserialize" << std::endl;
            }
        }

        // Print function for trajectory_elem
        void print() const {
            std::cout << "(x=" << x
                      << ", y=" << y
                      << ", yaw=" << yaw
                      << ", ref_vel=" << ref_vel
                      << ", time(s)" << time 
                      << ", is_pushing=" << is_pushing << ")";
        }

    };

    class trajectory
    {
    public:
        float time_zero=0;
        std::string header_delim = "!!!";
        std::string elem_delim = ";@;";
        std::string var_delim = ",,,";
        std::string header = "t"; // header for trajectory

        std::shared_ptr<std::vector<trajectory_elem>> trajectory_points;

        trajectory()
        {
            std::vector<trajectory_elem> traj(0);
            trajectory_points = std::make_shared<std::vector<trajectory_elem>>(traj);
        }

        trajectory(std::string& serialized_trajectory_str)
        {
            std::vector<trajectory_elem> traj(0);
            trajectory_points = std::make_shared<std::vector<trajectory_elem>>(traj);
            deserialize(serialized_trajectory_str);
        }


        void append_waypoint(trajectory_elem wpt)
        {
            trajectory_points->push_back(wpt);
        }

        std::string serialize()
        {
            // header!time_zero;x,y,yaw,vel,time,is_pushing;...;
            std::string out_str = header + header_delim + float2binarystr(time_zero) + elem_delim; // init with time_zero

            size_t traj_size = trajectory_points->size();
            for(size_t n=0; n<traj_size; n++)
            {
                std::string temp_str=""; // string for one waypoint
                temp_str += float2binarystr(trajectory_points->at(n).x);
                temp_str += var_delim;
                temp_str += float2binarystr(trajectory_points->at(n).y);
                temp_str += var_delim;
                temp_str += float2binarystr(trajectory_points->at(n).yaw);
                temp_str += var_delim;
                temp_str += float2binarystr(trajectory_points->at(n).ref_vel);
                temp_str += var_delim;
                temp_str += float2binarystr(trajectory_points->at(n).time);
                temp_str += var_delim;
                temp_str += bool2binarystr(trajectory_points->at(n).is_pushing);


                // append to the output string
                out_str += temp_str;
                if(n!=traj_size-1)
                    out_str += elem_delim;
            }

            return out_str;
        }

        void deserialize(std::string& str_traj)
        {
            // split header
            auto header_sp = split(str_traj,header_delim);
            if(header_sp[0] == "t") // trajectory
            {
                // split elements
                auto elem_sp = split(header_sp[1],elem_delim);
                // first elem is time_zero
                auto time_zero_str = elem_sp[0];
                time_zero = binarystr2float(time_zero_str);

                // parse trajectory
                size_t elem_size = elem_sp.size();
                for(size_t n=1; n<elem_size; n++)
                {
                    // split variables
                    auto var_sp = split(elem_sp[n],var_delim); // x y yaw vel time
                    float x_in = binarystr2float(var_sp[0]);
                    float y_in = binarystr2float(var_sp[1]);
                    float yaw_in = binarystr2float(var_sp[2]);
                    float vel_in = binarystr2float(var_sp[3]);
                    float time_in = binarystr2float(var_sp[4]);
                    bool is_pushing_in = binarystr2bool(var_sp[5]);

                    trajectory_elem temp_elem(x_in,y_in,yaw_in,vel_in,time_in,is_pushing_in);
                    append_waypoint(temp_elem);
                }
            }
            
            else
            {
                //unknown header
            }
        }

        // Print function for trajectory
        void print() const {
            std::cout << "Trajectory:" << std::endl;
            std::cout << "Time Zero: " << time_zero << std::endl;
            std::cout << "Waypoints:" << std::endl;
            for (size_t i = 0; i < trajectory_points->size(); ++i)
            {
                std::cout << "Waypoint " << i << ": ";
                trajectory_points->at(i).print();
                std::cout << std::endl;
            }
        }
    };
}

visualization_msgs::MarkerArray draw_obstacles(std::vector<ReloPush::trajectory_elem>& mo_list, ros::Publisher* pub_ptr, float size = 0.15f)
{
    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < mo_list.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "obj" + std::to_string(i);
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = i;
        marker.pose.position.x = mo_list[i].x;
        marker.pose.position.y = mo_list[i].y;
        marker.pose.position.z = size/2; // Assuming the cube is placed at z = 0.5

        double half_yaw = mo_list[i].yaw / 2;



        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = std::sin(half_yaw);
        marker.pose.orientation.w = std::cos(half_yaw);



        marker.scale.x = size; // Set the scale of the cube as desired
        marker.scale.y = size;
        marker.scale.z = size;
        //marker.color.r = 0.8274509803921568;
        //marker.color.g = 0.7254901960784313;
        //marker.color.b = 0.6235294117647059;

        //marker.color.r = 0.5764705882352941;
        //marker.color.g = 0.7647058823529411;
        //marker.color.b = 0.8627450980392157;

        // for figure
        //marker.color.r = 0.23137254901960785;
        //marker.color.g = 0.23921568627450981;
        //marker.color.b = 0.32941176470588235;

        // for video
        marker.color.r = 1;
        marker.color.g = 0.8196078;
        marker.color.b = 0.4;

        marker.color.a = 1.0;
        
        marker_array.markers.push_back(marker);

        std::cout << marker.ns << ": " << marker.pose.position.x << ", " << marker.pose.position.y << ", " << mo_list[i].yaw << std::endl;
    }
    pub_ptr->publish(marker_array);

    return marker_array;
}

visualization_msgs::MarkerArray draw_deliveries(std::vector<ReloPush::trajectory_elem>& d_list, ros::Publisher* pub_ptr, float size = 0.17f)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 0;

    for (size_t i = 0; i < d_list.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map"; // Change "map" to your desired frame
        marker.header.stamp = ros::Time::now();
        marker.ns = "obj" + std::to_string(i);
        marker.id = id++;
        marker.type = visualization_msgs::Marker::CUBE; // Change marker type as needed
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = d_list[i].x;
        marker.pose.position.y = d_list[i].y;
        marker.pose.position.z = size/2; // Adjust as needed
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(d_list[i].yaw); // Assuming th is in radians
        marker.scale.x = size; // Adjust size as needed
        marker.scale.y = size;
        marker.scale.z = size;
        //marker.color.r = 0.439; //0.439, 0.545, 0.459
        //marker.color.g = 0.545;
        //marker.color.b = 0.459;

        // for figure
        //marker.color.r = 0.2196078431372549; //0.439, 0.545, 0.459
        //marker.color.g = 0.40784313725490196;
        //marker.color.b = 0.4196078431372549;

        // for video
        marker.color.r = 0.9372549019607843; //0.439, 0.545, 0.459
        marker.color.g = 0.2784313725490196;
        marker.color.b = 0.43529411764705883;


        // for video 2
        marker.color.r = 0.9961; //0.439, 0.545, 0.459
        marker.color.g = 0.3725;
        marker.color.b = 0.3333;

        marker.color.a = 0.5; // Fully opaque
        marker_array.markers.push_back(marker);
    }
    pub_ptr->publish(marker_array);

    return marker_array;
}

visualization_msgs::Marker visualize_workspace_boundary(float max_x, float max_y, ros::Publisher* pubPtr)
{
    visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "rectangle";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;

        // Define the rectangle points (assuming it's axis-aligned for simplicity)
        geometry_msgs::Point p1, p2, p3, p4;
        p1.x = -0.1; p1.y = -0.1; p1.z = 0.0;
        p2.x = max_x+0.1; p2.y = -0.1; p2.z = 0.0;
        p3.x = max_x+0.1; p3.y = max_y; p3.z = 0.0;
        p4.x = -0.1; p4.y = max_y; p4.z = 0.0;

        // Add the points to the marker
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p3);
        marker.points.push_back(p4);
        marker.points.push_back(p1);  // Close the rectangle by returning to the first point

        // Define marker properties
        // for figure
        //marker.scale.x = 0.02;  // Line width
        //marker.color.r = 0.15294117647058825;
        //marker.color.g = 0.14901960784313725;
        //marker.color.b = 0.16862745098039217;
        //marker.color.a = 0.33;

        //for video
        marker.scale.x = 0.02;  // Line width
        marker.color.r = 1;
        marker.color.g = 0.9882352941176471;
        marker.color.b = 0.9764705882352941;
        marker.color.a = 0.33;

        // Publish the marker
        pubPtr->publish(marker);
        return marker;
}

float convertEulerRange_to_pi(float yaw) {
    if (yaw > M_PI) {
        yaw -= 2 * M_PI;
    }
    return yaw;
}

template<typename T>
Eigen::Quaternion<T> euler_to_quaternion_zyx(Eigen::Matrix<T, 3, 1>& euler_in)
{
    return Eigen::AngleAxis<T>(euler_in.z(), Eigen::Matrix<T, 3, 1>::UnitZ())
        * Eigen::AngleAxis<T>(euler_in.y(), Eigen::Matrix<T, 3, 1>::UnitY())
        * Eigen::AngleAxis<T>(euler_in.x(), Eigen::Matrix<T, 3, 1>::UnitX());
}

// quaternion to rotation matrix
template<typename T>
Eigen::Matrix<T, 3, 3> quaternion_to_rotation_matrix(Eigen::Quaternion<T>& q_in) {
    return q_in.normalized().toRotationMatrix();
}

template<typename T>
Eigen::Matrix<T,4,4> homo_matrix_from_R_t(Eigen::Matrix<T,3,3> R, Eigen::Matrix<T,3,1> t)
{
    Eigen::Matrix<T,4,4> H = Eigen::Matrix<T,4,4>::Identity();
    H.block(0,0,3,3) = R;
    H.block(0,3,3,1) = t;
    return H;
}

template<typename T>
std::pair<Eigen::Matrix<T,3,3>, Eigen::Matrix<T,3,1>> R_t_from_homo_matrix(Eigen::Matrix<T,4,4> H)
{
    Eigen::Matrix<T,3,3> R;
    Eigen::Matrix<T,3,1> t; 
    R = H.block(0,0,3,3);
    t = H.block(0,3,3,1);
    return std::make_pair(R,t);
}

// TF matrix to Eigen matrix
template<typename T>
void matrixTFToEigen(const tf::Matrix3x3& tf_matrix, Eigen::Matrix<T,3,3>& eigen_matrix) {
    // Populate the Eigen matrix with values from the tf::Matrix3x3
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            eigen_matrix(i, j) = static_cast<T>(tf_matrix[i][j]);
        }
    }
}

/*
* out: roll, pitch, yaw
*/
template<typename T>
Eigen::Matrix<T,3,1> rot_matrix_to_euler_ZYX(Eigen::Matrix<T,3,3> rmat)
{
    struct Euler
    {
        double yaw;
        double pitch;
        double roll;
    };
    const double pi = 3.14159265;
    Euler euler_out;
    //check singularity
    if(fabs(rmat(2,0) >= 1))
    {
        euler_out.yaw = 0;
        // From difference of angles formula
        if (rmat(2,0) < 0)  //gimbal locked down
        {
        double delta = atan2(rmat(0,1),rmat(0,2));
            euler_out.pitch = pi / double(2.0);
            euler_out.roll = delta;
        }
        else // gimbal locked up
        {
        double delta = atan2(-rmat(0,1),-rmat(0,2));
            euler_out.pitch = -1 * pi / double(2.0);
            euler_out.roll = delta;
        }
    }

    else
    {
        euler_out.pitch = -1 * asin(rmat(2,0));

        euler_out.roll = atan2(rmat(2,1)/cos(euler_out.pitch),
            rmat(2,2)/cos(euler_out.pitch));

        euler_out.yaw = atan2(rmat(1,0)/cos(euler_out.pitch),
            rmat(0,0)/cos(euler_out.pitch));
    }

    return Eigen::Matrix<T,3,1>(euler_out.roll, euler_out.pitch, euler_out.yaw);
}

// ROS tf transform to Tranformation Matrix
template<typename T>
std::pair<Eigen::Matrix<T,3,3>, Eigen::Matrix<T,3,1>> tf_to_Rt(tf::StampedTransform trans)
{
    tf::Quaternion trans_ori = trans.getRotation();
    tf::Vector3 trans_trans = trans.getOrigin();

    tf::Matrix3x3 rot_m(trans_ori);
    Eigen::Matrix<T,3,3> rot_m_eigen;
    //tf::matrixTFToEigen(rot_m, rot_m_eigen);
    matrixTFToEigen<float>(rot_m, rot_m_eigen);

    Eigen::Matrix<T,3,1> trans_v(trans_trans.getX(), trans_trans.getY(), trans_trans.getZ());
    //tf::vectorTFToEigen(trans_trans, trans_v);


    return std::make_pair(rot_m_eigen, trans_v);
}

tf::StampedTransform listen_tf(std::string from_tf, std::string to_tf)
{
  tf::StampedTransform transform;
  tf::TransformListener listener;
  try{
      listener.waitForTransform(from_tf, to_tf,
                                    ros::Time::now(), ros::Duration(1.0));
    listener.lookupTransform(from_tf, to_tf,
                             ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    //ros::Duration(0.1).sleep();
    //continue;Quaternion XYZW: -0.0396948 0.953661 -0.258644 -0.148522
    transform.setData(tf::Transform::getIdentity());
  }
  return transform;
}

std::pair<Eigen::Vector3f,Eigen::Vector3f> get_real_robotPose(ros::NodeHandle& nh)
{
        // todo: parse frames as params
        std::string from_tf = "map_mocap";
        std::string to_tf = "map";

        // get map tf
        // todo: move it to the beginning of the program to do it only once
        auto transform = listen_tf(from_tf, to_tf);
        auto robotPose_mocap = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/natnet_ros/mushr2/pose", nh);
        std::cout << "Real Robot Pose: " << robotPose_mocap->pose.position.x << ", " << robotPose_mocap->pose.position.y << std::endl;

        if (robotPose_mocap != nullptr) {
            // Robot pose on mocap frame
            Eigen::Vector3f robotPos_mocap(robotPose_mocap->pose.position.x, 
                                            robotPose_mocap->pose.position.y, 
                                            robotPose_mocap->pose.position.z);
            
            Eigen::Quaternionf robotQ_mocap(robotPose_mocap->pose.orientation.w,robotPose_mocap->pose.orientation.x,
                                            robotPose_mocap->pose.orientation.y,robotPose_mocap->pose.orientation.z);

            // to Rotation Matrix
            auto robotR_mocap = quaternion_to_rotation_matrix<float>(robotQ_mocap);
            // robot transformation matrix
            auto robotH_mocap = homo_matrix_from_R_t<float>(robotR_mocap, robotPos_mocap);

            // TF world to mocap
            Eigen::Vector3f t_w2mocap;
            Eigen::Matrix3f R_w2mocap;
            std::tie(R_w2mocap, t_w2mocap) = tf_to_Rt<float>(transform);
            // frame transformation matrix
            auto mocapH = homo_matrix_from_R_t(R_w2mocap, t_w2mocap);

            // transform
            auto robotH_w = mocapH.inverse() * robotH_mocap;
            Eigen::Vector3f t_w;
            Eigen::Matrix3f R_w;
            std::tie(R_w,t_w) = R_t_from_homo_matrix<float>(robotH_w);

            // get yaw
            auto euler_zyx = rot_matrix_to_euler_ZYX<float>(R_w);

            return std::make_pair(euler_zyx, t_w);
        }

        else {
            ROS_WARN("No message received within the timeout period.");
            // todo: handle exception
        }
}


int main(int argc, char** argv) {
    // Create a ZeroMQ context with one I/O thread.
    zmq::context_t context(1);
    
    // Create a REP (reply) socket.
    zmq::socket_t socket(context, zmq::socket_type::rep);
    
    // Bind the socket to TCP port 5555.
    try {
        socket.bind("tcp://*:5555");
        std::cout << "Server successfully bound to tcp://*:5555" << std::endl;
    } catch (const zmq::error_t& e) {
        std::cerr << "Binding failed: " << e.what() << std::endl;
        return -1;
    }

    
    std::cout << "Server is listening on port 5555..." << std::endl;

    ros::init(argc, argv, "string_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/mushr2/relopush/serialized_trajectory", 2);
    ros::Publisher robot_pose_reset = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
    robot_pose_reset_ptr = new ros::Publisher(robot_pose_reset);
    ros::Publisher object_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("movable_objects", 10);
    object_marker_pub_ptr = new ros::Publisher(object_marker_pub);
    ros::Publisher goal_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("goals", 10);
    goal_marker_pub_ptr = new ros::Publisher(goal_marker_pub);
    ros::Publisher boundary_pub = nh.advertise<visualization_msgs::Marker>("/workspace_boundary", 10);
    boundary_pub_ptr = new ros::Publisher(boundary_pub);
    

    // Run forever.
    while (true) {
        zmq::message_t request;
        
        // Wait for the next request from a client.
        socket.recv(request, zmq::recv_flags::none);
        
        // Convert the received message to a std::string.
        std::string received_msg(static_cast<char*>(request.data()), request.size());
        

        auto decode64 = base64_decode(received_msg,false);
        std::cout << "Received message: " << decode64 << std::endl;

        //ReloPush::trajectory traj_in(decode64);

        //traj_in.print();
        
        // Prepare a reply message.
        std::string reply_str = "Message received";

        ros::Rate loop_rate(100); // 10 Hz


        // decode base64
        

        //while (ros::ok())
        //{

            // header 't'
            if(decode64[0] == 't')
            {
                // encode for compatibility
                //std::string encoded_data = base64_encode(reinterpret_cast<const unsigned char*>(decode64.c_str()), decode64.length());
                std_msgs::String msg;
                msg.data = received_msg; // Assign your std::string to the data field

                // Publish the message
                pub.publish(msg);
            }
            else if(decode64[0] == 'r') // init robot pose for sim
            {
                auto header_sp = ReloPush::split(decode64,"!!!");
                ReloPush::trajectory_elem pose(header_sp[1]);
                std::cout << "Init Robot Pose for Sim: (" << pose.x << ", " << pose.y << ", " << pose.yaw << ")" << std::endl;
                // reset robot initpose on sim
                geometry_msgs::PoseWithCovarianceStamped init_pose;
                init_pose.pose.pose.position.x = pose.x;
                init_pose.pose.pose.position.y = pose.y;

                // angle in -pi ~ pi range
                float yaw = pose.yaw;
                yaw = convertEulerRange_to_pi(yaw);
                //to quaternion
                auto yaw_euler = Eigen::Vector3f(0,0,yaw);
                auto yaw_quat = euler_to_quaternion_zyx(yaw_euler);
                init_pose.pose.pose.orientation.w = yaw_quat.w();
                init_pose.pose.pose.orientation.x = yaw_quat.x();
                init_pose.pose.pose.orientation.y = yaw_quat.y();
                init_pose.pose.pose.orientation.z = yaw_quat.z();

                robot_pose_reset_ptr->publish(init_pose);
            }
            else if(decode64[0] == 'l') // real robot pose
            {
                auto r = get_real_robotPose(nh);
                ReloPush::trajectory_elem robot(r.second.x(),r.second.y(),r.first.z(),0,0,false);
                auto r_str = robot.serialize();
                auto r_enc = base64_encode(reinterpret_cast<const unsigned char*>(r_str.c_str()), r_str.length());

                reply_str = r_enc;
            }
            else if(decode64[0] == 'o' || decode64[0] == 'g')
            {
                //deserialize object info
                auto header_sp = ReloPush::split(decode64,"!!!");
                auto elem_sp = ReloPush::split(header_sp[1],";$;");
                std::vector<ReloPush::trajectory_elem> objs;
                for(auto& it : elem_sp)
                    objs.push_back(ReloPush::trajectory_elem(it));

                if(decode64[0] == 'o')
                    draw_obstacles(objs,object_marker_pub_ptr);
                else if(decode64[0] == 'g')
                    draw_deliveries(objs,goal_marker_pub_ptr);

                visualize_workspace_boundary(4,5.2,boundary_pub_ptr);
            }

            zmq::message_t reply(reply_str.size());
            memcpy(reply.data(), reply_str.c_str(), reply_str.size());
            
            // Send the reply back to the client.
            socket.send(reply, zmq::send_flags::none);
    
            ros::spinOnce();
            loop_rate.sleep();
        //}
    }

    return 0;
}