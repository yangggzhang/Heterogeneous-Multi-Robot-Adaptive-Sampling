#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_localization/navsat_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <cstdlib>
#include <queue>

	// rostopic pub /Jackal/gps_now std_msgs/Bool '{ data: 1}'
    //  rostopic pub Jackal/waypoint std_msgs/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [64, 64]}"


static int DEFAULT_RATE = 10;
static size_t DEFAULT_BUFFER = 50;

class myQueue{
private: 

    std::queue<double> x;
    std::queue<double> y;
    size_t size;
    double sum_x;
    double sum_y;
public:
    myQueue() {}

    void initialize(size_t size_in){
        size = size_in;
        sum_x = 0.0;
        sum_y = 0.0;
    }

    void push(double &loc_x, double &loc_y)
    {
        if (x.size() >= size)
        {
            sum_x -= x.front();
            sum_y -= y.front();
            x.pop();
            y.pop();
        }
        x.push(loc_x);
        y.push(loc_y);
        sum_x += loc_x;
        sum_y += loc_y;
    }

    void position(double &loc_x, double &loc_y)
    {
        double n = (double) x.size();
        assert(n > 0);
        loc_x = sum_x/n;
        loc_y = sum_y/n;
    }

};

class Jackal_GPS_waypoint{
private:
    std::string utm_zone;
    const size_t AVER_NUM = 10; 
    myQueue loc_q;
    ros::NodeHandle n;
    ros::Publisher goal_pub;
    ros::Subscriber send_goal;
    ros::Subscriber gps_sub;
    ros::Subscriber measure_sub;

public:
    Jackal_GPS_waypoint()
    {
        loc_q.initialize(DEFAULT_BUFFER);
        goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
        gps_sub = n.subscribe("gps/filtered", 10, &Jackal_GPS_waypoint::gpsCallback, this);
        send_goal = n.subscribe("Jackal/waypoint", 1, &Jackal_GPS_waypoint::goalCallback, this);
        measure_sub = n.subscribe("Jackal/gps_now", 1, &Jackal_GPS_waypoint::checkGPS, this);
    }


    geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped UTM_input){
        geometry_msgs::PointStamped map_point_output;
        bool notDone = true;
        tf::TransformListener listener; //create transformlistener object called listener
        ros::Time time_now = ros::Time::now();
        geometry_msgs::PointStamped UTM_point;
        while(notDone)
        {
            try
            {
                UTM_point.header.stamp = ros::Time::now();
                listener.waitForTransform("odom", "utm", time_now, ros::Duration(3.0));
                listener.transformPoint("odom", UTM_input, map_point_output);
                notDone = false;
            }
            catch (tf::TransformException& ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(0.01).sleep();
                //return;
            }
        }
        return map_point_output;
    }

    geometry_msgs::PointStamped latLongtoUTM(double lati_input, double longi_input)
    {
        double utm_x = 0, utm_y = 0;
        geometry_msgs::PointStamped UTM_point_output;

        //convert lat/long to utm
        RobotLocalization::NavsatConversions::LLtoUTM(lati_input, longi_input, utm_y, utm_x, utm_zone);

        //Construct UTM_point and map_point geometry messages
        UTM_point_output.header.frame_id = "utm";
        UTM_point_output.header.stamp = ros::Time(0);
        UTM_point_output.point.x = utm_x;
        UTM_point_output.point.y = utm_y;
        UTM_point_output.point.z = 0;

        return UTM_point_output;
    }

    geometry_msgs::PoseStamped calculate_goal(double goal_lat, double goal_lng){
        geometry_msgs::PoseStamped goal;

        double cur_lat, cur_lng; // utm_position
        loc_q.position(cur_lat, cur_lng);

        geometry_msgs::PointStamped cur_UTM_point = latLongtoUTM(cur_lat, cur_lng);
        geometry_msgs::PointStamped cur_map_point = UTMtoMapPoint(cur_UTM_point);

        geometry_msgs::PointStamped goal_UTM_point = latLongtoUTM(goal_lat, goal_lng);
        geometry_msgs::PointStamped goal_map_point = UTMtoMapPoint(goal_UTM_point);

        tf::Matrix3x3 rot_euler;
        tf::Quaternion rot_quat;

        // Calculate quaternion
        float x_curr = cur_map_point.point.x, y_curr = cur_map_point.point.y; // set current coords.
        float x_next = goal_map_point.point.x, y_next = goal_map_point.point.y; // set coords. of next waypoint
        float delta_x = x_next - x_curr, delta_y = y_next - y_curr;   // change in coords.
        float yaw_curr = 0, pitch_curr = 0, roll_curr = 0;
        yaw_curr = atan2(delta_y, delta_x);

        // Specify quaternions
        rot_euler.setEulerYPR(yaw_curr, pitch_curr, roll_curr);
        rot_euler.getRotation(rot_quat);

        goal.header.stamp =  ros::Time::now();
        goal.header.frame_id= "odom";
        goal.pose.position.x = goal_map_point.point.x;
        goal.pose.position.y = goal_map_point.point.y;
        goal.pose.orientation.x = rot_quat.getX();
        goal.pose.orientation.y = rot_quat.getY();
        goal.pose.orientation.z = rot_quat.getZ();
        goal.pose.orientation.w = rot_quat.getW();

        return goal;
    }

    void gpsCallback(const sensor_msgs::NavSatFix& msg){
        double lat = msg.latitude;
        double lng = msg.longitude;
        loc_q.push(lat, lng);
    }

    void goalCallback(const std_msgs::Float64MultiArray& msg)
    {
        ROS_INFO("Received Goal:%.8f, %.8f", msg.data[0], msg.data[1]); 
        geometry_msgs::PoseStamped goal = calculate_goal(msg.data[0], msg.data[1]);
        ROS_INFO("Goal point:%.8f, %.8f", goal.pose.position.x, goal.pose.position.y);
        goal_pub.publish(goal);
    }

    void checkGPS(const std_msgs::Bool& msg)
    {
        if (msg.data)
        {
            double curr_lat, curr_lng;
            loc_q.position(curr_lat, curr_lng);
            ROS_INFO("Jackal Position:%.8f, %.8f", curr_lat, curr_lng);
        }
        
    }


    void start(){
        ROS_INFO("Jackal starts GPS Waypoint Navigation!");
        ros::Rate r(DEFAULT_RATE);
        while(ros::ok())
        {
            ros::spinOnce();
            r.sleep();
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_waypoint"); //initiate node called gps_waypoint
    Jackal_GPS_waypoint jackal_node;
    jackal_node.start();
    ros::spin();
    // ros::NodeHandle n;
    // ROS_INFO("Initiated gps_waypoint node");
    // ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    // ros::Subscriber gps_sub = n.subscribe("fix", 10, gpsCallback);
    // MoveBaseClient ac("/move_base", true);
    //construct an action client that we use to communication with the action named move_base.
    //Setting true is telling the constructor to start ros::spin()

    // Initiate publisher to send end of node message
    // ros::Publisher pubWaypointNodeEnded = n.advertise<std_msgs::Bool>("/outdoor_waypoint_nav/waypoint_following_status", 100);
    
    //wait for the action server to come up
    // while(!ac.waitForServer(ros::Duration(5.0)))
    // {
    //     wait_count++;
    //     if(wait_count > 3)
    //     {
    //         ROS_ERROR("move_base action server did not come up, killing gps_waypoint node...");
    //         // Notify joy_launch_control that waypoint following is complete
    //         std_msgs::Bool node_ended;
    //         node_ended.data = true;
    //         pubWaypointNodeEnded.publish(node_ended);
    //         ros::shutdown();
    //     }
    //     ROS_INFO("Waiting for the move_base action server to come up");
    // }

    //Get Longitude and Latitude goals from text file

    //Count number of waypoints
    // numWaypoints = countWaypointsInFile(path_local);

    //Reading waypoints from text file and output results
    // waypointVect = getWaypoints(path_local);


    // ROS_INFO("Received Latitude goal!!!:%.8f", latiGoal);
    // ROS_INFO("Received longitude goal!!!:%.8f", longiGoal);

    // UTM_point = latLongtoUTM(latiGoal, longiGoal);
    //     // UTM_next = latLongtoUTM(latiNext, longiNext);

    //     //Transform UTM to map point in odom frame
    // map_point = UTMtoMapPoint(UTM_point);
    //     // map_next = UTMtoMapPoint(UTM_next);


    // ROS_INFO("Map point:%.8f, %.8f", map_point.point.x, map_point.point.y);

    // geometry_msgs::PoseStamped simple_goal;//
    // simple_goal.header.stamp =  ros::Time::now();
    // simple_goal.header.frame_id= "odom";
    // simple_goal.pose.position.x = map_point.point.x;
    // simple_goal.pose.position.y = map_point.point.y;
    // simple_goal.pose.orientation.w = 1;
    // // Send Goal
    // ROS_INFO("Sending goal");
    // goal_pub.publish(simple_goal);
    // waypointVect.push_back(std::make_pair(lati, longi));

    // Iterate through vector of waypoints for setting goals
    // for(iter = waypointVect.begin(); iter < waypointVect.end(); iter++)
    // {
    //     //Setting goal:
    //     latiGoal = iter->first;
    //     longiGoal = iter->second;
    //     bool final_point = false;

    //     //set next goal point if not at last waypoint
    //     if(iter < (waypointVect.end() - 1))
    //     {
    //         iter++;
    //         latiNext = iter->first;
    //         longiNext = iter->second;
    //         iter--;
    //     }    
    //     else //set to current
    //     {
    //         latiNext = iter->first;
    //         longiNext = iter->second;
    //         final_point = true;
    //     }    

    //     ROS_INFO("Received Latitude goal!!!:%.8f", latiGoal);
    //     ROS_INFO("Received longitude goal!!!:%.8f", longiGoal);

    //     //Convert lat/long to utm:
    //     UTM_point = latLongtoUTM(latiGoal, longiGoal);
    //     // UTM_next = latLongtoUTM(latiNext, longiNext);

    //     //Transform UTM to map point in odom frame
    //     map_point = UTMtoMapPoint(UTM_point);
    //     // map_next = UTMtoMapPoint(UTM_next);


    //     ROS_INFO("Map point:%.8f, %.8f", map_point.point.x, map_point.point.y);
    //     // ROS_INFO("Map next:%.8f, %.8f", map_next.point.x, map_next.point.y);
    //     // std::cout<<"Map point"<<map_point.point.x<<" "<<map_point.point.y<<std::endl;
    //     // std::cout<<"Map next"<<map_next.point.x<<" "<<map_next.point.y<<std::endl;

    //     //Build goal to send to move_base
    //     // move_base_msgs::MoveBaseGoal goal = buildGoal(map_point, map_next, final_point); //initiate a move_base_msg called goal
    //     geometry_msgs::PoseStamped simple_goal;//
    //     simple_goal.pose.position.x = map_point.point.x;
    //     simple_goal.pose.position.y = map_point.point.y;
    //     // Send Goal
    //     ROS_INFO("Sending goal");
    //     goal_pub.publish(simple_goal);
    //     // ac.sendGoal(simple_goal); //push goal to move_base node

    //     //Wait for result
    //     ac.waitForResult(); //waiting to see if move_base was able to reach goal

    //     if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //     {
    //         ROS_INFO("jackal has reached its goal!");
    //         //switch to next waypoint and repeat
    //     }
    //     else
    //     {
    //         ROS_ERROR("jackal was unable to reach its goal. GPS Waypoint unreachable.");
    //         ROS_INFO("Exiting node...");
    //         // Notify joy_launch_control that waypoint following is complete
    //         std_msgs::Bool node_ended;
    //         node_ended.data = true;
    //         pubWaypointNodeEnded.publish(node_ended);
    //         ros::shutdown();
    //     }
    // } // End for loop iterating through waypoint vector

    // ROS_INFO("jackal has reached all of its goals!!!\n");
    // ROS_INFO("Ending node...");

    // Notify joy_launch_control that waypoint following is complete
    // std_msgs::Bool node_ended;
    // node_ended.data = true;
    // pubWaypointNodeEnded.publish(node_ended);

    // ros::shutdown();
    // ros::spin();
    return 0;
}
