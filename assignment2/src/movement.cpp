#include "movement.h"
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include "sensor_msgs/LaserScan.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

// Constructor: Trasverse corrideor, detect tables and take initiail position
Movement::Movement(ros::NodeHandle& nh)
    : POS_X_ORIENTATION([] {
          geometry_msgs::Quaternion q;
          q.x = 0.0; q.y = 0.0; q.z = 0.0; q.w = 1.0;  // 0° (Positive X)
          return q;
      }()),
      POS_Y_ORIENTATION([] {
          geometry_msgs::Quaternion q;
          q.x = 0.0; q.y = 0.0; q.z = 0.707; q.w = 0.707;  // 90° (Positive Y)
          return q;
      }()),
      NEG_X_ORIENTATION([] {
          geometry_msgs::Quaternion q;
          q.x = 0.0; q.y = 0.0; q.z = 1.0; q.w = 0.0;  // 180° (Negative X)
          return q;
      }()),
      NEG_Y_ORIENTATION([] {
          geometry_msgs::Quaternion q;
          q.x = 0.0; q.y = 0.0; q.z = -0.707; q.w = 0.707;  // -90° (Negative Y)
          return q;
      }()),
      ac("move_base", true) {
    // Wait for MoveBase server
    ac.waitForServer();

    // Traverse the corridor
    ROS_INFO("Robot traverses the corridor.");
    sendGoalToMoveBase(MAX_CORRIDOR_X, 0.0, POS_X_ORIENTATION);

    // Perform first (approximate) tables detection
    ROS_INFO("Robot starts the first tables detection (raw).");
    ros::Duration(2.0).sleep();
    const sensor_msgs::LaserScan::ConstPtr scan_msg_1 = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh);
    detect_tables(scan_msg_1);

    // Move to second detection point
    ROS_INFO("Robot goes to second detection point.");
    spin(-M_PI_2);
    sendGoalToMoveBase(MAX_CORRIDOR_X, (TABLE_1_Y+TABLE_2_Y)/2, NEG_Y_ORIENTATION);
    spin(M_PI_2);

    // Perform second (precise) tables detection
    ROS_INFO("Robot starts the second tables detection (refined).");
    ros::Duration(2.0).sleep();
    const sensor_msgs::LaserScan::ConstPtr scan_msg_2 = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh);
    detect_tables(scan_msg_2);

    // Initialize offsets for docks and corners position
    std::vector<std::pair<float, float>> docks_offsets_1 = {
        {-DIST_1, 0}, {0, -DIST_1}, {DIST_1, 0}};
    std::vector<std::pair<float, float>> docks_offsets_2 = {
        {-DIST_2, 0}, {0, -DIST_2}, {DIST_2, 0}};
    std::vector<std::pair<float, float>> corns_offsets_1 = {
        {-DIST_1, DIST_1}, {DIST_1, DIST_1}};
    std::vector<std::pair<float, float>> corns_offsets_2 = {
        {-DIST_2, DIST_2}, {DIST_2, DIST_2}};

    // Initialize docks and corners positions
    for (auto off : docks_offsets_1)
        docks.emplace_back(TABLE_1_X + off.first, TABLE_1_Y - off.second);
    for (auto off : docks_offsets_2)
        docks.emplace_back(TABLE_2_X - off.first, TABLE_2_Y + off.second);
    for (auto off : corns_offsets_1)
        corns.emplace_back(TABLE_1_X + off.first, TABLE_1_Y + off.second);
    for (auto off : corns_offsets_2)
        corns.emplace_back(TABLE_2_X - off.first, TABLE_2_Y - off.second);

    // Move to initial position (dock 1)
    ROS_INFO("Moving to the initial position.");
    spin(M_PI_2);
    sendGoalToMoveBase(docks[0].first, docks[0].second, POS_Y_ORIENTATION);
    spin(-M_PI_2);
    cur_pos = 1;
}

// Tables detection function
void Movement::detect_tables(const sensor_msgs::LaserScan::ConstPtr& msg) {
    const float THR1 = 0.7;
    const float THR2 = 0.3;

    std::vector<std::vector<std::pair<float,float>>> points_proc;
    std::vector<std::pair<float,float>> tmp;
    points_proc.push_back(tmp);

    // Load data from laser_scan message
    int curr=0;
    float r = msg->ranges[0];
    float t = 0*msg->angle_increment;
    float x_prev = r*std::cos(t);
    float y_prev = r*std::sin(t);
    points_proc[curr].push_back(std::make_pair(x_prev, y_prev));

    // Split points in subvectors of contiguous ones
    for(int i=1; i<msg->ranges.size(); i++) {
        float r = msg->ranges[i];
        float t = i*msg->angle_increment + msg->angle_min;
        float x = r*std::cos(t);
        float y = r*std::sin(t);

        float dist = std::pow(x-x_prev, 2) + std::pow(y-y_prev, 2);
        if(dist > std::pow(THR1, 2)) {
            std::vector<std::pair<float,float>> tmp;
            points_proc.push_back(tmp);
            curr++;
        }
        points_proc[curr].push_back(std::make_pair(x, y));
        x_prev = x;
        y_prev = y;
    }

    // Save subvector indexes of candidate table points (distance between first and last close to table pole width)
    std::vector<int> table_idxs;
    for(int i=0; i<points_proc.size(); i++) {
        float x1 = points_proc[i][0].first;
        float x2 = points_proc[i][points_proc[i].size()-1].first;
        float y1 = points_proc[i][0].second;
        float y2 = points_proc[i][points_proc[i].size()-1].second;
        float dist = std::pow(x1-x2, 2) + std::pow(y1-y2, 2);
        if(dist < std::pow(THR2, 2)) 
            table_idxs.push_back(i);
    }

    // Obtain transform from base_laser_link to map
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    while(!tf_buffer.canTransform("map", "base_laser_link", ros::Time(0)))
        ros::Duration(0.001).sleep();
    geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform("map", "base_laser_link", ros::Time(0), ros::Duration(1.0));

    // For each subvector index compute the relative table coordinates
    std::vector<std::pair<float,float>> tables_coord;
    for(int i: table_idxs) {
        // Obtain x, y coord as mean between first and last point
        float x1 = points_proc[i][0].first;
        float x2 = points_proc[i][points_proc[i].size()-1].first;
        float y1 = points_proc[i][0].second;
        float y2 = points_proc[i][points_proc[i].size()-1].second;
        float x = (x1 + x2) / 2;
        float y = (y1 + y2) / 2;
        // Transform into map reference frame
        geometry_msgs::PoseStamped base_link_point;
        base_link_point.header.frame_id = "base_laser_link";
        base_link_point.header.stamp = ros::Time::now();
        base_link_point.pose.position.x = x;  
        base_link_point.pose.position.y = y;  
        base_link_point.pose.position.z = 0.0;
        base_link_point.pose.orientation.x = 0.0;
        base_link_point.pose.orientation.y = 0.0;
        base_link_point.pose.orientation.z = 0.0;
        base_link_point.pose.orientation.w = 1.0;
        geometry_msgs::PoseStamped map_point;
        tf2::doTransform(base_link_point, map_point, transform);

        tables_coord.push_back(std::make_pair(map_point.pose.position.x, map_point.pose.position.y));
        // ROS_INFO("Table found at x=%f y=%f", map_point.pose.position.x, map_point.pose.position.y);
    }

    // Erase first and last results always produced from lateral noise
    tables_coord.erase(tables_coord.begin());
    tables_coord.erase(tables_coord.end()-1);

    // Assign table 1 to closest detection, table 2 to the remaining one
    std::pair<float, float> table_1, table_2;
    float dist1 = std::pow(tables_coord[0].first, 2) + std::pow(tables_coord[0].second, 2);
    float dist2 = std::pow(tables_coord[1].first, 2) + std::pow(tables_coord[1].second, 2);
    if(dist1 < dist2) {
        table_1 = tables_coord[0];
        table_2 = tables_coord[1];
    }
    else {
        table_1 = tables_coord[1];
        table_2 = tables_coord[0];
    }

    // Tables alignment is assumed, x coordinate is set equal to both tables
    TABLE_1_X = (table_1.first + table_2.first) / 2;
    TABLE_1_Y = table_1.second;
    TABLE_2_X = (table_1.first + table_2.first) / 2;
    TABLE_2_Y = table_2.second;

    ROS_INFO("Table 1: x=%f y=%f", TABLE_1_X, TABLE_1_Y);
    ROS_INFO("Table 2: x=%f y=%f", TABLE_2_X, TABLE_2_Y);
}

// Send goal to MoveBase
void Movement::sendGoalToMoveBase(double x, double y, const geometry_msgs::Quaternion& orientation) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation = orientation;
    //ROS_INFO("Sending goal: x=%.2f, y=%.2f, orientation=[%.2f, %.2f, %.2f, %.2f]",
    //         x, y, orientation.x, orientation.y, orientation.z, orientation.w);
    ac.sendGoal(goal);
    if (ac.waitForResult(ros::Duration(20))) {
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            //ROS_INFO("Goal reached successfully!");
        } else {
            ROS_WARN("Failed to reach the goal.");
        }
    }
    ros::Duration(0.5).sleep();
}

// Clockwise movement
void Movement::go_clockwise(int target_pos) {
    // Set robot in the appropriate directino to start moving clockwisely
    geometry_msgs::Quaternion escape_orientation;
    switch (cur_pos) {
        case 2: escape_orientation = POS_X_ORIENTATION; break;
        case 5: escape_orientation = NEG_X_ORIENTATION; break;
        case 3: case 4: escape_orientation = NEG_Y_ORIENTATION; break;
        case 6: case 1: escape_orientation = POS_Y_ORIENTATION; break;
    }
    spin(M_PI_2);
    sendGoalToMoveBase(docks[cur_pos - 1].first, docks[cur_pos - 1].second, escape_orientation);
    // Move to next dock until the goal one is reached
    while(cur_pos != target_pos) {
        if(cur_pos==6) {
            cur_pos=1;
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, POS_Y_ORIENTATION);
        }
        else if(cur_pos==1) {
            sendGoalToMoveBase(corns[0].first, corns[0].second, POS_Y_ORIENTATION);
            spin(-M_PI_2);
            sendGoalToMoveBase(corns[0].first, corns[0].second, POS_X_ORIENTATION);
            cur_pos++;
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, POS_X_ORIENTATION);
        }
        else if(cur_pos==2) {
            sendGoalToMoveBase(corns[1].first, corns[1].second, POS_X_ORIENTATION);
            spin(-M_PI_2);
            sendGoalToMoveBase(corns[1].first, corns[1].second, NEG_Y_ORIENTATION);
            cur_pos++;
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, NEG_Y_ORIENTATION);
        }
        else if(cur_pos==3) {
            cur_pos++;
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, NEG_Y_ORIENTATION);
        }
        else if(cur_pos==4) {
            sendGoalToMoveBase(corns[2].first, corns[2].second, NEG_Y_ORIENTATION);
            spin(-M_PI_2);
            sendGoalToMoveBase(corns[2].first, corns[2].second, NEG_X_ORIENTATION);
            cur_pos++;
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, NEG_X_ORIENTATION);
        }
        else if(cur_pos==5) {
            sendGoalToMoveBase(corns[3].first, corns[3].second, NEG_X_ORIENTATION);
            spin(-M_PI_2);
            sendGoalToMoveBase(corns[3].first, corns[3].second, POS_Y_ORIENTATION);
            cur_pos++;
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, POS_Y_ORIENTATION);
        }
    }
    // // Orient the robot towards the table
    // spin(-M_PI_2);
    fix_pos();
}

// Counter-clockwise movement
void Movement::go_counter_clockwise(int target_pos) {
    // Set robot in the appropriate directino to start moving counter-clockwisely
    geometry_msgs::Quaternion escape_orientation;
    switch (cur_pos) {
        case 2: escape_orientation = NEG_X_ORIENTATION; break;
        case 5: escape_orientation = POS_X_ORIENTATION; break;
        case 3: case 4: escape_orientation = POS_Y_ORIENTATION; break;
        case 6: case 1: escape_orientation = NEG_Y_ORIENTATION; break;
    }
    spin(-M_PI_2);
    sendGoalToMoveBase(docks[cur_pos - 1].first, docks[cur_pos - 1].second, escape_orientation);
    // Move to next dock until the goal one is reached
    while(cur_pos != target_pos) {
        if(cur_pos==1) {
            cur_pos=6;
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, NEG_Y_ORIENTATION);
        }
        else if(cur_pos==6) {
            sendGoalToMoveBase(corns[3].first, corns[3].second, NEG_Y_ORIENTATION);
            spin(M_PI_2);
            sendGoalToMoveBase(corns[3].first, corns[3].second, POS_X_ORIENTATION);
            cur_pos--;
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, POS_X_ORIENTATION);
        }
        else if(cur_pos==5) {
            sendGoalToMoveBase(corns[2].first, corns[2].second, POS_X_ORIENTATION);
            spin(M_PI_2);
            sendGoalToMoveBase(corns[2].first, corns[2].second, POS_Y_ORIENTATION);
            cur_pos--;
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, POS_Y_ORIENTATION);
        }
        else if(cur_pos==4) {
            cur_pos--;
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, POS_Y_ORIENTATION);
        }
        else if(cur_pos==3) {
            sendGoalToMoveBase(corns[1].first, corns[1].second, POS_Y_ORIENTATION);
            spin(M_PI_2);
            sendGoalToMoveBase(corns[1].first, corns[1].second, NEG_X_ORIENTATION);
            cur_pos--;
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, NEG_Y_ORIENTATION);
        }
        else if(cur_pos==2) {
            sendGoalToMoveBase(corns[0].first, corns[0].second, NEG_X_ORIENTATION);
            spin(M_PI_2);
            sendGoalToMoveBase(corns[0].first, corns[0].second, NEG_Y_ORIENTATION);
            cur_pos--;
            sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, NEG_Y_ORIENTATION);
        }
    }
    // // Orient the robot towards the table
    // spin(M_PI_2);
    fix_pos();
}

// Navigate to target dock position choosing the shortest route
void Movement::goAround(int target_pos) {
    if (target_pos < 1 || target_pos > 6) {
        ROS_ERROR("Invalid target position.");
        return;
    }
    if (cur_pos == target_pos) {
        // ROS_WARN("Robot is already in target position.");
        return;
    }
    int clockwiseDistance = (target_pos - cur_pos + 6) % 6;
    int counterclockwiseDistance = (cur_pos - target_pos + 6) % 6;
    if (clockwiseDistance <= counterclockwiseDistance)
        go_clockwise(target_pos);
    else 
        go_counter_clockwise(target_pos);
}

// Fix robot position so that it is perfectly (perpendicularly) facing the nearest table
void Movement::fix_pos() {
    switch (cur_pos) {
        case 1: sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, POS_X_ORIENTATION); break;
        case 2: sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, NEG_Y_ORIENTATION); break;
        case 3: sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, NEG_X_ORIENTATION); break;
        case 4: sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, NEG_X_ORIENTATION); break;
        case 5: sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, POS_Y_ORIENTATION); break;
        case 6: sendGoalToMoveBase(docks[cur_pos-1].first, docks[cur_pos-1].second, POS_X_ORIENTATION); break;
    }
    ros::Duration(1.0).sleep();
}

// Change the current robot orientation of a given angle
void Movement::spin(double yaw) {
    move_base_msgs::MoveBaseGoal spin;
    spin.target_pose.header.frame_id="base_link";
    spin.target_pose.header.stamp=ros::Time::now();
    spin.target_pose.pose.position.x=0.0;
    spin.target_pose.pose.position.y=0.0;
    tf2::Quaternion q;
    q.setRPY(0,0,yaw);
    spin.target_pose.pose.orientation.x=q.x();
    spin.target_pose.pose.orientation.y=q.y();
    spin.target_pose.pose.orientation.z=q.z();
    spin.target_pose.pose.orientation.w=q.w();

    ac.sendGoal(spin);   
    bool finished = ac.waitForResult(ros::Duration(15.0));
    ros::Duration(0.5).sleep();
}

// Compute the distance of the given point from the selected dock
float Movement::dock_dist(float x, float y, int dock) {
    if(dock<1 || dock>6) {
        ROS_ERROR("Given dock index is not valid.");
        return -1.0;
    }
    float x1 = x;
    float y1 = y;
    float x2 = docks[dock-1].first;
    float y2 = docks[dock-1].second;
    return std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2);
}

// Compute the index of the dock closest to the given point
int Movement::closest_dock(float x, float y) {
    float minDist = std::numeric_limits<float>::max(); 
    int closestDock = -1;       
    for (int dock = 1; dock <= 6; ++dock) {
        float dist = dock_dist(x, y, dock);
        if (dist < minDist) {
            minDist = dist;
            closestDock = dock;
        }
    }
    return closestDock;
}
