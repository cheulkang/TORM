/* Author: Mincheul Kang */

#include <torm/torm_debug.h>

namespace torm {
    TormDebug::TormDebug(const planning_scene::PlanningSceneConstPtr& planning_scene,
                         std::string frame_id) {
        ros::NodeHandle node_handle("~");
        joint_pub_ = node_handle.advertise<sensor_msgs::JointState>("/joint_states", 10);
        display_pub_ = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
        marker_pub_ = node_handle.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
        planning_scene_ = planning_scene;
        frame_id_ = frame_id;

        line_list_.header.frame_id = frame_id;
        line_list_.header.stamp = ros::Time::now();
        line_list_.ns = "torm";
        line_list_.action = visualization_msgs::Marker::ADD;
        line_list_.pose.orientation.w = 1.0;
        line_list_.id = 2;
        line_list_.type = visualization_msgs::Marker::LINE_LIST;
        line_list_.scale.x = 0.01;
        line_list_.scale.y = 0.01;
        line_list_.color.r = 1.0;
        line_list_.color.a = 0.5;
        line_list_.colors.clear();
    }

    void TormDebug::visualizeConfiguration(std::vector<std::string> &indices, std::vector<double> &conf) {
        js_.position.clear();
        js_.name.clear();
        for(int j = 0; j < indices.size(); j++){
            js_.name.push_back(indices[j]);
            js_.position.push_back(conf[j]);
        }
        js_.header.stamp = ros::Time::now();
        joint_pub_.publish(js_);
        ros::Duration(0.1).sleep();
    }

    void TormDebug::visualizeTrajectory(std::vector<std::string> &indices, std::vector<std::vector<double> > traj) {
        display_trajectory_.trajectory.clear();

        robot_traj_.joint_trajectory.joint_names = indices;
        robot_traj_.joint_trajectory.header.stamp = ros::Time::now();
        robot_traj_.joint_trajectory.points.clear();
        robot_traj_.joint_trajectory.points.resize(traj.size());

        for (uint i = 0; i < robot_traj_.joint_trajectory.points.size(); i++) {
            robot_traj_.joint_trajectory.points[i].positions.resize(indices.size());
            for(uint j = 0; j < indices.size(); j++){
                robot_traj_.joint_trajectory.points[i].positions[j] = traj[i][j];
            }
        }
        display_trajectory_.trajectory.push_back(robot_traj_);
        display_pub_.publish(display_trajectory_);
        ros::Duration(1.0).sleep();
    }

    void TormDebug::visualizeEndEffectorPose_line(geometry_msgs::Point p1, geometry_msgs::Point p2, int color) {
        // 0: red, 1: green, 2: blue
        std_msgs::ColorRGBA lineColor;
        lineColor.a = 0.7;

        if(color == 0){
            lineColor.r = 1.0;
        }
        else if(color == 1){
            lineColor.g = 1.0;
        }
        else if(color == 2){
            lineColor.b = 1.0;
        }
        else{
            lineColor.r = 1.0;
            lineColor.g = 1.0;
            lineColor.b = 1.0;
        }

        line_list_.points.push_back(p1);
        line_list_.colors.push_back(lineColor);

        line_list_.points.push_back(p2);
        line_list_.colors.push_back(lineColor);
    }

    void TormDebug::publishEndEffectorPose_line(){
        for(uint i = 0; i < 10; i++){
            marker_pub_.publish(line_list_);
        }
    }
}
