/* Author: Mincheul Kang */

#ifndef TORM_TORM_DEBUG_H
#define TORM_TORM_DEBUG_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>

#include <torm/torm_ik_solver.h>

namespace torm
{
    class TormDebug
    {
    public:
        TormDebug(const planning_scene::PlanningSceneConstPtr& planning_scene,
                  std::string frame_id);
        virtual ~TormDebug(){};

        void visualizeConfiguration(std::vector<std::string> &indices, std::vector<double> &conf);
        void visualizeTrajectory(std::vector<std::string> &indices, std::vector<std::vector<double> > traj);
        void visualizeEndEffectorPose_line(geometry_msgs::Point p1, geometry_msgs::Point p2, int color);
        void publishEndEffectorPose_line();
    private:
        int num_joints_;
        std::string frame_id_;

        sensor_msgs::JointState js_;
        ros::Publisher joint_pub_;
        ros::Publisher display_pub_;
        ros::Publisher marker_pub_;

        moveit_msgs::DisplayTrajectory display_trajectory_;
        moveit_msgs::RobotTrajectory robot_traj_;

        planning_scene::PlanningSceneConstPtr planning_scene_;

        visualization_msgs::Marker line_list_;
    };
}

#endif //TORM_TORM_DEBUG_H
