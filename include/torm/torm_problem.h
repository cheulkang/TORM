/* Author: Mincheul Kang */

#ifndef TORM_TORM_PROBLEM_H
#define TORM_TORM_PROBLEM_H

#include <ros/ros.h>
#include <ros/package.h>

#include <fstream>
#include <trac_ik/trac_ik.hpp>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <torm/torm_utils.h>

namespace torm
{
    class TormProblem
    {
    public:
        TormProblem(const planning_scene::PlanningSceneConstPtr& planning_scene);
        ~TormProblem();

        void setCollisionObjects();
        void removeCollisionObjects(std::map<std::string, moveit_msgs::CollisionObject> &collision_objects_map);
        void setPlanningPath();
        moveit_msgs::CollisionObject makeCollisionObject(std::string name, double x, double y, double z,
                                                         double roll, double pitch, double yaw,
                                                         double size_x, double size_y, double size_z);
        std::string getSceneName();
        std::string getPlanningGroup();
        std::string getFixedFrame();
        std::string getBaseLink();
        std::string getTipLink();
        std::vector<KDL::Frame> getTargetPoses();
        std::vector<int> getSubSampledPoses();
        std::vector<std::string> getDefaultSettingJoints();
        std::vector<double> getDefaultSettingValues();
        std::vector<double> getStartConfiguration();
    private:
        ros::NodeHandle                                             nh_;
        std::string                                                 fixed_frame_;
        std::string                                                 planning_group_;
        std::string                                                 planning_base_link_;
        std::string                                                 planning_tip_link_;
        std::vector<double>                                         start_config_;
        std::vector<double>                                         start_pose_;
        std::string                                                 scene_name_;
        std::vector<std::string>                                    default_setting_joints_;
        std::vector<double>                                         default_setting_values_;
        std::vector<moveit_msgs::CollisionObject>                   collision_objects_;
        std::vector<KDL::Frame>                                     target_poses_;
        std::vector<int>                                            sub_sampled_poses_;

        planning_scene::PlanningSceneConstPtr                       planning_scene_;
        moveit::planning_interface::PlanningSceneInterface          planning_scene_interface_;
    };
}

#endif //TORM_TORM_PROBLEM_H
