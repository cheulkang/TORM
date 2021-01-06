/* Author: Mincheul Kang */

#include <ros/ros.h>
#include <torm/torm_parameters.h>
#include <torm/torm_ik_solver.h>
#include <torm/torm_debug.h>
#include <torm/torm_problem.h>
#include <torm/torm_optimizer.h>
#include <torm/torm_trajectory.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <visualization_msgs/Marker.h>
#include <eigen_conversions/eigen_kdl.h>

void initParameters(torm::TormParameters &params_, int endPose_size){
    params_.planning_time_limit_ = 50.0;
    params_.smoothness_cost_weight_ = 15.0/endPose_size;
    params_.endPose_cost_weight_ = 10.0;
    params_.obstacle_cost_weight_ = 6.0;
    params_.jacobian_cost_weight_ = 1.0;
    params_.learning_rate_ = 0.01;
    params_.smoothness_cost_velocity_ = 1.0;
    params_.smoothness_cost_acceleration_ = 0.0;
    params_.smoothness_cost_jerk_ = 0.0;
    params_.ridge_factor_ = 0.0;
    params_.use_pseudo_inverse_ = true;
    params_.pseudo_inverse_ridge_factor_ = 1e-4;
    params_.joint_update_limit_ = 0.1;
    params_.min_clearence_ = 0.3;
    params_.collision_threshold_ = 0.07;
    params_.use_stochastic_descent_ = true;
    params_.use_velocity_check_ = true;
    params_.use_singularity_check_ = false;

    params_.singularity_lower_bound_ = 0.005; // fetch arm
    params_.quantum_iter_ = 50;
    params_.gaussian_weight_ = 0.02;
    params_.quantum_tunnelling_iter_ = 100;
    params_.time_duration_ = 0.2;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "torm");
    ros::NodeHandle node_handle("~");

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    // initialize planning interface
    std::map<std::string, moveit_msgs::CollisionObject> collision_objects_map = planning_scene_interface_.getObjects();
    std::vector<moveit_msgs::CollisionObject> v_co;
    for(auto& kv : collision_objects_map){
        kv.second.operation = kv.second.REMOVE;
        v_co.push_back(kv.second);
    }
    planning_scene_interface_.applyCollisionObjects(v_co);
    ros::Duration(1.0).sleep();

    torm::TormProblem prob(planning_scene);

    const std::string PLANNING_GROUP = prob.getPlanningGroup();
    const robot_state::JointModelGroup *joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
    std::vector<std::string> indices = joint_model_group->getActiveJointModelNames();
    robot_model::JointBoundsVector joint_bounds;
    joint_bounds = joint_model_group->getActiveJointModelsBounds();
    unsigned int num_dof = (unsigned int)joint_bounds.size();

    collision_objects_map = planning_scene_interface_.getObjects();
    for(auto& kv : collision_objects_map){
        planning_scene->processCollisionObjectMsg(kv.second);
    }

    robot_state::RobotState state = planning_scene->getCurrentState();
    std::vector<std::string> defaultJoints = prob.getDefaultSettingJoints();
    std::vector<double> defaultValues = prob.getDefaultSettingValues();
    for(uint i = 0; i < defaultJoints.size(); i++)
        state.setJointPositions(defaultJoints[i], &defaultValues[i]);
    planning_scene->setCurrentState(state);

    torm::TormIKSolver iksolver(PLANNING_GROUP, planning_scene, prob.getBaseLink(), prob.getTipLink());
    torm::TormDebug debug(planning_scene, prob.getFixedFrame());

    std::vector<KDL::Frame> targetPoses = prob.getTargetPoses();
    std::vector<int> simplified_points;

    int gap = 50;
    for(int i = gap; i < targetPoses.size(); i+=gap){
        simplified_points.push_back(i);
    }
    if(simplified_points[simplified_points.size()-1] != targetPoses.size()-1){
        simplified_points.push_back(targetPoses.size()-1);
    }

    torm::TormParameters params;
    initParameters(params, targetPoses.size());

    // visualize input poses
    const Eigen::Isometry3d& baseT = planning_scene->getFrameTransform(prob.getBaseLink());
    KDL::Frame baseKDL;
    tf::transformEigenToKDL(baseT, baseKDL);
    for(uint i = 0; i < targetPoses.size()-1; i++){
        geometry_msgs::Point p1;
        p1.x = targetPoses[i].p.data[0] + baseKDL.p.data[0];
        p1.y = targetPoses[i].p.data[1] + baseKDL.p.data[1];
        p1.z = targetPoses[i].p.data[2] + baseKDL.p.data[2];

        geometry_msgs::Point p2;
        p2.x = targetPoses[i+1].p.data[0] + baseKDL.p.data[0];
        p2.y = targetPoses[i+1].p.data[1] + baseKDL.p.data[1];
        p2.z = targetPoses[i+1].p.data[2] + baseKDL.p.data[2];

        debug.visualizeEndEffectorPose_line(p1, p2, 0);
    }
    debug.publishEndEffectorPose_line();
    ros::Duration(1.0).sleep();

    bool start_flag = false;
    KDL::JntArray q_c(num_dof);
    std::vector<double> s_conf = prob.getStartConfiguration();
    if(s_conf.size() == 0){
        start_flag = true;
    }
    else{
        for (uint j = 0; j < num_dof; j++) {
            q_c(j) = s_conf[j];
        }
    }
    if(start_flag){
        if(!iksolver.ikSolverCollFree(targetPoses[0], q_c)){
            ROS_INFO("No found a valid start configuration.");
            return 0;
        }
    }

    // generate trajectory
    torm::TormTrajectory trajectory(planning_scene->getRobotModel(), int(targetPoses.size()), params.time_duration_, PLANNING_GROUP);
    trajectory.getTrajectoryPoint(0) = q_c.data;

    // trajectory optimization
    torm::TormOptimizer opt(&trajectory, planning_scene, PLANNING_GROUP, &params, state,
                            targetPoses, simplified_points, iksolver, start_flag, joint_bounds);

    bool result = opt.iterativeExploration();

    if(!result) {
        ROS_INFO("No found a valid trajectory.");
        return 0;
    }

    // visualize output poses
    std::vector<KDL::JntArray> trajectory_points;
    std::vector<std::vector<double> > trajectory_points_vis;
    std::vector<double> q(num_dof);
    for (int i = 0; i < trajectory.getNumPoints(); i++){
        for (int j = 0; j < num_dof; j++) {
            q_c(j) = trajectory.getTrajectoryPoint(i)(j);
            q[j] = q_c(j);
        }
        trajectory_points.push_back(q_c);
        trajectory_points_vis.push_back(q);
    }

    KDL::Frame ee;
    iksolver.fkSolver(trajectory_points[0], ee);
    for(uint i = 1; i < trajectory.getNumPoints(); i++){
        geometry_msgs::Point p1;
        p1.x = ee.p.data[0] + baseKDL.p.data[0];
        p1.y = ee.p.data[1] + baseKDL.p.data[1];
        p1.z = ee.p.data[2] + baseKDL.p.data[2];

        iksolver.fkSolver(trajectory_points[i], ee);
        geometry_msgs::Point p2;
        p2.x = ee.p.data[0] + baseKDL.p.data[0];
        p2.y = ee.p.data[1] + baseKDL.p.data[1];
        p2.z = ee.p.data[2] + baseKDL.p.data[2];

        debug.visualizeEndEffectorPose_line(p1, p2, 2);
    }
    debug.publishEndEffectorPose_line();
    ros::Duration(1.0).sleep();

    // visualize trajectory
    std::vector<std::string> indices_vis = indices;
    for(uint i = 0; i < defaultJoints.size(); i++){
        indices_vis.push_back(defaultJoints[i]);
    }
    for(uint i = 0; i < trajectory_points_vis.size(); i++) {
        for(uint j = 0; j < defaultValues.size(); j++){
            trajectory_points_vis[i].push_back(defaultValues[j]);
        }
    }
    debug.visualizeTrajectory(indices_vis, trajectory_points_vis);

    return 0;
}