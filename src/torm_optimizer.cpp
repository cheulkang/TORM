/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Modified by : Mincheul Kang */

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <torm/torm_utils.h>
#include <torm/torm_optimizer.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <eigen3/Eigen/Core>

namespace torm
{
    TormOptimizer::TormOptimizer(TormTrajectory* trajectory, const planning_scene::PlanningSceneConstPtr& planning_scene,
                                 const std::string& planning_group, TormParameters* parameters,
                                 const moveit::core::RobotState& start_state,
                                 const std::vector<KDL::Frame> &end_poses,
                                 const std::vector<int> &simplified_points,
                                 TormIKSolver& iksolver,
                                 const bool start_flag,
                                 const robot_model::JointBoundsVector& bounds)
            : full_trajectory_(trajectory)
            , kmodel_(planning_scene->getRobotModel())
            , planning_group_(planning_group)
            , parameters_(parameters)
            , group_trajectory_(*full_trajectory_, planning_group_, DIFF_RULE_LENGTH)
            , planning_scene_(planning_scene)
            , state_(start_state)
            , endPoses_desired_(end_poses)
            , simplified_points_(simplified_points)
            , iksolver_(iksolver)
            , start_flag_(start_flag)
    {
        double size_x = 2.5, size_y = 2.5, size_z = 5.0;
        double resolution = 0.01;
        const collision_detection::WorldPtr& w = (const collision_detection::WorldPtr &) planning_scene->getWorld();
        hy_world_ = new collision_detection::CollisionWorldHybrid(w, Eigen::Vector3d(size_x, size_y, size_z),
                                                                  Eigen::Vector3d(0.0, 0.0, 0.0),
                                                                  false,
                                                                  resolution, 0.0, 0.3);
        if (!hy_world_)
        {
            ROS_WARN_STREAM("Could not initialize hybrid collision world from planning scene");
            return;
        }

        std::map<std::string, std::vector<collision_detection::CollisionSphere>> link_body_decompositions;
        hy_robot_ = new collision_detection::CollisionRobotHybrid(kmodel_, link_body_decompositions,
                                                                  size_x, size_y, size_z,
                                                                  false,
                                                                  resolution, 0.0, 0.3);
        if (!hy_robot_)
        {
            ROS_WARN_STREAM("Could not initialize hybrid collision robot from planning scene");
            return;
        }

        for(uint i = 0; i < bounds.size(); i++){
            vel_limit_.push_back(bounds[i][0][0].max_velocity_);
        }

        initialize();

        const std::vector<const moveit::core::JointModel*> joint_models = joint_model_group_->getActiveJointModels();
        for (size_t joint_i = 0; joint_i < joint_models.size(); joint_i++) {
            const moveit::core::JointModel *joint_model = joint_models[joint_i];

            if (joint_model->getType() == moveit::core::JointModel::REVOLUTE) {
                const moveit::core::RevoluteJointModel *revolute_joint =
                        dynamic_cast<const moveit::core::RevoluteJointModel *>(joint_model);
                if (revolute_joint->isContinuous()) {
                    joint_kinds_.push_back(true);
                }
                else{
                    joint_kinds_.push_back(false);
                }
            }
        }

        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        generator_.seed(seed);

        for (int i = free_vars_start_; i <= free_vars_end_; i++){
            uint num = 1;
            std::vector<double> cart(6);
            for(int j = 0; j < 3; j++){
                cart[j] = (endPoses_desired_[i-free_vars_start_+1].p.data[j]-endPoses_desired_[i-free_vars_start_].p.data[j])/(num+1);
            }
            double rot1[3], rot2[3];
            endPoses_desired_[i-free_vars_start_].M.GetRPY(rot1[0], rot1[1], rot1[2]);
            endPoses_desired_[i-free_vars_start_+1].M.GetRPY(rot2[0], rot2[1], rot2[2]);
            for(int j = 3; j < 6; j++){
                double diff = rot2[j-3] - rot1[j-3];
                if(joint_kinds_[i]) {
                    if (std::abs(diff) > M_PI) {
                        if (diff < 0) {
                            diff = 2.0 * M_PI - std::abs(diff);
                        } else {
                            diff = 2.0 * M_PI - std::abs(diff);
                            diff = -diff;
                        }
                    }
                }
                cart[j] = diff/(num+1);
            }

            for(int j = 0; j < num; j++){
                KDL::Frame e;

                for(int k = 0; k < 3; k++){
                    e.p.data[k] = endPoses_desired_[i-free_vars_start_].p.data[k] + (j+1) * cart[k];
                }

                e.M = e.M.RPY(rot1[0] + (j+1) * cart[3], rot1[1] + (j+1) * cart[4], rot1[2] + (j+1) * cart[5]);

                endPoses_desired_intervals_.push_back(e);
            }
        }

        value_for_endPoses_costs_ = (double)(endPoses_desired_.size() + endPoses_desired_intervals_.size())/parameters_->endPose_cost_weight_;
    }

    void TormOptimizer::initialize()
    {
        // init some variables:
        num_vars_free_ = group_trajectory_.getNumFreePoints();
        num_vars_all_ = group_trajectory_.getNumPoints();
        num_joints_ = group_trajectory_.getNumJoints();

        free_vars_start_ = group_trajectory_.getStartIndex();
        free_vars_end_ = group_trajectory_.getEndIndex();

        delta_twist_.reserve(num_vars_free_);

        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        req.group_name = planning_group_;
        ros::WallTime wt = ros::WallTime::now();
        collision_detection::AllowedCollisionMatrix acm_ = planning_scene_->getAllowedCollisionMatrix();

        hy_world_->getCollisionGradients(req, res, *hy_robot_->getCollisionRobotDistanceField().get(), state_,
                                         &acm_, gsr_);

        ROS_INFO_STREAM("First coll check took " << (ros::WallTime::now() - wt));
        num_collision_points_ = 0;
        start_collision_ = 0;
        end_collision_ = gsr_->gradients_.size();
        for (size_t i = start_collision_; i < end_collision_; i++)
        {
            num_collision_points_ += gsr_->gradients_[i].gradients.size();
        }

        // set up the joint costs:
        joint_costs_.reserve(num_joints_);

        double max_cost_scale = 1.0;

        joint_model_group_ = planning_scene_->getRobotModel()->getJointModelGroup(planning_group_);

        const std::vector<const moveit::core::JointModel*> joint_models = joint_model_group_->getActiveJointModels();
        for (size_t i = 0; i < joint_models.size(); i++)
        {
            const moveit::core::JointModel* model = joint_models[i];
            double joint_cost = 1.0;
            std::string joint_name = model->getName();
            // nh.param("joint_costs/" + joint_name, joint_cost, 1.0);
            std::vector<double> derivative_costs(3);
            derivative_costs[0] = joint_cost * parameters_->smoothness_cost_velocity_;
            derivative_costs[1] = joint_cost * parameters_->smoothness_cost_acceleration_;
            derivative_costs[2] = joint_cost * parameters_->smoothness_cost_jerk_;
            joint_costs_.push_back(TormCost(group_trajectory_, i, derivative_costs, parameters_->ridge_factor_));
            double cost_scale = joint_costs_[i].getMaxQuadCostInvValue();
            if (max_cost_scale < cost_scale)
                max_cost_scale = cost_scale;
        }

        // scale the smoothness costs
        for (int i = 0; i < num_joints_; i++)
        {
            joint_costs_[i].scale(max_cost_scale);
        }

        // allocate memory for matrices:
        smoothness_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
        collision_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
        endPose_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
        final_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
        smoothness_derivative_ = Eigen::VectorXd::Zero(num_vars_all_);
        jacobian_ = Eigen::MatrixXd::Zero(3, num_joints_);
        jacobian_pseudo_inverse_ = Eigen::MatrixXd::Zero(num_joints_, 3);
        jacobian_jacobian_tranpose_ = Eigen::MatrixXd::Zero(3, 3);

        group_trajectory_backup_ = group_trajectory_.getTrajectory();
        local_group_trajectory_ = group_trajectory_.getTrajectory();
        best_trajectory_ = group_trajectory_.getTrajectory();

        collision_point_joint_names_.resize(num_vars_all_, std::vector<std::string>(num_collision_points_));
        collision_point_pos_eigen_.resize(num_vars_all_, EigenSTL::vector_Vector3d(num_collision_points_));
        collision_point_vel_eigen_.resize(num_vars_all_, EigenSTL::vector_Vector3d(num_collision_points_));
        collision_point_acc_eigen_.resize(num_vars_all_, EigenSTL::vector_Vector3d(num_collision_points_));
        joint_axes_.resize(num_vars_all_, EigenSTL::vector_Vector3d(num_joints_));
        joint_positions_.resize(num_vars_all_, EigenSTL::vector_Vector3d(num_joints_));

        collision_point_potential_.resize(num_vars_all_, std::vector<double>(num_collision_points_));
        collision_point_vel_mag_.resize(num_vars_all_, std::vector<double>(num_collision_points_));
        collision_point_potential_gradient_.resize(num_vars_all_, EigenSTL::vector_Vector3d(num_collision_points_));

        std::map<std::string, std::string> fixed_link_resolution_map;
        for (int i = 0; i < num_joints_; i++)
        {
            joint_names_.push_back(joint_model_group_->getActiveJointModels()[i]->getName());
            // ROS_INFO("Got joint %s", joint_names_[i].c_str());
            registerParents(joint_model_group_->getActiveJointModels()[i]);
            fixed_link_resolution_map[joint_names_[i]] = joint_names_[i];
        }

        for (const moveit::core::JointModel* jm : joint_model_group_->getFixedJointModels())
        {
            if (!jm->getParentLinkModel())  // root joint doesn't have a parent
                continue;

            fixed_link_resolution_map[jm->getName()] = jm->getParentLinkModel()->getParentJointModel()->getName();
        }

        for (size_t i = 0; i < joint_model_group_->getUpdatedLinkModels().size(); i++)
        {
            if (fixed_link_resolution_map.find(
                    joint_model_group_->getUpdatedLinkModels()[i]->getParentJointModel()->getName()) ==
                fixed_link_resolution_map.end())
            {
                const moveit::core::JointModel* parent_model = NULL;
                bool found_root = false;

                while (!found_root)
                {
                    if (parent_model == NULL)
                    {
                        parent_model = joint_model_group_->getUpdatedLinkModels()[i]->getParentJointModel();
                    }
                    else
                    {
                        parent_model = parent_model->getParentLinkModel()->getParentJointModel();
                        for (size_t j = 0; j < joint_names_.size(); j++)
                        {
                            if (parent_model->getName() == joint_names_[j])
                            {
                                found_root = true;
                            }
                        }
                    }
                }
                fixed_link_resolution_map[joint_model_group_->getUpdatedLinkModels()[i]->getParentJointModel()->getName()] =
                        parent_model->getName();
            }
        }

        int start = free_vars_start_;
        int end = free_vars_end_;
        for (int i = start; i <= end; ++i)
        {
            size_t j = 0;
            for (size_t g = start_collision_; g < end_collision_; g++)
            {
                collision_detection::GradientInfo& info = gsr_->gradients_[g];

                for (size_t k = 0; k < info.sphere_locations.size(); k++)
                {
                    if (fixed_link_resolution_map.find(info.joint_name) != fixed_link_resolution_map.end())
                    {
                        collision_point_joint_names_[i][j] = fixed_link_resolution_map[info.joint_name];
                    }
                    else
                    {
                        ROS_ERROR("Couldn't find joint %s!", info.joint_name.c_str());
                    }
                    j++;
                }
            }
        }
    }

    TormOptimizer::~TormOptimizer()
    {
        destroy();
    }

    void TormOptimizer::registerParents(const moveit::core::JointModel* model)
    {
        const moveit::core::JointModel* parent_model = NULL;
        bool found_root = false;

        if (model == kmodel_->getRootJoint())
            return;

        while (!found_root)
        {
            if (parent_model == NULL)
            {
                if (model->getParentLinkModel() == NULL)
                {
                    ROS_ERROR_STREAM("Model " << model->getName() << " not root but has NULL link model parent");
                    return;
                }
                else if (model->getParentLinkModel()->getParentJointModel() == NULL)
                {
                    ROS_ERROR_STREAM("Model " << model->getName() << " not root but has NULL joint model parent");
                    return;
                }
                parent_model = model->getParentLinkModel()->getParentJointModel();
            }
            else
            {
                if (parent_model == kmodel_->getRootJoint())
                {
                    found_root = true;
                }
                else
                {
                    parent_model = parent_model->getParentLinkModel()->getParentJointModel();
                }
            }
            joint_parent_map_[model->getName()][parent_model->getName()] = true;
        }
    }

    bool TormOptimizer::localOptimizeTSGD(int maxiter){
        bool optimization_result = false;
        double eCost;
        for (int iter = 0; iter < maxiter; iter++) {
            for (int stage = 0; stage < 2; stage++){
                if(stage == 0){
                    performForwardKinematics();
                    eCost = getEndPoseCost(false);
                }
                else{
                    eCost = getEndPoseCost(true);
                }

                if(best_trajectory_backup_cost_ > eCost){
                    local_group_trajectory_ = group_trajectory_.getTrajectory();
                    if(!isCurrentTrajectoryCollisionFree() && checkJointVelocityLimit() && checkSingularity()){
                        optimization_result = true;
                        best_trajectory_backup_cost_ = eCost;
                        best_trajectory_ = group_trajectory_.getTrajectory();
                        std::cout << eCost/value_for_endPoses_costs_ << " " << (ros::WallTime::now() - start_time_).toSec() << std::endl;
                    }
                }

                if(stage == 0){
                    calculateSmoothnessIncrements();
                    calculateCollisionIncrements();
                    calculateFeasibleIncrements();
                }
                else{
                    calculateEndPoseIncrements();
                }

                addIncrementsToTrajectory();
                handleJointLimits();
                updateGoalConfiguration();

                updateFullTrajectory();
            }
        }

        return optimization_result;
    }

    bool TormOptimizer::iterativeExploration(){
        iteration_ = 0;
        best_trajectory_backup_cost_ = DBL_MAX;

        start_time_ = ros::WallTime::now();
        performForwardKinematics();
        iteration_++;

        double min_v = parameters_->obstacle_cost_weight_-2.0;
        double max_v = parameters_->obstacle_cost_weight_+2.0;
        while((ros::WallTime::now() - start_time_).toSec() < parameters_->planning_time_limit_){
            double f = (double)rand() / RAND_MAX;
            parameters_->obstacle_cost_weight_ = min_v + f * (max_v - min_v);
            getNewTrajectory();
            localOptimizeTSGD(parameters_->exploration_iter_);
        }

        group_trajectory_.getTrajectory() = best_trajectory_;
        updateFullTrajectory();

        if(best_trajectory_backup_cost_ != DBL_MAX){
            return true;
        }
        else{
            return false;
        }
    }

    bool TormOptimizer::checkJointVelocityLimit(){
        if(!parameters_->use_velocity_check_)
            return true;

        // calculate velocity
        double vt = parameters_->time_duration_;
        for (uint i = free_vars_start_-1; i < free_vars_end_; i++){
            Eigen::VectorXd q_c = local_group_trajectory_.row(i);
            Eigen::VectorXd q_t = local_group_trajectory_.row(i+1);

            for (uint j = 0; j < num_joints_; j++){
                double m = std::abs(q_t(j) - q_c(j));
                if(m > M_PI){
                    m = M_PI*2.0 - m;
                }
                double mt = m/vt;
                if(mt > vel_limit_[j]){
                    return false;
                }
            }
        }
        return true;
    }

    bool TormOptimizer::checkSingularity(){
        if(!parameters_->use_singularity_check_)
            return true;

        KDL::JntArray q(num_joints_);
        KDL::Jacobian jac(num_joints_);

        for (int i = free_vars_start_; i <= free_vars_end_; i++) {
            q.data = local_group_trajectory_.row(i);
            iksolver_.getJacobian(q, jac);
            double yosh = std::abs(std::sqrt((jac.data * jac.data.transpose()).determinant()));
            if(yosh < parameters_->singularity_lower_bound_){
                return false;
            }
        }

        return true;
    }

    bool TormOptimizer::isCurrentTrajectoryCollisionFree() const
    {
        bool collision = false;
        moveit_msgs::RobotTrajectory traj;
        traj.joint_trajectory.joint_names = joint_names_;

        std::vector<double> conf(num_joints_);
        for (int i = free_vars_start_-1; i <= free_vars_end_; i++)
        {
            for (int j = 0; j < group_trajectory_.getNumJoints(); j++) {
                conf[j] = local_group_trajectory_(i, j);
            }
            if(!iksolver_.collisionChecking(conf)){
                return true;
            }
        }

        return collision;
    }

    void TormOptimizer::calculatePseudoInverse()
    {
        jacobian_jacobian_tranpose_ =
                jacobian_ * jacobian_.transpose() + Eigen::MatrixXd::Identity(3, 3) * parameters_->pseudo_inverse_ridge_factor_;
        jacobian_pseudo_inverse_ = jacobian_.transpose() * jacobian_jacobian_tranpose_.inverse();
    }

    void TormOptimizer::updateFullTrajectory()
    {
        full_trajectory_->updateFromGroupTrajectory(group_trajectory_);
    }

    void TormOptimizer::fillInLinearInterpolation(int start, int goal)
    {
        Eigen::VectorXd st = group_trajectory_.getTrajectoryPoint(start);
        Eigen::VectorXd gt = group_trajectory_.getTrajectoryPoint(goal);
        double num = goal-start;
        for (int i = 0; i < num_joints_; i++) {
            double diff = (gt(i) - st(i));
            if(joint_kinds_[i]) {
                if (std::abs(diff) > M_PI) {
                    if (diff < 0) {
                        diff = 2 * M_PI - std::abs(diff);
                    } else {
                        diff = -2 * M_PI + std::abs(diff);
                    }
                }
            }
            double theta = diff / num;

            int t = 1;
            for (int j = start+1; j <= goal; j++) {
                group_trajectory_(j, i) = st(i) + (t++) * theta;
            }
        }
    }

    void TormOptimizer::getNewTrajectory() {
        KDL::JntArray q_c(num_joints_);
        KDL::JntArray q_t(num_joints_);
        KDL::JntArray q_cc(num_joints_);

        uint num_simplified_points = simplified_points_.size();
        int start = DIFF_RULE_LENGTH-2;
        int end = num_vars_all_ - DIFF_RULE_LENGTH + 1;

        int start_point = start;
        Eigen::MatrixXd xi = group_trajectory_.getTrajectory();

        //// reset start configuartion
        if(start_flag_){
            if(iksolver_.ikSolverCollFree(endPoses_desired_[0], q_t)){
            }
            else{
                while(!iksolver_.ikSolver(endPoses_desired_[0], q_t));
            }
            group_trajectory_.getTrajectoryPoint(start_point) = q_t.data;
        }
        for (uint i = 0; i < num_simplified_points; i++){
            q_c.data = group_trajectory_.getTrajectoryPoint(start_point);
            KDL::JntArray q_tt;
            q_tt = q_c;
            double bestCost = 10000000000.0;
            std::vector<KDL::JntArray> random_confs;

            for (uint j = 0; j < parameters_->traj_generation_iter_; j++) {
                if(iksolver_.ikSolverCollFree(endPoses_desired_[simplified_points_[i]], q_t)){
                    random_confs.push_back(q_t);
                }
                else if(iksolver_.ikSolver(endPoses_desired_[simplified_points_[i]], q_t)){
                    random_confs.push_back(q_t);
                }
            }

            for (uint j = 0; j < random_confs.size(); j++){
                // update goal configuariton
                group_trajectory_.getTrajectoryPoint(simplified_points_[i]+5) = random_confs[j].data;
                // calculate initial trajectory
                fillInLinearInterpolation(start_point, simplified_points_[i]+5);

                // calculate cost
                double eCost = getEndPoseCost(start_point, simplified_points_[i]+5);
                if(bestCost < eCost){
                    continue;
                }
                updateFullTrajectory();
                performForwardKinematics(start_point, simplified_points_[i]+5);
                double cCost = getCollisionCost(start_point, simplified_points_[i]+5);
                double cost = cCost + eCost;
                if(j == 0){
                    bestCost = cost;
                    for (uint k = (uint)start_point; k <= simplified_points_[i]+5; k++){
                        xi.row(k) = group_trajectory_.getTrajectoryPoint(k);
                    }
                }
                else{
                    if(bestCost > cost){
                        bestCost = cost;
                        for (uint k = (uint)start_point; k <= simplified_points_[i]+5; k++){
                            xi.row(k) = group_trajectory_.getTrajectoryPoint(k);
                        }
                    }
                }
            }

            // update group_trajectory to best xi
            for (uint k = (uint)start_point; k <= simplified_points_[i]+5; k++){
                group_trajectory_.getTrajectoryPoint(k) = xi.row(k);
            }
            start_point = simplified_points_[i]+5;
        }
        for (int j = 0; j < start; j++){
            group_trajectory_.getTrajectoryPoint(j) = xi.row(start);
        }
        for (int j = start; j < end; j++){
            group_trajectory_.getTrajectoryPoint(j) = xi.row(j);
        }
        for (int j = end; j < num_vars_all_; j++){
            group_trajectory_.getTrajectoryPoint(j) = xi.row(end);
        }
        handleJointLimits();
        updateFullTrajectory();
    }

    void TormOptimizer::updateStartConfiguration(){
        Eigen::MatrixXd::RowXpr xx = group_trajectory_.getTrajectoryPoint(free_vars_start_);
        for(uint i = 0; i < free_vars_start_; i++){
            group_trajectory_.getTrajectoryPoint(i) = xx;
        }
    }

    void TormOptimizer::updateGoalConfiguration(){
        Eigen::MatrixXd::RowXpr xx = group_trajectory_.getTrajectoryPoint(free_vars_end_);
        for(uint i = free_vars_end_+1; i < num_vars_all_; i++){
            group_trajectory_.getTrajectoryPoint(i) = xx;
        }
    }

    void TormOptimizer::computeJointProperties(int trajectory_point)
    {
        for (int j = 0; j < num_joints_; j++)
        {
            const moveit::core::JointModel* joint_model = state_.getJointModel(joint_names_[j]);
            const moveit::core::RevoluteJointModel* revolute_joint =
                    dynamic_cast<const moveit::core::RevoluteJointModel*>(joint_model);
            const moveit::core::PrismaticJointModel* prismatic_joint =
                    dynamic_cast<const moveit::core::PrismaticJointModel*>(joint_model);

            std::string parent_link_name = joint_model->getParentLinkModel()->getName();
            std::string child_link_name = joint_model->getChildLinkModel()->getName();
            Eigen::Affine3d joint_transform =
                    state_.getGlobalLinkTransform(parent_link_name) *
                    (kmodel_->getLinkModel(child_link_name)->getJointOriginTransform() * (state_.getJointTransform(joint_model)));

            // joint_transform = inverseWorldTransform * jointTransform;
            Eigen::Vector3d axis;

            if (revolute_joint != NULL)
            {
                axis = revolute_joint->getAxis();
            }
            else if (prismatic_joint != NULL)
            {
                axis = prismatic_joint->getAxis();
            }
            else
            {
                axis = Eigen::Vector3d::Identity();
            }

            axis = joint_transform * axis;

            joint_axes_[trajectory_point][j] = axis;
            joint_positions_[trajectory_point][j] = joint_transform.translation();
        }
    }

    template <typename Derived>
    void TormOptimizer::getJacobian(int trajectory_point, Eigen::Vector3d& collision_point_pos, std::string& joint_name,
                                    Eigen::MatrixBase<Derived>& jacobian) const
    {
        for (int j = 0; j < num_joints_; j++)
        {
            if (isParent(joint_name, joint_names_[j]))
            {
                Eigen::Vector3d column = joint_axes_[trajectory_point][j].cross(
                        Eigen::Vector3d(collision_point_pos(0), collision_point_pos(1), collision_point_pos(2)) -
                        joint_positions_[trajectory_point][j]);

                jacobian.col(j)[0] = column.x();
                jacobian.col(j)[1] = column.y();
                jacobian.col(j)[2] = column.z();
            }
            else
            {
                jacobian.col(j)[0] = 0.0;
                jacobian.col(j)[1] = 0.0;
                jacobian.col(j)[2] = 0.0;
            }
        }
    }

    void TormOptimizer::handleJointLimits()
    {
        const std::vector<const moveit::core::JointModel*> joint_models = joint_model_group_->getActiveJointModels();
        for (size_t joint_i = 0; joint_i < joint_models.size(); joint_i++)
        {
            const moveit::core::JointModel* joint_model = joint_models[joint_i];

            if (joint_model->getType() == moveit::core::JointModel::REVOLUTE)
            {
                const moveit::core::RevoluteJointModel* revolute_joint =
                        dynamic_cast<const moveit::core::RevoluteJointModel*>(joint_model);
                if (revolute_joint->isContinuous())
                {
                    continue;
                }
            }

            const moveit::core::JointModel::Bounds& bounds = joint_model->getVariableBounds();

            double joint_max = -DBL_MAX;
            double joint_min = DBL_MAX;

            for (moveit::core::JointModel::Bounds::const_iterator it = bounds.begin(); it != bounds.end(); it++)
            {
                if (it->min_position_ < joint_min)
                {
                    joint_min = it->min_position_;
                }

                if (it->max_position_ > joint_max)
                {
                    joint_max = it->max_position_;
                }
            }

            int count = 0;

            bool violation = false;
            do
            {
                double max_abs_violation = 1e-6;
                double max_violation = 0.0;
                int max_violation_index = 0;
                violation = false;
                for (int i = free_vars_start_; i <= free_vars_end_; i++)
                {
                    double amount = 0.0;
                    double absolute_amount = 0.0;
                    if (group_trajectory_(i, joint_i) > joint_max)
                    {
                        amount = joint_max - group_trajectory_(i, joint_i);
                        absolute_amount = fabs(amount);
                    }
                    else if (group_trajectory_(i, joint_i) < joint_min)
                    {
                        amount = joint_min - group_trajectory_(i, joint_i);
                        absolute_amount = fabs(amount);
                    }
                    if (absolute_amount > max_abs_violation)
                    {
                        max_abs_violation = absolute_amount;
                        max_violation = amount;
                        max_violation_index = i;
                        violation = true;
                    }
                }

                if (violation)
                {
                    int free_var_index = max_violation_index - free_vars_start_;
                    double multiplier =
                            max_violation / joint_costs_[joint_i].getQuadraticCostInverse()(free_var_index, free_var_index);
                    group_trajectory_.getFreeJointTrajectoryBlock(joint_i) +=
                            multiplier * joint_costs_[joint_i].getQuadraticCostInverse().col(free_var_index);
                }
                if (++count > 10)
                    break;
            } while (violation);
        }
    }

    void TormOptimizer::performForwardKinematics(int start, int end)
    {
        // for each point in the trajectory
        for (int i = start; i <= end; ++i)
        {
            // Set Robot state from trajectory point...
            collision_detection::CollisionRequest req;
            collision_detection::CollisionResult res;
            req.group_name = planning_group_;
            setRobotStateFromPoint(group_trajectory_, i);

            hy_world_->getCollisionGradients(req, res, *hy_robot_->getCollisionRobotDistanceField().get(), state_, &acm_, gsr_);
            computeJointProperties(i);

            // Keep vars in scope
            {
                size_t j = 0;
                for (size_t g = start_collision_; g < end_collision_; g++)
                {
                    collision_detection::GradientInfo& info = gsr_->gradients_[g];

                    for (size_t k = 0; k < info.sphere_locations.size(); k++)
                    {
                        collision_point_pos_eigen_[i][j][0] = info.sphere_locations[k].x();
                        collision_point_pos_eigen_[i][j][1] = info.sphere_locations[k].y();
                        collision_point_pos_eigen_[i][j][2] = info.sphere_locations[k].z();

                        collision_point_potential_[i][j] = getPotential(info.distances[k], info.sphere_radii[k], parameters_->min_clearence_);

                        collision_point_potential_gradient_[i][j][0] = info.gradients[k].x();
                        collision_point_potential_gradient_[i][j][1] = info.gradients[k].y();
                        collision_point_potential_gradient_[i][j][2] = info.gradients[k].z();

                        j++;
                    }
                }
            }
        }
        // now, get the vel and acc for each collision point (using finite differencing)
        double inv_time = 1.0 / group_trajectory_.getDiscretization();
        for (int i = start; i <= end; ++i)
        {
            for (int j = 0; j < num_collision_points_; j++)
            {
                collision_point_vel_eigen_[i][j] = Eigen::Vector3d(0, 0, 0);
                for (int k = -DIFF_RULE_LENGTH / 2; k <= DIFF_RULE_LENGTH / 2; k++)
                {
                    int t = i + k;
                    if(t > end){
                        t = end;
                    }
                    collision_point_vel_eigen_[i][j] +=
                            (inv_time * DIFF_RULES[0][k + DIFF_RULE_LENGTH / 2]) * collision_point_pos_eigen_[t][j];
                }

                // get the norm of the velocity:
                collision_point_vel_mag_[i][j] = collision_point_vel_eigen_[i][j].norm();
            }
        }
    }

    void TormOptimizer::performForwardKinematics()
    {
        double inv_time = 1.0 / group_trajectory_.getDiscretization();
        double inv_time_sq = inv_time * inv_time;

        // calculate the forward kinematics for the fixed states only in the first iteration:
        int start = free_vars_start_;
        if (iteration_ == 0) {
            start = 0;
        }
        int end = num_vars_all_-1;

        // for each point in the trajectory
        for (int i = start; i <= end; ++i)
        {
            // Set Robot state from trajectory point...
            collision_detection::CollisionRequest req;
            collision_detection::CollisionResult res;
            req.group_name = planning_group_;
            setRobotStateFromPoint(group_trajectory_, i);

            hy_world_->getCollisionGradients(req, res, *hy_robot_->getCollisionRobotDistanceField().get(), state_, &acm_, gsr_);
            computeJointProperties(i);

            // Keep vars in scope
            {
                size_t j = 0;
                for (size_t g = start_collision_; g < end_collision_; g++)
                {
                    collision_detection::GradientInfo& info = gsr_->gradients_[g];

                    for (size_t k = 0; k < info.sphere_locations.size(); k++)
                    {
                        collision_point_pos_eigen_[i][j][0] = info.sphere_locations[k].x();
                        collision_point_pos_eigen_[i][j][1] = info.sphere_locations[k].y();
                        collision_point_pos_eigen_[i][j][2] = info.sphere_locations[k].z();

                        collision_point_potential_[i][j] =
                                getPotential(info.distances[k], info.sphere_radii[k], parameters_->min_clearence_);
                        collision_point_potential_gradient_[i][j][0] = info.gradients[k].x();
                        collision_point_potential_gradient_[i][j][1] = info.gradients[k].y();
                        collision_point_potential_gradient_[i][j][2] = info.gradients[k].z();

                        j++;
                    }
                }
            }
        }

        // now, get the vel and acc for each collision point (using finite differencing)
        for (int i = free_vars_start_; i <= free_vars_end_; i++)
        {
            for (int j = 0; j < num_collision_points_; j++)
            {
                collision_point_vel_eigen_[i][j] = Eigen::Vector3d(0, 0, 0);
                collision_point_acc_eigen_[i][j] = Eigen::Vector3d(0, 0, 0);
                for (int k = -DIFF_RULE_LENGTH / 2; k <= DIFF_RULE_LENGTH / 2; k++)
                {
                    collision_point_vel_eigen_[i][j] +=
                            (inv_time * DIFF_RULES[0][k + DIFF_RULE_LENGTH / 2]) * collision_point_pos_eigen_[i + k][j];
                    collision_point_acc_eigen_[i][j] +=
                            (inv_time_sq * DIFF_RULES[1][k + DIFF_RULE_LENGTH / 2]) * collision_point_pos_eigen_[i + k][j];
                }

                // get the norm of the velocity:
                collision_point_vel_mag_[i][j] = collision_point_vel_eigen_[i][j].norm();
            }
        }
    }

    void TormOptimizer::setRobotStateFromPoint(TormTrajectory& group_trajectory, int i) {
        const Eigen::MatrixXd::RowXpr& point = group_trajectory.getTrajectoryPoint(i);

        std::vector<double> joint_states;
        for (int j = 0; j < group_trajectory.getNumJoints(); j++)
        {
            joint_states.push_back(point(0, j));
        }

        state_.setJointGroupPositions(planning_group_, joint_states);
        state_.update();
    }

    //// cost function and its gradient function
    double TormOptimizer::getEndPoseCost(bool grad) {
        double endPose_cost = 0.0;

        // forward kinematics
        Eigen::MatrixXd xi = group_trajectory_.getTrajectory();
        KDL::JntArray q(num_joints_);
        KDL::JntArray pre_q(num_joints_);
        pre_q.data = xi.row(0);
        KDL::JntArray delta_q(num_joints_);
        KDL::Frame endPoses_c;
        KDL::Twist delta_twist;

        uint vv = free_vars_start_-1;
        for (int i = free_vars_start_; i <= free_vars_end_; i++) {
            q.data = xi.row(i);
            iksolver_.fkSolver(q, endPoses_c);
            delta_twist = diff(endPoses_c, endPoses_desired_[i-vv]);
            iksolver_.vikSolver(q, delta_twist, delta_q);
            if(grad)
                endPose_increments_.row(i-free_vars_start_) = delta_q.data;
            double dd = KDL::dot(delta_twist.vel, delta_twist.vel) +
                        KDL::dot(delta_twist.rot, delta_twist.rot);
            endPose_cost += dd;

            KDL::JntArray q_t(num_joints_);
            q_t.data = (pre_q.data + q.data) / 2;
            iksolver_.fkSolver(q_t, endPoses_c);
            delta_twist = diff(endPoses_c, endPoses_desired_intervals_[i-free_vars_start_]);
            iksolver_.vikSolver(q_t, delta_twist, delta_q);
            dd = KDL::dot(delta_twist.vel, delta_twist.vel) +
                 KDL::dot(delta_twist.rot, delta_twist.rot);
            endPose_cost += dd;

            pre_q = q;
        }

        return parameters_->endPose_cost_weight_ * endPose_cost;
    }
    double TormOptimizer::getEndPoseCost(int start, int end) {
        double endPose_cost = 0.0;

        // forward kinematics
        Eigen::MatrixXd xi = group_trajectory_.getTrajectory();
        KDL::JntArray q(num_joints_);
        KDL::JntArray delta_q(num_joints_);

        KDL::Frame ec;
        KDL::Twist dt;
        for (int i = start+1; i < end; i++) {
            q.data = xi.row(i);
            iksolver_.fkSolver(q, ec);
            dt = diff(ec, endPoses_desired_[i-5]);
            double dd = KDL::dot(dt.vel, dt.vel) +
                        KDL::dot(dt.rot, dt.rot);
            endPose_cost += dd;
        }

        return parameters_->endPose_cost_weight_ * endPose_cost;
    }

    double TormOptimizer::getCollisionCost(int start, int end)
    {
        double collision_cost = 0.0;

        for (int i = start; i <= end; i++)
        {
            double state_collision_cost = 0.0;
            for (int j = 0; j < num_collision_points_; j++)
            {
                state_collision_cost += collision_point_potential_[i][j] * collision_point_vel_mag_[i][j];
            }
            collision_cost += state_collision_cost;
        }

        return parameters_->obstacle_cost_weight_ * collision_cost;
    }

    void TormOptimizer::calculateSmoothnessIncrements()
    {
        for (int i = 0; i < num_joints_; i++)
        {
            joint_costs_[i].getDerivative(group_trajectory_.getJointTrajectory(i), smoothness_derivative_);
            smoothness_increments_.col(i) = -smoothness_derivative_.segment(group_trajectory_.getStartIndex(), num_vars_free_);
        }
    }

    void TormOptimizer::calculateEndPoseIncrements(){
        for (int i = 0; i < num_joints_; i++) {
            final_increments_.col(i) = parameters_->jacobian_cost_weight_ * endPose_increments_.col(i);
        }
    }

    void TormOptimizer::calculateCollisionIncrements()
    {
        double potential;
        double vel_mag_sq;
        double vel_mag;
        Eigen::Vector3d potential_gradient;
        Eigen::Vector3d normalized_velocity;
        Eigen::Matrix3d orthogonal_projector;
        Eigen::Vector3d curvature_vector;
        Eigen::Vector3d cartesian_gradient;

        collision_increments_.setZero(num_vars_free_, num_joints_);

        int startPoint = 0;
        int endPoint = free_vars_end_;

        // In stochastic descent, simply use a random point in the trajectory, rather than all the trajectory points.
        // This is faster and guaranteed to converge, but it may take more iterations in the worst case.
        if (parameters_->use_stochastic_descent_) {
            startPoint = (int)(((double)random() / (double)RAND_MAX) * (free_vars_end_ - free_vars_start_) + free_vars_start_);
            if (startPoint < free_vars_start_)
                startPoint = free_vars_start_;
            if (startPoint > free_vars_end_)
                startPoint = free_vars_end_;
            endPoint = startPoint;
        }
        else {
            startPoint = free_vars_start_;
        }

        for (int i = startPoint; i <= endPoint; i++)
        {
            for (int j = 0; j < num_collision_points_; j++)
            {
                potential = collision_point_potential_[i][j];

                if (potential < 0.0001)
                    continue;

                potential_gradient = -collision_point_potential_gradient_[i][j];

                vel_mag = collision_point_vel_mag_[i][j];
                vel_mag_sq = vel_mag * vel_mag;

                normalized_velocity = collision_point_vel_eigen_[i][j] / vel_mag;
                orthogonal_projector = Eigen::Matrix3d::Identity() - (normalized_velocity * normalized_velocity.transpose());
                curvature_vector = (orthogonal_projector * collision_point_acc_eigen_[i][j]) / vel_mag_sq;
                cartesian_gradient = vel_mag * (orthogonal_projector * potential_gradient - potential * curvature_vector);

                // pass it through the jacobian transpose to get the increments
                getJacobian(i, collision_point_pos_eigen_[i][j], collision_point_joint_names_[i][j], jacobian_);

                if (parameters_->use_pseudo_inverse_)
                {
                    calculatePseudoInverse();
                    collision_increments_.row(i - free_vars_start_).transpose() -= jacobian_pseudo_inverse_ * cartesian_gradient;
                }
                else
                {
                    collision_increments_.row(i - free_vars_start_).transpose() -= jacobian_.transpose() * cartesian_gradient;
                }
            }
        }
    }

    void TormOptimizer::calculateFeasibleIncrements()
    {
        for (int i = 0; i < num_joints_; i++){
            final_increments_.col(i) =
                    parameters_->learning_rate_ * ((joint_costs_[i].getQuadraticCostInverse() *
                                                    (parameters_->smoothness_cost_weight_ * smoothness_increments_.col(i)
                                                     + parameters_->obstacle_cost_weight_ * collision_increments_.col(i))
                    ));
        }
    }

    void TormOptimizer::addIncrementsToTrajectory() {
        const std::vector<const moveit::core::JointModel*>& joint_models = joint_model_group_->getActiveJointModels();
        for (size_t i = 0; i < joint_models.size(); i++)
        {
            double scale = 1.0;
            double max = final_increments_.col(i).maxCoeff();
            double min = final_increments_.col(i).minCoeff();
            double max_scale = parameters_->joint_update_limit_ / fabs(max);
            double min_scale = parameters_->joint_update_limit_ / fabs(min);
            if (max_scale < scale)
                scale = max_scale;
            if (min_scale < scale)
                scale = min_scale;
            group_trajectory_.getFreeTrajectoryBlock().col(i) += scale * final_increments_.col(i);
        }
    }
}  // namespace torm
