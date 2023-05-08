/* Author: Mincheul Kang */

#include <ros/ros.h>
#include <torm/torm_ik_solver.h>

namespace torm
{
    TormIKSolver::TormIKSolver(std::string planning_group, const planning_scene::PlanningSceneConstPtr& planning_scene,
                               const std::string base_link, const std::string tip_link) {
        planning_group_ = planning_group;
        planning_scene_ = planning_scene;

        tracik_solver_.reset(new TRAC_IK::TRAC_IK (base_link, tip_link, "/robot_description", 0.001, 1e-4));

        setCollisionChecker();

        bool valid = tracik_solver_->getKDLChain(chain_);
        valid = tracik_solver_->getKDLLimits(ll_, ul_);
        num_joint_ = chain_.getNrOfJoints();

        // Set up KDL IK
        KDL::ChainFkSolverPos_recursive fk_solver(chain_);

        fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_)); // Forward kin. solver
        vik_solver_.reset(new KDL::ChainIkSolverVel_pinv(chain_)); // PseudoInverse vel solver
        jacsolver_.reset(new KDL::ChainJntToJacSolver(chain_));

        max_tried_ = 30;
    }

    void TormIKSolver::getRandomConfiguration(KDL::JntArray& q){
        for(uint j = 0; j < num_joint_; j++){
            q(j) = fRand(ll_(j), ul_(j));
        }
    }

    void TormIKSolver::setCollisionChecker() {
        collision_request_.group_name = planning_group_;
        collision_request_.contacts = true;
        collision_request_.max_contacts = 100;
        collision_request_.max_contacts_per_pair = 1;
        collision_request_.verbose = false;
    }

    bool TormIKSolver::collisionChecking(std::vector<double> values) {
        collision_result_.clear();
        robot_state::RobotState state = planning_scene_->getCurrentState();
        state.setJointGroupPositions(planning_group_, values);
        planning_scene_->checkCollision(collision_request_, collision_result_, state);

        return !collision_result_.collision;
    }
	
    bool TormIKSolver::selfCollisionChecking(std::vector<double> values) {
        collision_result_.clear();
        robot_state::RobotState state = planning_scene_->getCurrentState();
        state.setJointGroupPositions(planning_group_, values);
        planning_scene_->checkSelfCollision(collision_request_, collision_result_, state);

        return !collision_result_.collision;
    }

    double TormIKSolver::fRand(double min, double max) const {
        double f = (double)rand() / RAND_MAX;
        if(max > 2*M_PI){
            return -M_PI + f * (2*M_PI);
        }
        return min + f * (max - min);
    }

    double TormIKSolver::fRand(int i) const {
        double f = (double)rand() / RAND_MAX;
        if(ul_(i) > 2*M_PI){
            return -M_PI + f * (2*M_PI);
        }
        return ll_(i) + f * (ul_(i) - ll_(i));
    }

    bool TormIKSolver::ikSolverCollFree(const KDL::Frame& p_in, KDL::JntArray& q_out) {
        int rc = -1;
        int tried = 0;

        KDL::JntArray q_c(num_joint_);
        std::vector<double> conf(num_joint_);
        while(rc < 0){
            for(uint j = 0; j < num_joint_; j++){
                q_c(j) = fRand(ll_(j), ul_(j));
            }
            rc = tracik_solver_->CartToJnt(q_c, p_in, q_out);
            for(uint j = 0; j < num_joint_; j++){
                conf[j] = q_out(j);
            }
            if(!collisionChecking(conf)){
                rc = -1;
            }
            tried++;
            if(tried == max_tried_ && rc < 0){
                return false;
            }
        }
        return true;
    }

    bool TormIKSolver::ikSolverCollFree(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out) {
        int rc = tracik_solver_->CartToJnt(q_init, p_in, q_out);
        if(rc < 0){
            return false;
        }
        std::vector<double> conf(num_joint_);
        for(uint j = 0; j < num_joint_; j++){
            conf[j] = q_out(j);
        }
        if(!collisionChecking(conf)){
            return false;
        }
        else{
            return true;
        }
    }

    bool TormIKSolver::ikSolver(const KDL::Frame& p_in, KDL::JntArray& q_out) {
        int rc = -1;
        KDL::JntArray q_c(num_joint_);

        int tried = 0;
        while(rc < 0){
            for(uint j = 0; j < num_joint_; j++){
                q_c(j) = fRand(ll_(j), ul_(j));
            }
            rc = tracik_solver_->CartToJnt(q_c, p_in, q_out);
            tried++;
            if(tried == max_tried_ && rc < 0){
                return false;
            }
        }
        return true;
    }

    bool TormIKSolver::ikSolver(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out) {
        int rc = tracik_solver_->CartToJnt(q_init, p_in, q_out);
        if(rc < 0){
            return false;
        }
        return true;
    }

    void TormIKSolver::fkSolver(const KDL::JntArray& q_init, KDL::Frame& p_in){
        fk_solver_->JntToCart(q_init, p_in);
    }

    uint TormIKSolver::getDoF(){
        return num_joint_;
    }

    void TormIKSolver::vikSolver(const KDL::JntArray& q, const KDL::Twist& delta_twist, KDL::JntArray& delta_q){
        vik_solver_->CartToJnt(q, delta_twist, delta_q);
    }

    void TormIKSolver::getJacobian(const KDL::JntArray& q, KDL::Jacobian& jac){
        jacsolver_->JntToJac(q, jac);
    }
}
