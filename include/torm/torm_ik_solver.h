/* Author: Mincheul Kang */

#ifndef TORM_IK_SOLVER_H
#define TORM_IK_SOLVER_H

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <trac_ik/trac_ik.hpp>
#include <moveit/planning_scene/planning_scene.h>

namespace torm
{
    class TormIKSolver
    {
    public:
        TormIKSolver(std::string planning_group, const planning_scene::PlanningSceneConstPtr& planning_scene,
                const std::string base_link, const std::string tip_link);
        virtual ~TormIKSolver(){};

        double fRand(double min, double max) const;
        double fRand(int i) const;
        void getRandomConfiguration(KDL::JntArray& q);
        void setCollisionChecker();
        bool collisionChecking(std::vector<double> values);

        bool ikSolver(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out);
        bool ikSolver(const KDL::Frame& p_in, KDL::JntArray& q_out);
        bool ikSolverCollFree(const KDL::Frame& p_in, KDL::JntArray& q_out);
        bool ikSolverCollFree(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out);
        void fkSolver(const KDL::JntArray& q_init, KDL::Frame& p_in);
        void vikSolver(const KDL::JntArray& q, const KDL::Twist& delta_twist, KDL::JntArray& delta_q);
        void getJacobian(const KDL::JntArray& q, KDL::Jacobian& jac);
        uint getDoF();
    private:
        KDL::Chain chain_;
        KDL::JntArray ll_, ul_; //lower joint limits, upper joint limits
        uint num_joint_;
        std::string planning_group_;
        planning_scene::PlanningSceneConstPtr planning_scene_;

        collision_detection::CollisionRequest collision_request_;
        collision_detection::CollisionResult collision_result_;

        std::unique_ptr<TRAC_IK::TRAC_IK> tracik_solver_;
        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
        std::unique_ptr<KDL::ChainJntToJacSolver> jacsolver_;
        std::unique_ptr<KDL::ChainIkSolverVel_pinv> vik_solver_;

        uint max_tried_;
    };
}

#endif //TORM_MOTION_PLANNER_TORM_IK_SOLVER_H
