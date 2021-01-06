/* Author: Mincheul Kang */

#include <torm/torm_problem.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

namespace torm {
    TormProblem::TormProblem(const planning_scene::PlanningSceneConstPtr& planning_scene){
        planning_scene_ = planning_scene;

        nh_.getParam("/torm/fixed_frame", fixed_frame_);
        nh_.getParam("/torm/planning_group", planning_group_);
        nh_.getParam("/torm/planning_base_link", planning_base_link_);
        nh_.getParam("/torm/planning_tip_link", planning_tip_link_);
        nh_.getParam("/torm/start_config", start_config_);
        nh_.getParam("/torm/start_pose", start_pose_);
        nh_.getParam("/torm/scene_name", scene_name_);
        nh_.getParam("/torm/default_setting_joints", default_setting_joints_);
        nh_.getParam("/torm/default_setting_values", default_setting_values_);

        setCollisionObjects();
        planning_scene_interface_.applyCollisionObjects(collision_objects_);
        setPlanningPath();
    }

    TormProblem::~TormProblem() {

    }

    void TormProblem::setCollisionObjects(){
        XmlRpc::XmlRpcValue obs;

        nh_.getParam("/torm/obstacles", obs);
        for(uint i = 0; i < obs.size(); i++){
            std::string obs_name = "obs" + std::to_string(i);

            collision_objects_.push_back(makeCollisionObject(obs_name, double(obs[i][0]["x"]), double(obs[i][1]["y"]), double(obs[i][2]["z"]),
                                double(obs[i][3]["roll"]), double(obs[i][4]["pitch"]), double(obs[i][5]["yaw"]),
                                double(obs[i][6]["size_x"]), double(obs[i][7]["size_y"]), double(obs[i][8]["size_z"])));
        }

    }

    void TormProblem::removeCollisionObjects(std::map<std::string, moveit_msgs::CollisionObject> &collision_objects_map){
        for(auto& kv : collision_objects_){
            kv.operation = kv.REMOVE;
        }
        planning_scene_interface_.applyCollisionObjects(collision_objects_);
        ros::Duration(1.0).sleep();
    }

    moveit_msgs::CollisionObject TormProblem::makeCollisionObject(std::string name, double x, double y, double z,
                                                                  double roll, double pitch, double yaw,
                                                                  double size_x, double size_y, double size_z){
        moveit_msgs::CollisionObject co;

        co.id = name;
        co.header.frame_id = fixed_frame_;

        co.primitives.resize(1);
        co.primitives[0].type = co.primitives[0].BOX;
        co.primitives[0].dimensions.resize(3);
        co.primitives[0].dimensions[0] = size_x;
        co.primitives[0].dimensions[1] = size_y;
        co.primitives[0].dimensions[2] = size_z;

        co.primitive_poses.resize(1);
        co.primitive_poses[0].position.x = x;
        co.primitive_poses[0].position.y = y;
        co.primitive_poses[0].position.z = z;

        tf::Quaternion q;
        q.setRPY(roll, pitch, yaw);

        co.primitive_poses[0].orientation.w = q.w();
        co.primitive_poses[0].orientation.x = q.x();
        co.primitive_poses[0].orientation.y = q.y();
        co.primitive_poses[0].orientation.z = q.z();

        co.operation = co.ADD;

        return co;
    }

    void TormProblem::setPlanningPath(){
        std::string str;
        std::string path = ros::package::getPath("torm");
        std::ifstream inFile(path + "/src/data/scene/" + scene_name_);
        KDL::Frame end_effector_pose;

        std::vector<std::vector<double> > input_poss;
        std::vector<std::vector<double> > input_rots;
        while(std::getline(inFile, str, '\n')){
            std::vector<std::string> tokens;
            tokens = split(str, ';');
            std::vector<double> tt;
            std::vector<double> tr;
            tt = split_f(tokens[0], ',');
            tr = split_f(tokens[1], ',');
            input_poss.push_back(tt);
            input_rots.push_back(tr);
        }
        inFile.close();

        end_effector_pose.p.data[0] = start_pose_[0];
        end_effector_pose.p.data[1] = start_pose_[1];
        end_effector_pose.p.data[2] = start_pose_[2];

        end_effector_pose.M = end_effector_pose.M.RPY(start_pose_[3], start_pose_[4], start_pose_[5]);

        uint num_motion = input_poss.size();
        for (uint i = 0; i < num_motion; i++){
            KDL::Frame p = end_effector_pose;
            p.p.data[0] += input_poss[i][0];
            p.p.data[1] += input_poss[i][1];
            p.p.data[2] += input_poss[i][2];

            tf::Quaternion q(input_rots[i][1], input_rots[i][2], input_rots[i][3], input_rots[i][0]);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            p.M = p.M.RPY(roll, pitch, yaw);

            target_poses_.push_back(p);
        }
    }

    std::string TormProblem::getSceneName(){
        return scene_name_;
    }

    std::string TormProblem::getPlanningGroup(){
        return planning_group_;
    }

    std::string TormProblem::getFixedFrame(){
        return fixed_frame_;
    }

    std::string TormProblem::getBaseLink(){
        return planning_base_link_;
    }

    std::string TormProblem::getTipLink(){
        return planning_tip_link_;
    }

    std::vector<KDL::Frame> TormProblem::getTargetPoses(){
        return target_poses_;
    }

    std::vector<std::string> TormProblem::getDefaultSettingJoints(){
        return default_setting_joints_;
    }

    std::vector<double> TormProblem::getDefaultSettingValues(){
        return default_setting_values_;
    }

    std::vector<double> TormProblem::getStartConfiguration(){
        return start_config_;
    }
}