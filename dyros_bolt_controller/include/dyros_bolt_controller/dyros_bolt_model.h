#ifndef DYROS_JET_MODEL_H
#define DYROS_JET_MODEL_H

#include <string>
#include <map>

#include <ros/ros.h>
#include <ros/package.h>
#include <rbdl/rbdl.h>
// #include <rbdl/addons/urdfreader/urdfreader.h>
// #include <libdwbc/dwbc.h>
#include "math_type_define.h"

// #define MODEL_DOF 6

namespace dyros_bolt_controller
{

class DyrosBoltModel
{
public:
    DyrosBoltModel();
    
    enum EndEffector : unsigned int {EE_LEFT_FOOT, EE_RIGHT_FOOT};

    static constexpr size_t HW_TOTAL_DOF = 8;
    // static constexpr size_t MODEL_DOF = 6;

    static constexpr size_t MODEL_WITH_VIRTUAL_DOF = 14;

    static const std::string JOINT_NAME[HW_TOTAL_DOF];
    static const int JOINT_ID[HW_TOTAL_DOF];

    static constexpr const char* EE_NAME[2] = {"FL_FOOT", "FR_FOOT"};

    unsigned int end_effector_id_[2];
    const unsigned int joint_start_index_[2];
    VectorXd command_Torque;

    void test();

    std::map<std::string, size_t> joint_name_map_;
    inline size_t getIndex(const std::string& joint_name) const
    {
        return joint_name_map_.at(joint_name);
    }

    inline bool isPossibleIndex(const std::string& joint_name) const
    {
        return (joint_name_map_.find(joint_name) != joint_name_map_.end());
    }

    // Calc Jacobian, Transformation
    void updateKinematics(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot, const Eigen::VectorXd &qddot);
    void updateSensorData(const Eigen::Vector6d &r_ft, const Eigen::Vector6d &l_ft);

    void updateSensorData(const Eigen::Vector6d &r_ft, const Eigen::Vector6d &l_ft, const Eigen::Vector12d &q_ext, const Eigen::Vector3d &acc, const Eigen::Vector3d &angvel, const Eigen::Vector3d &grav_rpy);

    void updateSimCom(const Eigen::Vector3d &sim_com);
    void updateSimGyro(const Eigen::Vector3d &sim_gyro);
    void updateSimAccel(const Eigen::Vector3d &sim_accel);
    void updateSimRfoot(const Eigen::Isometry3d &sim_rfoot);
    void updateSimLfoot(const Eigen::Isometry3d &sim_lfoot);
    void updateSimBase(const Eigen::Isometry3d &sim_base);
    void updateMujCom(const Eigen::Vector6d &sim_lfoot);
    void updateMujComDot(const Eigen::Vector6d &sim_base);


    void getTransformEndEffector(EndEffector ee, Eigen::Isometry3d* transform_matrix);
    // void getTransformEndEffector(EndEffector ee, Eigen::Vector3d* position, Eigen::Matrix3d* rotation);

    // void getTransformEndEffector(EndEffector ee, const Eigen::VectorXd& q, bool update_kinematics,
    //                                 Eigen::Vector3d* position, Eigen::Matrix3d* rotation);


    void getJacobianMatrix4DoF(EndEffector ee, Eigen::Matrix<double, 6, 4> *jacobian);
    void getJacobianMatrix14DoF(EndEffector ee, Eigen::Matrix<double, 6, MODEL_WITH_VIRTUAL_DOF> *jacobian);

    void getCenterOfMassPosition(Eigen::Vector3d* position);
    void getCenterOfMassPositionDot(Eigen::Vector3d* position);

    void getInertiaMatrixDoF(Eigen::Matrix<double, MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF> *inertia);
    void getInertiaMatrixlegDoF(Eigen::Matrix<double, MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF> *leg_inertia);

    const Eigen::Vector12d& getCurrentExtencoder(){ return q_ext_; }
    const Eigen::Isometry3d& getCurrentTransform(EndEffector ee) { return current_transform_[ee]; }
    const Eigen::Matrix<double, 6, 4>& getLegJacobian(EndEffector ee) { return leg_jacobian_[ee]; }
    const Eigen::Matrix<double, 6, MODEL_WITH_VIRTUAL_DOF>& getLegWithVLinkJacobian(EndEffector ee) { return leg_with_vlink_jacobian_[ee]; }
    const Eigen::Vector3d& getCurrentCom(){ return com_;}
    const Eigen::Vector3d& getCurrentComDot(){return comDot_;}

    const Eigen::Vector3d& getSimulationCom(){return com_simulation_;}
    const Eigen::Vector3d& getSimulationGyro(){return gyro_simulation_;}
    const Eigen::Vector3d& getSimulationAccel(){return accel_simulation_;}


    const Eigen::Vector6d& getMujocoCom(){return q_virtual_1;}
    const Eigen::Vector6d& getMujocoComDot(){return q_dot_virtual_1;}

    const Eigen::Isometry3d& getSimulationRfoot(){return rfoot_simulation_;}
    const Eigen::Isometry3d& getSimulationLfoot(){return lfoot_simulation_;}
    const Eigen::Isometry3d& getSimulationBase(){return base_simulation_;}

    const Eigen::Vector6d& getRightFootForce() {return r_ft_wrench_;}
    const Eigen::Vector6d& getLeftFootForce() {return l_ft_wrench_;}
    const Eigen::Vector3d& getImuAccel() {return accel_;}
    const Eigen::Vector3d& getImuAngvel() {return angvel_;}
    const Eigen::Vector3d& getImuGravityDirection() {return grav_rpy_;}


    const Eigen::Matrix<double, MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF>& getLegInertia() { return leg_inertia_mat_; }
    const Eigen::Matrix<double, MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF>& getFullInertia() { return full_inertia_mat_; }

private:
    // DWBC::RobotData rd_;
    RigidBodyDynamics::Model model_;
    
    Eigen::Vector6d q_;
    Eigen::Matrix<double, MODEL_WITH_VIRTUAL_DOF, 1> q_virtual_;
    Eigen::Matrix<double, MODEL_WITH_VIRTUAL_DOF, 1> q_virtual_dot_;
    Eigen::Matrix<double, MODEL_WITH_VIRTUAL_DOF, 1> q_virtual_ddot_;
    Eigen::Vector12d q_ext_;

    bool extencoder_init_flag_;

    Eigen::Vector3d base_position_;

    Eigen::Isometry3d current_transform_[2];

    Eigen::Matrix<double, 6, 4> leg_jacobian_[2];
    Eigen::Matrix<double, 6, MODEL_WITH_VIRTUAL_DOF> leg_with_vlink_jacobian_[2];

    Eigen::Matrix<double, MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF> full_inertia_mat_;
    Eigen::Matrix<double, MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF> leg_inertia_mat_;


    Eigen::Vector3d com_;
    Eigen::Vector3d comDot_;
    Eigen::Vector3d com_simulation_;

    Eigen::Vector3d accel_;
    Eigen::Vector3d angvel_;
    Eigen::Vector3d grav_rpy_;

    Eigen::Vector6d r_ft_wrench_;
    Eigen::Vector6d l_ft_wrench_;

    Eigen::Vector3d gyro_simulation_;
    Eigen::Vector3d accel_simulation_;

    Eigen::Isometry3d rfoot_simulation_;
    Eigen::Isometry3d lfoot_simulation_;
    Eigen::Isometry3d base_simulation_;

    Eigen::Vector6d q_virtual_1;
    Eigen::Vector6d q_dot_virtual_1;

    Eigen::MatrixXd A_;
    Eigen::MatrixXd A_temp_;

};
typedef Eigen::Matrix<double, DyrosBoltModel::HW_TOTAL_DOF, 1> VectorQd;
}
#endif // DYROS_JET_MODEL_H