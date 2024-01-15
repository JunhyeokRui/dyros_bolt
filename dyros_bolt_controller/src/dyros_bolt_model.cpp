#include "dyros_bolt_controller/dyros_bolt_model.h"
#include "fstream"



namespace dyros_bolt_controller
{
// std::ofstream torque_txt("/home/yong/Desktop/torque.txt");
constexpr const char* DyrosBoltModel::EE_NAME[2];
constexpr const size_t DyrosBoltModel::HW_TOTAL_DOF;
constexpr const size_t DyrosBoltModel::MODEL_DOF;

constexpr const size_t DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF;
  
// These should be replaced by YAML or URDF or something
const std::string DyrosBoltModel::JOINT_NAME[DyrosBoltModel::HW_TOTAL_DOF] = {
    "FL_HAA", "FL_HFE", "FL_KFE", "FL_ANKLE",
    "FR_HAA", "FR_HFE", "FR_KFE", "FR_ANKLE"};

// Dynamixel Hardware IDs
const int DyrosBoltModel::JOINT_ID[DyrosBoltModel::HW_TOTAL_DOF] = {
    1,2,3,4,
    5,6,7,8};       

DyrosBoltModel::DyrosBoltModel() :
    joint_start_index_{0, 4}
{
    base_position_.setZero();
    q_.setZero();
    q_ext_.setZero();
    extencoder_init_flag_ = false;

    std::string desc_package_path = ros::package::getPath("dyros_bolt_description");
    // std::string urdf_path = desc_package_path + "/robots/bolt.urdf";
    std::string urdf_path = desc_package_path + "/robots/bolt_with_passive.urdf";

    ROS_INFO("Loading DYROS BOLT description from = %s",urdf_path.c_str());
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &model_, true, false);
    // rd_.LoadModelData(urdf_path, true, 1); 
    ROS_INFO("Successfully loaded.");
    // ROS_INFO("Total DoF = %d", rd_.model_.dof_count);
    ROS_INFO("Total DoF = %d", model_.dof_count);
    // ROS_INFO("Total DoF = %d", rd_.model_.q_size);
    ROS_INFO("Total DoF = %d", model_.q_size);
    // ROS_INFO("Total DoF = %d", rd_.model_.qdot_size);
    ROS_INFO("Total DoF = %d", model_.qdot_size);

    if(model_.dof_count != MODEL_WITH_VIRTUAL_DOF)
    {
        ROS_WARN("The DoF in the model file and the code do not match.");
        // ROS_WARN("Model file = %d, Code = %d", rd_.model_.dof_count, (int)MODEL_WITH_VIRTUAL_DOF);
        ROS_WARN("Model file = %d, Code = %d", model_.dof_count, (int)MODEL_WITH_VIRTUAL_DOF);
    }

    for (size_t i=0; i<2; i++)
    {
        // end_effector_id_[i] = rd_.model_.GetBodyId(EE_NAME[i]);
        end_effector_id_[i] = model_.GetBodyId(EE_NAME[i]);
        ROS_INFO("%s: id - %d",EE_NAME[i], end_effector_id_[i]);
        // std::cout << rd_.model_.mBodies[end_effector_id_[i]].mCenterOfMass << std::endl;
        std::cout << model_.mBodies[end_effector_id_[i]].mCenterOfMass << std::endl;
    }

    for (size_t i=0; i<HW_TOTAL_DOF; i++)
    {
        joint_name_map_[JOINT_NAME[i]] = i;
    }
}

void DyrosBoltModel::test()
{
    // Eigen::Matrix<double, DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF+1, 1> q_vjoint;
    Eigen::Matrix<double, DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF, 1> q_vjoint;
    Eigen::Matrix<double, DyrosBoltModel::MODEL_WITH_VIRTUAL_DOF, 1> qdot_vjoint;
    q_vjoint.setZero();
    q_vjoint(MODEL_WITH_VIRTUAL_DOF) = 1;
    // q_vjoint << 0, 0, 0.43, 0, 0, 0,
    //             1.57, 1.57, 0, 0,
    //             0, 0, 0, 0;

    qdot_vjoint.setZero();


    updateKinematics(q_vjoint, qdot_vjoint, qdot_vjoint);

    std::cout << "left_leg_jacobian_" << std::endl;
    std::cout << leg_jacobian_[0] << std::endl << std::endl;
    std::cout << "right_leg_jacobian_" << std::endl;
    std::cout << leg_jacobian_[1] << std::endl;
    std::cout << "current_transform_" << std::endl;
    std::cout << current_transform_[0].translation() << std::endl << std::endl;
    std::cout << current_transform_[1].translation() << std::endl << std::endl;
    std::cout << "com" << std::endl;
    std::cout << com_<< std::endl;
}

void DyrosBoltModel::updateKinematics(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& qddot)
{
    // q_virtual_ = q;
    // q_virtual_dot_ = qdot;
    // q_virtual_ddot_ = qddot;

    RigidBodyDynamics::UpdateKinematicsCustom(model_, &q, &qdot, &qddot);

    VectorXd G_, B_;
    G_.setZero(MODEL_WITH_VIRTUAL_DOF);
    B_.setZero(MODEL_WITH_VIRTUAL_DOF);

    RigidBodyDynamics::NonlinearEffects(model_, q, qddot, G_);
    RigidBodyDynamics::NonlinearEffects(model_, q, qdot, B_);

    // Gravity + Coriolis
    // std::cout << "GRAVITY"<<std::endl;
    // std::cout << G_.block<MODEL_DOF,1>(6,0).transpose() << std::endl;
    // std::cout << B_.block<MODEL_DOF,1>(6,0).transpose() << std::endl;

    command_Torque = B_.block<MODEL_DOF,1>(6,0);
    // command_Torque.setZero(MODEL_DOF);

    // getInertiaMatrixDoF(&full_inertia_mat_);
    // getInertiaMatrixlegDoF(&leg_inertia_mat_);
    getCenterOfMassPosition(&com_);
    getCenterOfMassPositionDot(&comDot_);
    for(unsigned int i=0; i<2; i++)
    {
        getTransformEndEffector((EndEffector)i, &current_transform_[i]);
        getJacobianMatrix4DoF((EndEffector)i, &leg_jacobian_[i]);
        getJacobianMatrix14DoF((EndEffector)i, &leg_with_vlink_jacobian_[i]);
    }
}

void DyrosBoltModel::updateSensorData(const Eigen::Vector6d &r_ft, const Eigen::Vector6d &l_ft, const Eigen::Vector12d &q_ext, const Eigen::Vector3d &acc, const Eigen::Vector3d &angvel, const Eigen::Vector3d &grav_rpy)
{
    r_ft_wrench_ = r_ft;
    l_ft_wrench_ = l_ft;

    q_ext_ = q_ext;
    accel_ = acc;
    angvel_ = angvel;
    grav_rpy_ = grav_rpy;
}

void DyrosBoltModel::updateSimCom(const Eigen::Vector3d &sim_com)
{
    com_simulation_ = sim_com;
}

void DyrosBoltModel::updateSimGyro(const Eigen::Vector3d &sim_gyro)
{
    gyro_simulation_ = sim_gyro;
}

void DyrosBoltModel::updateSimAccel(const Eigen::Vector3d &sim_accel)
{
    accel_simulation_ = sim_accel;
}

void DyrosBoltModel::updateSimRfoot(const Eigen::Isometry3d &sim_rfoot)
{
    rfoot_simulation_ = sim_rfoot;
}

void DyrosBoltModel::updateSimLfoot(const Eigen::Isometry3d &sim_lfoot)
{
    lfoot_simulation_ = sim_lfoot;
}

void DyrosBoltModel::updateSimBase(const Eigen::Isometry3d &sim_base)
{
    base_simulation_ = sim_base;
}

void DyrosBoltModel::updateMujCom(const Eigen::Vector6d &sim_lfoot)
{
    q_virtual_1 = sim_lfoot;
}

void DyrosBoltModel::updateMujComDot(const Eigen::Vector6d &sim_base)
{
    q_dot_virtual_1 = sim_base;
}


void DyrosBoltModel::getTransformEndEffector // must call updateKinematics before calling this function
(EndEffector ee, Eigen::Isometry3d* transform_matrix)
{
    // *transform_matrix = rd_.link_[end_effector_id_[ee]].GetSpatialTranform();
    // transform_matrix->translation() = rd_.link_[end_effector_id_[ee]-2].xpos;
    // transform_matrix->linear() =rd_.link_[end_effector_id_[ee]-2].rotm;

    transform_matrix->translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates
      (model_, q_virtual_, end_effector_id_[ee], base_position_, false);
    transform_matrix->linear() = RigidBodyDynamics::CalcBodyWorldOrientation
      (model_, q_virtual_, end_effector_id_[ee], false).transpose();
}


void DyrosBoltModel::getJacobianMatrix4DoF(EndEffector ee, Eigen::Matrix<double, 6, 4> *jacobian)
{
    // *jacobian = rd_.link_[end_effector_id_[ee]-2].jac_.middleCols<4>(joint_start_index_[ee]+6);
    Eigen::MatrixXd full_jacobian(6,MODEL_WITH_VIRTUAL_DOF);
    full_jacobian.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, end_effector_id_[ee],
                                            Eigen::Vector3d::Zero(), full_jacobian, false);
    // swap
    jacobian->block<3, 4>(0, 0) = full_jacobian.block<3, 4>(3, joint_start_index_[ee]+6);
    jacobian->block<3, 4>(3, 0) = full_jacobian.block<3, 4>(0, joint_start_index_[ee]+6);
}

void DyrosBoltModel::getJacobianMatrix14DoF(EndEffector ee, Eigen::Matrix<double, 6, MODEL_WITH_VIRTUAL_DOF> *jacobian)
{
    Eigen::MatrixXd full_jacobian(6,MODEL_WITH_VIRTUAL_DOF);
    full_jacobian.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, end_effector_id_[ee],
                                            Eigen::Vector3d::Zero(), full_jacobian, false);
    switch (ee)
    {
    case EE_LEFT_FOOT:
        // Virtual Link
        // jacobian->block<6, 6>(0, 0) = rd_.link_[end_effector_id_[ee]-2].jac_.leftCols(6);
        // left Leg Link
        // jacobian->block<6, 4>(0, 6) = rd_.link_[end_effector_id_[ee]-2].jac_.leftCols<4>(joint_start_index_[ee]+6);
        // Virtual Link
        jacobian->block<3, 6>(0, 0) = full_jacobian.block<3, 6>(3, 0);
        jacobian->block<3, 6>(3, 0) = full_jacobian.block<3, 6>(0, 0);

        // left Leg Link
        jacobian->block<3, 4>(0, 6) = full_jacobian.block<3, 4>(3, joint_start_index_[ee]+6);
        jacobian->block<3, 4>(3, 6) = full_jacobian.block<3, 4>(0, joint_start_index_[ee]+6);
        break;
    case EE_RIGHT_FOOT:
        // Virtual Link
        // jacobian->block<6, 6>(0, 0) = rd_.link_[end_effector_id_[ee]-2].jac_.leftCols(6);
        // right Leg Link
        // jacobian->block<6, 4>(0, 10) = rd_.link_[end_effector_id_[ee]-2].jac_.leftCols<4>(joint_start_index_[ee]+6);
        // Virtual Link
        jacobian->block<3, 6>(0, 0) = full_jacobian.block<3, 6>(3, 0);
        jacobian->block<3, 6>(3, 0) = full_jacobian.block<3, 6>(0, 0);

        // left Leg Link
        jacobian->block<3, 4>(0, 10) = full_jacobian.block<3, 4>(3, joint_start_index_[ee]+6);
        jacobian->block<3, 4>(3, 10) = full_jacobian.block<3, 4>(0, joint_start_index_[ee]+6);
        break;
    }
}

void DyrosBoltModel::getCenterOfMassPosition(Eigen::Vector3d* position)
{
    // *position = rd_.com_pos;
    RigidBodyDynamics::Math::Vector3d com, com_dot;
    com.setZero();
    VectorXd qddot;
    qddot.setZero(MODEL_WITH_VIRTUAL_DOF);
    double mass;
    ///////////////////////////////////////////model//..q..qdot..qddot..mass..com..comdot.comddot..angularmomentum..change_of_angularmomentum..updatekin
    RigidBodyDynamics::Utils::CalcCenterOfMass(model_, q_virtual_, q_virtual_dot_, &qddot, mass, com, &com_dot, NULL, NULL, NULL, false);

    *position = com;
}

void DyrosBoltModel::getCenterOfMassPositionDot(Eigen::Vector3d* position_dot)
{
    // *position_dot = rd_.com_vel;
    RigidBodyDynamics::Math::Vector3d com, com_dot;
    com.setZero();
    VectorXd qddot;
    qddot.setZero(MODEL_WITH_VIRTUAL_DOF);
    double mass;

    RigidBodyDynamics::Utils::CalcCenterOfMass(model_, q_virtual_, q_virtual_dot_, &qddot, mass, com, &com_dot, NULL, NULL, NULL, false);
    *position_dot = com_dot;
}

void DyrosBoltModel::getInertiaMatrixDoF(Eigen::Matrix<double, MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF> *inertia)
{
    // *inertia = rd_.A_;
}

void DyrosBoltModel::getInertiaMatrixlegDoF(Eigen::Matrix<double, MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF> *leg_inertia)
{
//   leg_inertia->block<MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF>(0, 0) = rd_.A_.block<MODEL_WITH_VIRTUAL_DOF, MODEL_WITH_VIRTUAL_DOF>(0, 0);
    // *leg_inertia = rd_.A_;
}

}