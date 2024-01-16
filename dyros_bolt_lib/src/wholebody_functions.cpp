#include "wholebody_functions.h"

using namespace DYROS_BOLT;

namespace WBC
{
    void SetContactInit(RobotData &rd_)
    {
        rd_.grav_ref << 0, 0, -9.81;

        rd_.ee_[0].contact_point << 0.03, 0, -0.1585;
        rd_.ee_[1].contact_point << 0.03, 0, -0.1585;
        // rd_.ee_[2].contact_point << 0.0, 0.0, -0.035;
        // rd_.ee_[3].contact_point << 0.0, 0.0, -0.035;

        rd_.ee_[0].sensor_point << 0.0, 0.0, -0.09;
        rd_.ee_[1].sensor_point << 0.0, 0.0, -0.09;

        rd_.ee_[0].InitializeEE(rd_.link_[DYROS_BOLT::FL_LOWER_LEG], 0.15, 0.075, 40, 0.2, 0.2);
        rd_.ee_[1].InitializeEE(rd_.link_[DYROS_BOLT::FR_LOWER_LEG], 0.15, 0.075, 40, 0.2, 0.2);

        // rd_.ee_[2].InitializeEE(rd_.link_[DYROS_BOLT::Left_Hand], 0.02, 0.02, 40, 0.2, 0.2);
        // rd_.ee_[3].InitializeEE(rd_.link_[DYROS_BOLT::Right_Hand], 0.02, 0.02, 40, 0.2, 0.2);

        for (int i = 0; i < MODEL_DOF; i++)
            rd_.torque_limit[i] = 1000 / NM2CNT[i];
    }

    void CheckTorqueLimit(RobotData &rd_, VectorQd command_torque)
    {
        double warn_percentage = 0.8;
        double max_percentage = 0.99;

        if (rd_.torqueLimitWarning)
        {
        }
        else
        {
        }

        if (rd_.torqueLimitCritical)
        {
        }
        else
        {
        }

        bool torque_warn[MODEL_DOF] = {false};
        bool torque_ciritical[MODEL_DOF] = {false};

        for (int i = 0; i < MODEL_DOF; i++)
        {
            if (command_torque(i) > rd_.torque_limit[i] * max_percentage)
            {
                torque_warn[i] = true;
            }

            if (command_torque(i) > rd_.torque_limit[i] * warn_percentage)
            {
                torque_ciritical[i] = true;
            }
        }

        bool all_warn = false;
        bool all_critical = false;

        for (int i = 0; i < MODEL_DOF; i++)
        {
            all_warn = all_warn || torque_warn[i];
            all_critical = all_critical || torque_ciritical[i];
        }

        if (torque_ciritical)
        {

            if (all_warn)
            {
                if (all_critical)
                {
                    if (!rd_.torqueLimitCritical)
                    {
                        std::cout << " Torque Critical, over 100 percent" << std::endl;
                        rd_.positionControlSwitch = true;

                        rd_.torqueLimitCritical = true;
                    }
                }
                else
                {
                    rd_.torqueLimitCritical = false;

                    if (!rd_.torqueLimitWarning)
                    {
                        std::cout << " Torque Warning, over 80 percent" << std::endl;
                        rd_.torqueLimitWarning = true;
                    }
                }
            }
            else
            {
                rd_.torqueLimitWarning = false;
                rd_.torqueLimitCritical = false;
            }
        }
    }

    void CalcContact(RobotData &rd_)
    {
        rd_.I_C.setIdentity(rd_.contact_index * 6, rd_.contact_index * 6);
        rd_.Lambda_c = (rd_.J_C * rd_.A_inv_ * rd_.J_C.transpose()).inverse();
        rd_.J_C_INV_T = rd_.Lambda_c * rd_.J_C * rd_.A_inv_;
        rd_.N_C = MatrixVVd::Identity() - rd_.J_C.transpose() * rd_.J_C_INV_T;
        rd_.W = rd_.A_inv_.bottomRows(MODEL_DOF) * rd_.N_C.rightCols(MODEL_DOF);
        // rd_.W_inv = DyrosMath::WinvCalc(rd_.W, rd_.qr_V2);

        rd_.W_inv = DyrosMath::pinv_COD(rd_.W, rd_.qr_V2);

        //// this line is problem!!!! (to JH)
        // rd_.NwJw = rd_.qr_V2.transpose() * (rd_.J_C_INV_T.rightCols(MODEL_DOF).topRows(6) * rd_.qr_V2.transpose()).inverse();
 
        rd_.G.setZero();
        for (int i = 0; i < MODEL_DOF + 1; i++)
            rd_.G -= rd_.link_[i].jac_com.cast<double>().topRows(3).transpose() * rd_.link_[i].mass * rd_.grav_ref;
    }

    bool GravMinMax(VectorQd torque)
    {
        static bool loading = false;
        char gf_[] = "/home/dyros/.DYROS_BOLT_bootlog/minmax_log";
        char gf_v[] = "/home/dyros/.DYROS_BOLT_bootlog/minmax_view";
        static double tminmax[66];
        // torque,

        if (!loading)
        {
            std::ifstream ifs(gf_, std::ios::binary);

            if (!ifs.is_open())
            {
                std::cout << "GMM read failed " << std::endl;
            }
            else
            {
                for (int i = 0; i < 66; i++)
                {
                    ifs.read(reinterpret_cast<char *>(&tminmax[i]), sizeof(double));
                }

                ifs.close();
            }
            loading = true;
        }

        bool record = false;
        for (int i = 0; i < 33; i++)
        {
            if (tminmax[i] > torque[i])
            {
                record = true;

                tminmax[i] = torque[i];
            }

            if (tminmax[i + 33] < torque[i])
            {
                record = true;

                tminmax[i + 33] = torque[i];
            }
        }

        if (record)
        {
            std::ofstream ofs(gf_, std::ios::binary);
            std::ofstream ofs_view(gf_v);
            if (!ofs.is_open())
            {
                std::cout << "GMM write failed " << std::endl;
            }
            else
            {
                for (int i = 0; i < 66; i++)
                    ofs.write(reinterpret_cast<char const *>(&tminmax[i]), sizeof(double));
                ofs.close();
            }

            if (!ofs_view.is_open())
            {
                std::cout << "GMM view write failed " << std::endl;
            }
            else
            {
                ofs_view << "MIN VALUE : " << std::endl;
                for (int i = 0; i < 33; i++)
                {
                    ofs_view << tminmax[i] << std::endl;
                }
                ofs_view << "MAX VALUE : " << std::endl;
                for (int i = 0; i < 33; i++)
                {
                    ofs_view << tminmax[i + 33] << std::endl;
                }
            }
        }

        return true;
    }

    void ContactCalcDefault(RobotData &Robot)
    {

        Robot.ee_[0].SetContact(Robot.model_, Robot.q_virtual_);
        Robot.ee_[1].SetContact(Robot.model_, Robot.q_virtual_);
        // Robot.ee_[2].SetContact(Robot.model_, Robot.q_virtual_);
        // Robot.ee_[3].SetContact(Robot.model_, Robot.q_virtual_);
    }

    // VectorQd GetGravityOnContactLocal(RobotData Robot, bool left_foot, bool right_foot)
    // {
    //     SetContact(Robot,left_foot,right_foot);
    //     return WBC::ContactForceRedistributionTorqueWalking(Robot, WBC::GravityCompensationTorque(Robot), 0.9, 1, 0);
    // }

    void SetContact(RobotData &Robot, bool left_foot, bool right_foot, bool left_hand, bool right_hand)
    {
        Robot.ee_[0].contact = left_foot;
        Robot.ee_[1].contact = right_foot;
        // Robot.ee_[2].contact = left_hand;
        // Robot.ee_[3].contact = right_hand;

        Robot.contact_index = 0;
        if (left_foot)
        {
            Robot.contact_part[Robot.contact_index] = DYROS_BOLT::FL_LOWER_LEG;
            Robot.ee_idx[Robot.contact_index] = 0;
            Robot.contact_index++;
        }
        if (right_foot)
        {
            Robot.contact_part[Robot.contact_index] = DYROS_BOLT::FR_LOWER_LEG;
            Robot.ee_idx[Robot.contact_index] = 1;
            Robot.contact_index++;
        }
        // if (left_hand)
        // {
        //     Robot.contact_part[Robot.contact_index] = DYROS_BOLT::Left_Hand;
        //     Robot.ee_idx[Robot.contact_index] = 2;
        //     Robot.contact_index++;
        // }
        // if (right_hand)
        // {
        //     Robot.contact_part[Robot.contact_index] = DYROS_BOLT::Right_Hand;
        //     Robot.ee_idx[Robot.contact_index] = 3;
        //     Robot.contact_index++;
        // }

        Robot.J_C.setZero(Robot.contact_index * 6, MODEL_DOF_VIRTUAL);

        for (int i = 0; i < Robot.contact_index; i++)
        {
            Robot.J_C.block(i * 6, 0, 6, MODEL_DOF_VIRTUAL) = Robot.ee_[Robot.ee_idx[i]].jac_contact.cast<double>();
        }
        CalcContact(Robot);
    }

    Vector3d GetFstarPos(LinkData &link_, bool a_traj_switch, bool com_switch)
    {
        Vector3d fstar;

        if (com_switch)
        {
            if (a_traj_switch)
            {
                for (int i = 0; i < 3; i++)
                    fstar(i) = link_.pos_p_gain(i) * (link_.x_traj(i) - link_.xipos(i)) + link_.pos_d_gain(i) * (link_.v_traj(i) - link_.vi(i)) + link_.pos_a_gain(i) * link_.a_traj(i);
            }
            else
            {

                for (int i = 0; i < 3; i++)
                    fstar(i) = link_.pos_p_gain(i) * (link_.x_traj(i) - link_.xipos(i)) + link_.pos_d_gain(i) * (link_.v_traj(i) - link_.vi(i));
            }
        }
        else
        {

            if (a_traj_switch)
            {
                for (int i = 0; i < 3; i++)
                    fstar(i) = link_.pos_a_gain(i) * link_.a_traj(i) + link_.pos_p_gain(i) * (link_.x_traj(i) - link_.xpos(i)) + link_.pos_d_gain(i) * (link_.v_traj(i) - link_.v(i));
            }
            else
            {

                for (int i = 0; i < 3; i++)
                    fstar(i) = link_.pos_p_gain(i) * (link_.x_traj(i) - link_.xpos(i)) + link_.pos_d_gain(i) * (link_.v_traj(i) - link_.v(i));
            }
        }

        return fstar;
    }

    Vector3d GetFstarRot(LinkData &link_)
    {
        Vector3d fstar;

        Vector3d ad;

        ad = DyrosMath::getPhi(link_.rotm, link_.r_traj);

        for (int i = 0; i < 3; i++)
            fstar(i) = -link_.rot_p_gain(i) * ad(i) + link_.rot_d_gain(i) * (link_.w_traj(i) - link_.w(i));

        // std::cout << ad.transpose() << "\t" << (link_.w_traj - link_.w).transpose() << std::endl;
        // std::cout << DyrosMath::rot2Euler_tf(link_.rotm).transpose() << "\t" << DyrosMath::rot2Euler_tf(link_.r_traj) << std::endl;
        // return link_.rot_p_gain.cwiseProduct(DyrosMath::getPhi(link_.rotm, link_.r_traj)); // + link_.rot_d_gain.cwiseProduct(link_.w_traj - link_.w);

        return fstar;
    }

    Vector6d GetFstar6d(LinkData &link_, bool a_traj_switch, bool com_switch)
    {
        Vector6d f_star;

        f_star.segment(0, 3) = GetFstarPos(link_, a_traj_switch, com_switch);
        f_star.segment(3, 3) = GetFstarRot(link_);

        return f_star;
    }

    // void CalcNonlinear(RobotData &rd_)
    // {
    //     RigidBodyDynamics::Math::VectorNd tau_;
    //     RigidBodyDynamics::NonlinearEffects(rd_.model_, rd_.q_virtual_, VectorVQf::Zero(), tau_);

    //     std::cout<<tau_.transpose();
    // }

    VectorQd GravityCompensationTorque(RobotData &rd_)
    {
        rd_.torque_grav = rd_.W_inv * (rd_.A_inv_.bottomRows(MODEL_DOF) * (rd_.N_C * rd_.G));
        rd_.P_C = rd_.J_C_INV_T * rd_.G;

        return rd_.torque_grav;
    }

    VectorQd GravityCompenstationTorque_Isolated(RobotData &rd_, bool contact_left_foot_, bool contact_right_foot_)
    {
        return VectorQd::Zero();
    }

    // template <int TaskNum>
    // Eigen::Matrix<double, MODEL_DOF, TaskNum> GetJKT(Matrix<double, TaskNum, MODEL_DOF_VIRTUAL> J_task_)
    // {
    //     Matrix<double, MODEL_DOF, TaskNum> res;
    //     Matrix<double, TaskNum, TaskNum> lambda_ = (J_task_ * rd_.A_inv_ * rd_.N_C * J_task_.transpose()).inverse();
    //     Matrix<double, TaskNum, MODEL_DOF> Q_ = lambda_ * J_task_ * rd_.A_inv_ * rd_.N_C.rightCols(MODEL_DOF);

    //     res = rd_.W_inv * DyrosMath::pinv_QR(Q_ * rd_.W_inv * Q_.transpose());

    //     return res;
    // }
    MatrixXd GetJKT1(RobotData &rd_, MatrixXd &J_task)
    {
        MatrixXd lambda_ = (J_task * rd_.A_inv_ * rd_.N_C * J_task.transpose()).inverse();
        MatrixXd Q_ = lambda_ * J_task * rd_.A_inv_ * rd_.N_C.rightCols(MODEL_DOF);

        return rd_.W_inv * Q_.transpose() * DyrosMath::pinv_QR_prev(Q_ * rd_.W_inv * Q_.transpose());
    }
    MatrixXd GetJKT1(RobotData &rd_, MatrixXd &J_task, MatrixXd &lambda)
    {
        lambda = (J_task * rd_.A_inv_ * rd_.N_C * J_task.transpose()).inverse();
        MatrixXd Q_ = lambda * J_task * rd_.A_inv_ * rd_.N_C.rightCols(MODEL_DOF);

        return rd_.W_inv * Q_.transpose() * DyrosMath::pinv_QR_prev(Q_ * rd_.W_inv * Q_.transpose());
    }
    MatrixXd GetJKT2(RobotData &rd_, MatrixXd &J_task)
    {
        int t_d = J_task.rows();
        MatrixXd lambda_ = (J_task * rd_.A_inv_ * rd_.N_C * J_task.transpose()).llt().solve(MatrixXd::Identity(t_d, t_d));
        MatrixXd Q_ = lambda_ * J_task * rd_.A_inv_ * rd_.N_C.rightCols(MODEL_DOF);

        return rd_.W_inv * Q_.transpose() * DyrosMath::pinv_QR(Q_ * rd_.W_inv * Q_.transpose());
    }

    VectorQd TaskControlTorque(RobotData &rd_, VectorXd f_star)
    {
        int task_dof = rd_.J_task.rows();
        // rd_.J_task = J_task;
        rd_.J_task_T = rd_.J_task.transpose();

        rd_.lambda_inv = rd_.J_task * rd_.A_inv_ * rd_.N_C * rd_.J_task_T;

        rd_.lambda = rd_.lambda_inv.llt().solve(MatrixXd::Identity(task_dof, task_dof));

        rd_.J_task_inv_T = rd_.lambda * rd_.J_task * rd_.A_inv_ * rd_.N_C;

        rd_.Q = rd_.J_task_inv_T.rightCols(MODEL_DOF);

        rd_.Q_T_ = rd_.Q.transpose();

        // std::cout<<"1"<<std::endl;
        rd_.Q_temp = rd_.Q * rd_.W_inv * rd_.Q_T_;
        // std::cout<<"2"<<std::endl;

        rd_.Q_temp_inv = DyrosMath::pinv_COD(rd_.Q_temp);

        // DyrosMath::dc_inv_QR(rd_.J_task)

        // std::cout<<"3"<<std::endl;

        return rd_.W_inv * (rd_.Q_T_ * (rd_.Q_temp_inv * (rd_.lambda * f_star)));
    }

    VectorQd ContactTorqueQP(RobotData &rd_, VectorQd Control_Torque)
    {
        /*

        modified axis contact force : mfc ( each contact point's z axis is piercing COM position)

        minimize contact forces.




        rotm * (rd_.J_C_INV_T.rightCols(MODEL_DOF) * control_torque + rd_.J_C_INV_T * rd_.G);


        rd_.J_C_INV_T.rightCols(MODEL_DOF).transpose() * rotm.transpose() * rotm *rd_.J_C_INV_T.rightCols(MODEL_DOF) - 2 * rd_.J_C_INV_T * rd_.G



        min mfc except z axis

        s.t. zmp condition, friction condition.

        H matrix :

        contact condition : on/off
        contact mode : plane, line, point


        s. t. zmp condition :      (left_rotm.transpose(), 0; 0, right_rotm.transpose()) * rd_.J_C_INV_T.rightCols(MODEL_DOF) * (control_torue + contact_torque) + rd_.J_C_INV_T * rd_.G



        contact_force = rd_.J_C_INV_T.rightCols(MODEL_DOF)*(torque_control + torque_contact) + rd_.J_C_INV_T * rd_.G;

        torque_contact = Robot.qr_V2.transpose() * (Robot.J_C_INV_T.rightCols(MODEL_DOF).topRows(6) * Robot.qr_V2.transpose()).inverse() * desired_contact_force;

        rd_.J_C_INV_T.rightCols(MODEL_DOF) * (torque + nwjw * des_Force) - rd_.

        torque_contact = nwjw * des_Force

        */

        // Vector12d Fc = rd_.J_C_INV_T.rightCols(MODEL_DOF) * Control_Torque - rd_.J_C_INV_T * rd_.G;

        // Vector3d cpos_l, cpos_r;

        // cpos_l = rd_.link_[Left_Foot].rotm.transpose() * (rd_.link_[COM_id].xpos - rd_.link_[Left_Foot].xpos);
        // cpos_r = rd_.link_[Right_Foot].rotm.transpose() * (rd_.link_[COM_id].xpos - rd_.link_[Right_Foot].xpos);

        // Matrix3d crot_l, crot_r;

        // crot_r = DyrosMath::rotateWithX(-atan(cpos_r(1) / cpos_r(2))) * DyrosMath::rotateWithY(atan(cpos_r(0) / sqrt(cpos_r(1) * cpos_r(1) + cpos_r(2) * cpos_r(2))));

        // crot_l = DyrosMath::rotateWithX(-atan(cpos_l(1) / cpos_l(2))) * DyrosMath::rotateWithY(atan(cpos_l(0) / sqrt(cpos_l(1) * cpos_l(1) + cpos_l(2) * cpos_l(2))));

        // Vector3d cres_l, cres_r;

        // cres_l = crot_l.transpose() * cpos_l;

        // cres_r = crot_r.transpose() * cpos_r;

        // Vector12d ContactForce;
        // Eigen::Matrix<double, 12, 12> RotM_;

        if (rd_.contact_index == 1)
        {
            return VectorQd::Zero();
        }
        else
        {

            Eigen::MatrixXd crot_matrix;
            Eigen::MatrixXd RotW;

            crot_matrix.setZero(rd_.contact_index * 6, rd_.contact_index * 6);

            RotW.setIdentity(rd_.contact_index * 6, rd_.contact_index * 6);

            for (int i = 0; i < rd_.contact_index; i++)
            {
                Vector3d cv = rd_.link_[rd_.contact_part[i]].rotm.transpose() * (rd_.link_[COM_id].xpos - rd_.link_[rd_.contact_part[i]].xpos);
                Matrix3d cm = DyrosMath::rotateWithX(-atan(cv(1) / cv(2))) * DyrosMath::rotateWithY(atan(cv(0) / sqrt(cv(1) * cv(1) + cv(2) * cv(2))));
                crot_matrix.block(i * 6, i * 6, 3, 3) = crot_matrix.block(i * 6 + 3, i * 6 + 3, 3, 3) = cm.transpose() * rd_.link_[rd_.contact_part[i]].rotm.transpose();
                RotW(i * 6 + 2, i * 6 + 2) = 0;
            }

            // RotM_.setZero();

            // RotM_.block(0, 0, 3, 3) = crot_l.transpose() * rd_.link_[Left_Foot].rotm.transpose();
            // RotM_.block(3, 3, 3, 3) = crot_l.transpose() * rd_.link_[Left_Foot].rotm.transpose();

            // RotM_.block(6, 6, 3, 3) = crot_r.transpose() * rd_.link_[Right_Foot].rotm.transpose();
            // RotM_.block(9, 9, 3, 3) = crot_r.transpose() * rd_.link_[Right_Foot].rotm.transpose();

            // Vector12d COM_Rel_ContactForce;

            // COM_Rel_ContactForce = RotM_ * Fc;

            static CQuadraticProgram qp_torque_contact_;
            static int contact_dof, const_num;

            const_num = 4;

            qp_torque_contact_.InitializeProblemSize(rd_.contact_index * 6 - 6, rd_.contact_index * const_num);

            static MatrixXd H;
            static VectorXd g;
            static MatrixXd A_t;

            // std::cout<<"1"<<std::endl;

            MatrixXd NwJw = rd_.qr_V2.transpose() * (rd_.J_C_INV_T.rightCols(MODEL_DOF).topRows(6) * rd_.qr_V2.transpose()).inverse();

            // std::cout<<"2"<<std::endl;
            A_t = RotW * crot_matrix * rd_.J_C_INV_T.rightCols(MODEL_DOF) * NwJw;

            // std::cout<<"3"<<std::endl;
            H = A_t.transpose() * A_t;

            // std::cout<<"4"<<std::endl;
            g = (RotW * crot_matrix * (rd_.J_C_INV_T.rightCols(MODEL_DOF) * Control_Torque - rd_.P_C)).transpose() * A_t;

            // std::cout<<"5"<<std::endl;
            qp_torque_contact_.UpdateMinProblem(H, g);

            // Constraint

            MatrixXd A_const_a;
            A_const_a.setZero(const_num * rd_.contact_index, 6 * rd_.contact_index);

            MatrixXd A__mat;

            MatrixXd A_rot;

            A_rot.setZero(rd_.contact_index * 6, rd_.contact_index * 6);

            for (int i = 0; i < rd_.contact_index; i++)
            {
                // std::cout<<"1"<<std::endl;
                A_rot.block(i * 6, i * 6, 3, 3) = rd_.link_[rd_.contact_part[i]].rotm.transpose(); // rd_.ee_[i].rotm.transpose();

                // std::cout<<"1"<<std::endl;
                A_rot.block(i * 6 + 3, i * 6 + 3, 3, 3) = rd_.link_[rd_.contact_part[i]].rotm.transpose();

                // std::cout<<"1"<<std::endl;
                A_const_a(i * const_num + 0, i * 6 + 2) = rd_.ee_[i].cs_y_length;
                A_const_a(i * const_num + 0, i * 6 + 3) = 1;

                A_const_a(i * const_num + 1, i * 6 + 2) = rd_.ee_[i].cs_y_length;
                A_const_a(i * const_num + 1, i * 6 + 3) = -1;

                A_const_a(i * const_num + 2, i * 6 + 2) = rd_.ee_[i].cs_x_length;
                A_const_a(i * const_num + 2, i * 6 + 4) = 1;

                A_const_a(i * const_num + 3, i * 6 + 2) = rd_.ee_[i].cs_x_length;
                A_const_a(i * const_num + 3, i * 6 + 4) = -1;
            }

            // std::cout<<"0"<<std::endl;
            Eigen::VectorXd bA = A_const_a * A_rot * (rd_.P_C - rd_.J_C_INV_T.rightCols(MODEL_DOF) * Control_Torque);

            // std::cout<<"1"<<std::endl;

            Eigen::VectorXd lbA;
            lbA.setConstant(rd_.contact_index * const_num, -1000);

            // std::cout<<"2"<<std::endl;

            qp_torque_contact_.UpdateSubjectToAx(A_const_a * A_rot * rd_.J_C_INV_T.rightCols(MODEL_DOF) * NwJw, lbA - bA, bA);

            // std::cout << "3" << std::endl;

            // qp_torque_contact_.UpdateSubjectToAx()

            // std::cout<<"6"<<std::endl;
            Eigen::VectorXd qp_result;

            qp_torque_contact_.SolveQPoases(100, qp_result);

            return NwJw * qp_result;
        }
    }

    VectorQd TaskControlTorqueQP(RobotData &rd_, VectorXd f_star, bool init)
    {

        /*
        Task Control Torque Calculation with QP


        Task Space Dynamics : J_task_inv_T * (S_k)^T * Task_torque = Lambda * fstar + p

        p = J_task_inv_T * g

        Contact Force Dynamics : Fc = J_c_inv_T * Control_torque - p_c

        J_c_inv_T = lambda_c * J_v * A_inv

        p_c = J_c_inv_T * g;


        QP Formulation :

            min qacc^T * A * qacc

            subject to J_task_inv_T * (S_k)^T * task_torque = lambda * fstar

            variable size : MODEL_DOF
            constraint size : task_dof

            in qpoases form,

            min   0.5 * x' * H * x + x' * g
            x

            lbA < A x < ubA
            lb < x < ub

            qacc = A_inv * N_c * (s_k)^T * Torque_task; //q acceleration by task_torque //except gravity effect

            S_k : [0_(dof x 6) I_(dof x dof)]

            S_k can be made with :
                Eigen::Matrix<double, MODEL_DOF, MODEL_DOF +6> S_k;
                S_k.setZero();
                S_k.segment(0,6,MODEL_DOF,MODEL_DOF).setIdentity();
            or
                S_k -> .rightCols(MODEL_DOF);
                s_k.transpose() -> . bottomRows(MODEL_DOF);

            H = (A_inv * N_c * (s_k)^T)^T * A * A_inv * N_c * (S_k)^T;

            A = J_task_inv_T * (S_k)^T

            lbA = lambda * fstar
            ubA = lambda * fstar
        */

        int task_dof = rd_.J_task.rows();
        // rd_.J_task = J_task;
        rd_.J_task_T = rd_.J_task.transpose(); // MODEL_DOF_VIRTUAL * task_dof

        // Task Control Torque;
        rd_.J_task_T = rd_.J_task.transpose();

        rd_.lambda_inv = rd_.J_task * rd_.A_inv_ * rd_.N_C * rd_.J_task_T;

        rd_.lambda = rd_.lambda_inv.llt().solve(MatrixXd::Identity(task_dof, task_dof)); // task_dof * task_dof

        rd_.J_task_inv_T = rd_.lambda * rd_.J_task * rd_.A_inv_ * rd_.N_C; // task_dof * MODEL_DOF_VIRTUAL

        int var_size = MODEL_DOF;

        int constraint_size = task_dof;

        rd_.J_task_inv_T.rightCols(MODEL_DOF);

        static CQuadraticProgram qp_torque_;

        if (init)
            qp_torque_.InitializeProblemSize(var_size, task_dof);

        Eigen::Matrix<double, MODEL_DOF, MODEL_DOF> H_;

        // rd_.J_task_inv_T.rightCols(MODEL_DOF)

        H_ = (rd_.A_inv_ * rd_.N_C.rightCols(MODEL_DOF)).transpose() * rd_.N_C.rightCols(MODEL_DOF);

        Eigen::MatrixXd A_qp;

        A_qp.setZero(task_dof, task_dof);

        A_qp = rd_.J_task_inv_T.rightCols(MODEL_DOF);

        VectorXd lba, uba;

        lba = rd_.lambda * f_star;
        uba = rd_.lambda * f_star;

        VectorXd g_;
        g_.setZero(MODEL_DOF);

        VectorXd lb, ub;

        lb.setZero(MODEL_DOF);
        ub.setZero(MODEL_DOF);

        /*
        ContactForce Constraint

        Contact Force : COM vector modified axis base,

        */
        Vector3d cpos_l, cpos_r;

        cpos_l = rd_.link_[FL_LOWER_LEG].rotm.transpose() * (rd_.link_[COM_id].xpos - rd_.link_[FL_LOWER_LEG].xpos);
        cpos_r = rd_.link_[FR_LOWER_LEG].rotm.transpose() * (rd_.link_[COM_id].xpos - rd_.link_[FR_LOWER_LEG].xpos);

        Matrix3d crot_l, crot_r;

        crot_r = DyrosMath::rotateWithX(-atan(cpos_r(1) / cpos_r(2))) * DyrosMath::rotateWithY(atan(cpos_r(0) / sqrt(cpos_r(1) * cpos_r(1) + cpos_r(2) * cpos_r(2))));

        crot_l = DyrosMath::rotateWithX(-atan(cpos_l(1) / cpos_l(2))) * DyrosMath::rotateWithY(atan(cpos_l(0) / sqrt(cpos_l(1) * cpos_l(1) + cpos_l(2) * cpos_l(2))));

        Vector3d cres_l, cres_r;

        cres_l = crot_l.transpose() * cpos_l;

        cres_r = crot_r.transpose() * cpos_r;

        Vector12d ContactForce;
        Eigen::Matrix<double, 12, 12> RotM_;

        RotM_.setZero();

        RotM_.block(0, 0, 3, 3) = crot_l.transpose() * rd_.link_[FL_LOWER_LEG].rotm.transpose();
        RotM_.block(3, 3, 3, 3) = crot_l.transpose() * rd_.link_[FL_LOWER_LEG].rotm.transpose();

        RotM_.block(6, 6, 3, 3) = crot_r.transpose() * rd_.link_[FR_LOWER_LEG].rotm.transpose();
        RotM_.block(9, 9, 3, 3) = crot_r.transpose() * rd_.link_[FR_LOWER_LEG].rotm.transpose();

        Vector12d COM_Rel_ContactForce;

        COM_Rel_ContactForce = RotM_ * ContactForce;

        Eigen::Matrix<double, 12, 12> Mod_;

        Mod_.setIdentity();

        Mod_(2, 2) = 0;
        Mod_(8, 8) = 0;

        Eigen::Matrix<double, 12, 12> Res;

        Res = RotM_.transpose() * Mod_.transpose() * Mod_ * RotM_;

        // std::cout << Res << std::endl;

        qp_torque_.EnableEqualityCondition(0.001);
        qp_torque_.UpdateMinProblem(H_, g_);
        qp_torque_.UpdateSubjectToAx(A_qp, lba, uba);

        for (int i = 0; i < MODEL_DOF; i++)
        {
            lb(i) = -1000 / NM2CNT[i] - rd_.torque_grav[i];

            ub(i) = 1000 / NM2CNT[i] - rd_.torque_grav[i];
        }

        qp_torque_.UpdateSubjectToX(lb, ub);

        VectorXd qp_result;

        qp_torque_.SolveQPoases(10, qp_result);

        return qp_result;
    }

    void ForceRedistributionTwoContactMod2(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d &ResultantForce, Eigen::Vector12d &ForceRedistribution, double &eta)
    {
        Eigen::Matrix6x12d W;
        W.setZero();

        W.block(0, 0, 6, 6).setIdentity();
        W.block(0, 6, 6, 6).setIdentity();
        W.block(3, 0, 3, 3) = DyrosMath::skm(P1);
        W.block(3, 6, 3, 3) = DyrosMath::skm(P2);

        ResultantForce = W * F12; // F1F2;

        double eta_lb = 1.0 - eta_cust;
        double eta_ub = eta_cust;
        // printf("1 lb %f ub %f\n",eta_lb,eta_ub);
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////
        // boundary of eta Mx, A*eta + B < 0
        double A = (P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2);
        double B = ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2);
        double C = ratio_y * footwidth / 2.0 * abs(ResultantForce(2));
        double a = A * A;
        double b = 2.0 * A * B;
        double c = B * B - C * C;
        double sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        double sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        if (sol_eta1 > sol_eta2) // sol_eta1 ÀÌ upper boundary
        {
            if (sol_eta1 < eta_ub)
            {
                eta_ub = sol_eta1;
            }

            if (sol_eta2 > eta_lb)
            {
                eta_lb = sol_eta2;
            }
        }
        else // sol_eta2 ÀÌ upper boundary
        {
            if (sol_eta2 < eta_ub)
            {
                eta_ub = sol_eta2;
            }

            if (sol_eta1 > eta_lb)
            {
                eta_lb = sol_eta1;
            }
        }

        // printf("3 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////
        // boundary of eta My, A*eta + B < 0
        A = -(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2);
        B = ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2);
        C = ratio_x * footlength / 2.0 * abs(ResultantForce(2));
        a = A * A;
        b = 2.0 * A * B;
        c = B * B - C * C;
        sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        if (sol_eta1 > sol_eta2) // sol_eta1 ÀÌ upper boundary
        {
            if (sol_eta1 < eta_ub)
                eta_ub = sol_eta1;

            if (sol_eta2 > eta_lb)
                eta_lb = sol_eta2;
        }
        else // sol_eta2 ÀÌ upper boundary
        {
            if (sol_eta2 < eta_ub)
                eta_ub = sol_eta2;

            if (sol_eta1 > eta_lb)
                eta_lb = sol_eta1;
        }

        // printf("5 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////
        // boundary of eta Mz, (A^2-C^2)*eta^2 + 2*A*B*eta + B^2 < 0
        A = -(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0);
        B = ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1);
        C = staticFrictionCoeff * abs(ResultantForce(2));
        a = A * A;
        b = 2.0 * A * B;
        c = B * B - C * C;
        sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        if (sol_eta1 > sol_eta2) // sol_eta1 ÀÌ upper boundary
        {
            if (sol_eta1 < eta_ub)
                eta_ub = sol_eta1;
            if (sol_eta2 > eta_lb)
                eta_lb = sol_eta2;
        }
        else // sol_eta2 ÀÌ upper boundary
        {
            if (sol_eta2 < eta_ub)
                eta_ub = sol_eta2;
            if (sol_eta1 > eta_lb)
                eta_lb = sol_eta1;
        }
        // printf("6 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);

        double eta_s = (-ResultantForce(3) - P2(2) * ResultantForce(1) + P2(1) * ResultantForce(2)) / ((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2));

        eta = eta_s;
        if (eta_s > eta_ub)
            eta = eta_ub;
        else if (eta_s < eta_lb)
            eta = eta_lb;

        if ((eta > eta_cust) || (eta < 1.0 - eta_cust))
            eta = 0.5;

        ForceRedistribution(0) = eta * ResultantForce(0);
        ForceRedistribution(1) = eta * ResultantForce(1);
        ForceRedistribution(2) = eta * ResultantForce(2);
        ForceRedistribution(3) = ((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2)) * eta * eta + (ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2)) * eta;
        ForceRedistribution(4) = (-(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2)) * eta * eta + (ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2)) * eta;
        ForceRedistribution(5) = (-(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0)) * eta * eta + (ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1)) * eta;
        ForceRedistribution(6) = (1.0 - eta) * ResultantForce(0);
        ForceRedistribution(7) = (1.0 - eta) * ResultantForce(1);
        ForceRedistribution(8) = (1.0 - eta) * ResultantForce(2);
        ForceRedistribution(9) = (1.0 - eta) * (((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2)) * eta + (ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2)));
        ForceRedistribution(10) = (1.0 - eta) * ((-(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2)) * eta + (ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2)));
        ForceRedistribution(11) = (1.0 - eta) * ((-(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0)) * eta + (ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1)));
        // ForceRedistribution(9) = (1.0-eta)/eta*ForceRedistribution(3);
        // ForceRedistribution(10) = (1.0-eta)/eta*ForceRedistribution(4);
        // ForceRedistribution(11) = (1.0-eta)/eta*ForceRedistribution(5);
    }
    VectorQd ContactForceRedistributionTorqueWalking(RobotData &Robot, VectorQd command_torque, double eta, double ratio, int supportFoot)
    {

        int contact_dof_ = Robot.J_C.rows();

        if (contact_dof_ == 12)
        {
            Vector12d ContactForce_ = Robot.J_C_INV_T.rightCols(MODEL_DOF) * command_torque - Robot.P_C;

            Vector3d P1_ = Robot.ee_[0].xpos_contact - Robot.link_[COM_id].xpos;
            Vector3d P2_ = Robot.ee_[1].xpos_contact - Robot.link_[COM_id].xpos;

            Matrix3d Rotyaw = DyrosMath::rotateWithZ(-Robot.yaw);

            Eigen::Matrix<double, 12, 12> force_rot_yaw;
            force_rot_yaw.setZero();
            for (int i = 0; i < 4; i++)
            {
                force_rot_yaw.block(i * 3, i * 3, 3, 3) = Rotyaw;
            }

            Vector6d ResultantForce_;
            ResultantForce_.setZero();

            Vector12d ResultRedistribution_;
            ResultRedistribution_.setZero();

            Vector12d F12 = force_rot_yaw * ContactForce_;

            double eta_cust = 0.99;
            double foot_length = 0.26;
            double foot_width = 0.1;

            ForceRedistributionTwoContactMod2(0.99, foot_length, foot_width, 1.0, 0.9, 0.9, Rotyaw * P1_, Rotyaw * P2_, F12, ResultantForce_, ResultRedistribution_, eta);
            Robot.fc_redist_ = force_rot_yaw.transpose() * ResultRedistribution_;

            Vector12d desired_force;
            desired_force.setZero();

            bool right_master;

            if (supportFoot == 0)
            {
                right_master = 1.0;
            }
            else
            {
                right_master = 0.0;
            }

            if (right_master)
            {

                desired_force.segment(0, 6) = -ContactForce_.segment(0, 6) + ratio * Robot.fc_redist_.segment(0, 6);
                Robot.torque_contact = Robot.qr_V2.transpose() * (Robot.J_C_INV_T.rightCols(MODEL_DOF).topRows(6) * Robot.qr_V2.transpose()).inverse() * desired_force.segment(0, 6);
            }
            else
            {
                desired_force.segment(6, 6) = -ContactForce_.segment(6, 6) + ratio * Robot.fc_redist_.segment(6, 6);
                Robot.torque_contact = Robot.qr_V2.transpose() * (Robot.J_C_INV_T.rightCols(MODEL_DOF).bottomRows(6) * Robot.qr_V2.transpose()).inverse() * desired_force.segment(6, 6);
            }
        }
        else
        {
            Robot.torque_contact.setZero();
        }

        return Robot.torque_contact + command_torque;
    }

    VectorQd ContactForceRedistributionTorque(RobotData &Robot, VectorQd command_torque, double eta)
    {
        int contact_dof_ = Robot.J_C.rows();

        if (contact_dof_ == 12)
        {
            Vector12d ContactForce_ = Robot.J_C_INV_T.rightCols(MODEL_DOF) * command_torque - Robot.P_C;

            Vector3d P1_ = Robot.ee_[0].xpos_contact - Robot.link_[COM_id].xpos;
            Vector3d P2_ = Robot.ee_[1].xpos_contact - Robot.link_[COM_id].xpos;

            Matrix3d Rotyaw = DyrosMath::rotateWithZ(-Robot.yaw);

            Eigen::Matrix<double, 12, 12> force_rot_yaw;
            force_rot_yaw.setZero();
            for (int i = 0; i < 4; i++)
            {
                force_rot_yaw.block(i * 3, i * 3, 3, 3) = Rotyaw;
            }

            Vector6d ResultantForce_;
            ResultantForce_.setZero();

            Vector12d ResultRedistribution_;
            ResultRedistribution_.setZero();

            Vector12d F12 = force_rot_yaw * ContactForce_;

            double eta_cust = 0.99;
            double foot_length = 0.26;
            double foot_width = 0.1;

            ForceRedistributionTwoContactMod2(0.99, foot_length, foot_width, 1.0, 0.9, 0.9, Rotyaw * P1_, Rotyaw * P2_, F12, ResultantForce_, ResultRedistribution_, eta);
            Robot.fc_redist_ = force_rot_yaw.transpose() * ResultRedistribution_;

            Vector12d desired_force;
            desired_force.setZero();

            desired_force.segment(6, 6) = -ContactForce_.segment(6, 6) + Robot.fc_redist_.segment(6, 6);
            Robot.torque_contact = Robot.qr_V2.transpose() * (Robot.J_C_INV_T.rightCols(MODEL_DOF).bottomRows(6) * Robot.qr_V2.transpose()).inverse() * desired_force.segment(6, 6);
        }
        else
        {
            Robot.torque_contact.setZero();
        }

        return Robot.torque_contact + command_torque;
    }

    //////////////////////////JS functions
    Vector3d GetFstarPosJS(LinkData &link_, Eigen::Vector3d Desired_pos, Eigen::Vector3d Current_pos, Eigen::Vector3d Desired_vel, Eigen::Vector3d Current_vel)
    {
        Vector3d fstar;

        for (int i = 0; i < 3; i++)
            fstar(i) = link_.pos_p_gain(i) * (Desired_pos(i) - Current_pos(i)) + link_.pos_d_gain(i) * (Desired_vel(i) - Current_vel(i));

        return fstar;
    }

    Vector3d GetFstarRotJS(LinkData &link_)
    {
        Vector3d fstar;

        Vector3d ad;

        ad = DyrosMath::getPhi(link_.rotm, link_.r_traj);

        for (int i = 0; i < 3; i++)
            fstar(i) = -link_.rot_p_gain(i) * ad(i) + link_.rot_d_gain(i) * (link_.w_traj(i) - link_.w(i));

        // std::cout << ad.transpose() << "\t" << (link_.w_traj - link_.w).transpose() << std::endl;
        // std::cout << DyrosMath::rot2Euler_tf(link_.rotm).transpose() << "\t" << DyrosMath::rot2Euler_tf(link_.r_traj) << std::endl;
        // return link_.rot_p_gain.cwiseProduct(DyrosMath::getPhi(link_.rotm, link_.r_traj)); // + link_.rot_d_gain.cwiseProduct(link_.w_traj - link_.w);

        return fstar;
    }

    VectorQd TaskControlTorqueMotor(RobotData &rd_, VectorXd f_star, Eigen::MatrixVVd B, Eigen::MatrixVVd B_inv)
    {
        int task_dof = rd_.J_task.rows();
        // rd_.J_task = J_task;
        rd_.J_task_T = rd_.J_task.transpose();

        // Eigen::MatrixVVd B;
        // Eigen::MatrixVVd B_inv;
        Eigen::MatrixXd lambda_m;
        Eigen::MatrixXd lambda_m_inv;
        Eigen::MatrixXd J_task_inv_m_T;
        Eigen::MatrixXd Q_m, Q_m_T, Q_m_temp, Q_m_temp_inv;

        lambda_m.resize(task_dof, task_dof);
        lambda_m_inv.resize(task_dof, task_dof);
        J_task_inv_m_T.resize(task_dof, MODEL_DOF_VIRTUAL);
        Q_m.resize(task_dof, MODEL_DOF);
        Q_m_T.resize(MODEL_DOF, task_dof);
        Q_m_temp.resize(task_dof, task_dof);
        Q_m_temp_inv.resize(task_dof, task_dof);

        lambda_m_inv = rd_.J_task * B_inv * rd_.N_C * rd_.J_task_T;

        lambda_m = lambda_m_inv.llt().solve(MatrixXd::Identity(task_dof, task_dof));

        J_task_inv_m_T = lambda_m * rd_.J_task * B_inv * rd_.N_C;

        // cout << "Lambda\n"<< lambda_m.block(0,0,6,6) << endl << endl;

        Q_m = J_task_inv_m_T.rightCols(MODEL_DOF);
        Q_m_T = Q_m.transpose();

        // std::cout<<"1"<<std::endl;
        Q_m_temp = Q_m * rd_.W_inv * Q_m_T;
        // std::cout<<"2"<<std::endl;

        Q_m_temp_inv = DyrosMath::pinv_QR(Q_m_temp);

        // Eigen::MatrixXd JkT;
        // JkT=rd_.W_inv * Q_m_T * Q_m_temp_inv;
        // cout << "J_k\n"<< JkT.transpose().block(0,0,6,6) << endl << endl;
        return rd_.W_inv * (Q_m_T * (Q_m_temp_inv * (lambda_m * f_star)));
    }

    VectorQd TaskControlTorqueExtra(RobotData &rd_, VectorXd f_star, Eigen::MatrixVVd B, Eigen::MatrixVVd B_inv)
    {
        // int task_dof = 1;
        int task_dof = rd_.J_task.rows();
        rd_.J_task_T = rd_.J_task.transpose();

        // Eigen::MatrixXd Jtask_z, Jtask_z_T;
        // Jtask_z.resize(1,rd_.J_task.cols());
        // Jtask_z_T.resize(rd_.J_task.cols(),1);
        // Jtask_z = rd_.J_task.block(2,0,1,rd_.J_task.cols());
        // Jtask_z_T = Jtask_z.transpose();

        // Task Control Torque;
        Eigen::MatrixXd lambda_m;
        Eigen::MatrixXd lambda_m_inv;
        Eigen::MatrixXd J_task_inv_m_T;
        Eigen::MatrixXd Q_e, Q_e_T, Q_e_temp, Q_e_temp_inv;

        lambda_m.resize(task_dof, task_dof);
        lambda_m_inv.resize(task_dof, task_dof);
        J_task_inv_m_T.resize(task_dof, MODEL_DOF_VIRTUAL);

        Eigen::MatrixXd Si;
        Eigen::MatrixXd SiT;
        Eigen::MatrixXd SikT;

        // knee
        Si.setZero(2, MODEL_DOF + 6);
        SiT.setZero(MODEL_DOF + 6, 2);
        SikT.setZero(MODEL_DOF, 2);
        Si(0, 9) = 1.0;
        Si(1, 15) = 1.0;
        SiT = Si.transpose();
        SikT(3, 0) = 1.0;
        SikT(9, 1) = 1.0;

        // knee & hip
        // Si.setZero(8, MODEL_DOF + 6);
        // SiT.setZero(MODEL_DOF + 6, 8);
        // SikT.setZero(MODEL_DOF, 8);
        // Si(0, 6) = 1.0;
        // Si(1, 7) = 1.0;
        // Si(2, 8) = 1.0;
        // Si(3, 9) = 1.0;//무릎관절

        // Si(4, 12) = 1.0;
        // Si(5, 13) = 1.0;
        // Si(6, 14) = 1.0;
        // Si(7, 15) = 1.0;//무릎관절
        // SiT = Si.transpose();
        // SikT(0, 0) = 1.0;
        // SikT(1, 1) = 1.0;
        // SikT(2, 2) = 1.0;
        // SikT(3, 3) = 1.0;
        // SikT(6, 4) = 1.0;
        // SikT(7, 5) = 1.0;
        // SikT(8, 6) = 1.0;
        // SikT(9, 7) = 1.0;

        B_inv = B.llt().solve(MatrixXd::Identity(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL));

        lambda_m_inv = rd_.J_task * B_inv * rd_.N_C * rd_.J_task_T;
        // lambda_m_inv = Jtask_z * B_inv * rd_.N_C * Jtask_z_T;

        lambda_m = lambda_m_inv.llt().solve(MatrixXd::Identity(task_dof, task_dof));

        J_task_inv_m_T = lambda_m * rd_.J_task * B_inv * rd_.N_C;
        // J_task_inv_m_T = lambda_m * Jtask_z * B_inv * rd_.N_C;

        Eigen::MatrixXd Wi;
        Eigen::MatrixXd Wi_inv;

        Wi = Si * rd_.A_inv_ * rd_.N_C * SiT; // 2 types for w matrix
        Wi_inv = DyrosMath::pinv_QR(Wi);

        Q_e = J_task_inv_m_T * SiT;
        Q_e_T = Q_e.transpose();

        // std::cout<<"1"<<std::endl;
        Q_e_temp = Q_e * Wi_inv * Q_e_T;
        // std::cout<<"2"<<std::endl;

        Q_e_temp_inv = DyrosMath::pinv_QR(Q_e_temp);
        // DyrosMath::dc_inv_QR(rd_.J_task)
        return SikT * Wi_inv * (Q_e_T * (Q_e_temp_inv * (lambda_m * f_star)));
    }

    VectorXd Forcecompute(RobotData &rd_, VectorXd torque, Eigen::MatrixVVd B, Eigen::MatrixVVd B_inv)
    {
        int task_dof = rd_.J_task.rows();
        // rd_.J_task = J_task;
        rd_.J_task_T = rd_.J_task.transpose();

        // Eigen::MatrixVVd B;
        // Eigen::MatrixVVd B_inv;
        Eigen::VectorXd vTorque;
        Eigen::MatrixXd lambda_m;
        Eigen::MatrixXd lambda_m_inv;
        Eigen::MatrixXd J_task_inv_m_T;
        Eigen::MatrixXd Q_m, Q_m_T, Q_m_temp, Q_m_temp_inv;

        vTorque.resize(MODEL_DOF_VIRTUAL);
        lambda_m.resize(task_dof, task_dof);
        lambda_m_inv.resize(task_dof, task_dof);
        J_task_inv_m_T.resize(task_dof, MODEL_DOF_VIRTUAL);
        Q_m.resize(task_dof, MODEL_DOF);
        Q_m_T.resize(MODEL_DOF, task_dof);
        Q_m_temp.resize(task_dof, task_dof);
        Q_m_temp_inv.resize(task_dof, task_dof);

        vTorque.setZero();

        lambda_m_inv = rd_.J_task * B_inv * rd_.N_C * rd_.J_task_T;
        lambda_m = lambda_m_inv.llt().solve(MatrixXd::Identity(task_dof, task_dof));
        J_task_inv_m_T = lambda_m * rd_.J_task * B_inv * rd_.N_C;

        Eigen::VectorXd value;
        value.resize(8);

        Eigen::VectorXd force;
        vTorque(6 + 0) = torque(0);
        force = J_task_inv_m_T * vTorque;
        value(0) = force(2);

        vTorque(6 + 0) = 0.0;
        vTorque(6 + 1) = torque(1);
        force = J_task_inv_m_T * vTorque;
        value(1) = force(2);

        vTorque(6 + 1) = 0.0;
        vTorque(6 + 2) = torque(2);
        force = J_task_inv_m_T * vTorque;
        value(2) = force(2);

        vTorque(6 + 2) = 0.0;
        vTorque(6 + 3) = torque(3);
        force = J_task_inv_m_T * vTorque;
        value(3) = force(2);

        vTorque(6 + 3) = 0.0;
        vTorque(6 + 6) = torque(6);
        force = J_task_inv_m_T * vTorque;
        value(4) = force(2);

        vTorque(6 + 6) = 0.0;
        vTorque(6 + 7) = torque(7);
        force = J_task_inv_m_T * vTorque;
        value(5) = force(2);

        vTorque(6 + 7) = 0.0;
        vTorque(6 + 8) = torque(8);
        force = J_task_inv_m_T * vTorque;
        value(6) = force(2);

        vTorque(6 + 8) = 0.0;
        vTorque(6 + 9) = torque(9);
        force = J_task_inv_m_T * vTorque;
        value(7) = force(2);

        return value;
    }

    VectorQd Vgravitytoruqe(RobotData &rd_)
    {
        rd_.G.setZero();
        for (int i = 0; i < MODEL_DOF + 1; i++)
            rd_.G -= rd_.link_[i].jac_com.cast<double>().topRows(3).transpose() * rd_.link_[i].mass * rd_.grav_ref;

        VectorQd grav;
        grav = rd_.G.segment(6, MODEL_DOF);
        rd_.torque_grav = rd_.W_inv * (rd_.A_inv_.bottomRows(MODEL_DOF) * (rd_.N_C * rd_.G));
        rd_.P_C = rd_.J_C_INV_T * rd_.G;

        return grav;
    }

    VectorQd newtasktorque(RobotData &rd_, VectorXd f_star, Eigen::MatrixVVd B, Eigen::MatrixVVd B_inv, VectorXd Fc)
    {
        VectorQd zero;
        zero.setZero();
        int task_dof = rd_.J_task.rows();
        rd_.J_task_T = rd_.J_task.transpose();

        // Eigen::MatrixVVd B;
        // Eigen::MatrixVVd B_inv;
        Eigen::MatrixXd lambda_m;
        Eigen::MatrixXd lambda_m_inv;
        Eigen::MatrixXd J_task_inv_m_T;
        Eigen::MatrixXd Q_m, Q_m_T, Q_m_temp, Q_m_temp_inv;

        lambda_m.resize(task_dof, task_dof);
        lambda_m_inv.resize(task_dof, task_dof);
        J_task_inv_m_T.resize(task_dof, MODEL_DOF_VIRTUAL);
        Q_m.resize(task_dof, MODEL_DOF);
        Q_m_T.resize(MODEL_DOF, task_dof);
        Q_m_temp.resize(task_dof, task_dof);
        Q_m_temp_inv.resize(task_dof, task_dof);

        lambda_m_inv = rd_.J_task * B_inv * rd_.J_task_T;

        lambda_m = lambda_m_inv.llt().solve(MatrixXd::Identity(task_dof, task_dof));

        J_task_inv_m_T = lambda_m * rd_.J_task * B_inv;

        Eigen::MatrixXd Wi;
        Eigen::MatrixXd Wi_inv;

        Wi = rd_.A_inv_.block(6, 6, MODEL_DOF, MODEL_DOF); // 2 types for w matrix
        Wi_inv = DyrosMath::pinv_QR(Wi);

        Q_m = J_task_inv_m_T.rightCols(MODEL_DOF);
        Q_m_T = Q_m.transpose();
        Q_m_temp = Q_m * Wi_inv * Q_m_T;

        // cout << lambda_m_inv << endl << endl;

        Q_m_temp_inv = DyrosMath::pinv_QR(Q_m_temp);

        return Wi_inv * (Q_m_T * (Q_m_temp_inv * (lambda_m * f_star + J_task_inv_m_T * rd_.J_C.transpose() * Fc)));
    }

    VectorQd ContactForceRedistributionTorqueJS(RobotData &Robot, VectorQd command_torque, double ratio, int supportFoot) //
    {

        int contact_dof_ = Robot.J_C.rows();

        if (contact_dof_ == 12)
        {
            Vector12d ContactForce_ = Robot.J_C_INV_T.rightCols(MODEL_DOF) * command_torque - Robot.P_C;

            Vector3d P1_ = Robot.ee_[0].xpos_contact - Robot.link_[COM_id].xpos;
            Vector3d P2_ = Robot.ee_[1].xpos_contact - Robot.link_[COM_id].xpos;

            Matrix3d Rotyaw = DyrosMath::rotateWithZ(-Robot.yaw);

            Eigen::Matrix<double, 12, 12> force_rot_yaw;
            force_rot_yaw.setZero();
            for (int i = 0; i < 4; i++)
            {
                force_rot_yaw.block(i * 3, i * 3, 3, 3) = Rotyaw;
            }

            Vector6d ResultantForce_;
            ResultantForce_.setZero();

            Vector12d ResultRedistribution_;
            ResultRedistribution_.setZero();

            Vector12d F12 = force_rot_yaw * ContactForce_;

            double eta_cust = 0.99;
            double foot_length = 0.26;
            double foot_width = 0.1;
            double eta;

            ForceRedistributionTwoContactMod2(0.99, foot_length, foot_width, 1.0, 0.9, 0.9, Rotyaw * P1_, Rotyaw * P2_, F12, ResultantForce_, ResultRedistribution_, eta);
            Robot.fc_redist_ = force_rot_yaw.transpose() * ResultRedistribution_;

            Vector12d desired_force;
            desired_force.setZero();

            bool right_master;

            if (supportFoot == 0)
            {
                right_master = 1.0;
            }
            else
            {
                right_master = 0.0;
            }

            if (right_master)
            {

                desired_force.segment(0, 6) = -ContactForce_.segment(0, 6) + ratio * Robot.fc_redist_.segment(0, 6);
                Robot.torque_contact = Robot.qr_V2.transpose() * (Robot.J_C_INV_T.rightCols(MODEL_DOF).topRows(6) * Robot.qr_V2.transpose()).inverse() * desired_force.segment(0, 6);
            }
            else
            {
                desired_force.segment(6, 6) = -ContactForce_.segment(6, 6) + ratio * Robot.fc_redist_.segment(6, 6);
                Robot.torque_contact = Robot.qr_V2.transpose() * (Robot.J_C_INV_T.rightCols(MODEL_DOF).bottomRows(6) * Robot.qr_V2.transpose()).inverse() * desired_force.segment(6, 6);
            }
        }
        else
        {
            Robot.torque_contact.setZero();
        }

        return Robot.torque_contact;
    }

    Vector3d GetZMPpos_from_ContactForce(RobotData &Robot, VectorXd ContactForce)
    {

        Vector3d zmp_pos;
        zmp_pos.setZero();

        double total_fz = 0;

        for (int i = 0; i < Robot.contact_index; i++)
            total_fz = total_fz + ContactForce(2 + i * 6);

        for (int i = 0; i < Robot.contact_index; i++)
        {
            Robot.ee_[Robot.ee_idx[i]].zmp(0) = Robot.ee_[Robot.ee_idx[i]].xpos_contact(0) + (-ContactForce(i * 6 + 4) / ContactForce(i * 6 + 2));
            Robot.ee_[Robot.ee_idx[i]].zmp(1) = Robot.ee_[Robot.ee_idx[i]].xpos_contact(1) + (ContactForce(i * 6 + 3) / ContactForce(i * 6 + 2));
            Robot.ee_[Robot.ee_idx[i]].zmp(2) = Robot.ee_[Robot.ee_idx[i]].xpos_contact(2);

            zmp_pos = zmp_pos + Robot.ee_[Robot.ee_idx[i]].zmp * ContactForce(i * 6 + 2) / total_fz;
        }

        // Vector3d zmp_pos;
        // Vector3d P_;
        // zmp_pos.setZero();
        // P_.setZero();

        // Vector3d zmp_r, zmp_l;

        // zmp_l(0) = Robot.ee_[0].xpos_contact(0) + (-Robot.ContactForce(4) - Robot.ContactForce(0) * (Robot.ee_[0].xpos_contact(2) - Robot.ee_[0].xpos_contact(2))) / Robot.ContactForce(2);
        // zmp_l(1) = Robot.ee_[0].xpos_contact(1) + (Robot.ContactForce(3) - Robot.ContactForce(1) * (Robot.ee_[0].xpos_contact(2) - Robot.ee_[0].xpos_contact(2))) / Robot.ContactForce(2);

        // zmp_r(0) = Robot.ee_[1].xpos_contact(0) + (-Robot.ContactForce(6 + 4) - Robot.ContactForce(6) * (Robot.ee_[1].xpos_contact(2) - Robot.ee_[1].xpos_contact(2))) / Robot.ContactForce(6 + 2);
        // zmp_r(1) = Robot.ee_[1].xpos_contact(1) + (Robot.ContactForce(6 + 3) - Robot.ContactForce(6 + 1) * (Robot.ee_[1].xpos_contact(2) - Robot.ee_[1].xpos_contact(2))) / Robot.ContactForce(6 + 2);

        // if (Robot.ee_[0].contact && Robot.ee_[1].contact)
        // {
        //     zmp_pos(0) = (zmp_l(0) * Robot.ContactForce(2) + zmp_r(0) * Robot.ContactForce(8)) / (Robot.ContactForce(2) + Robot.ContactForce(8));
        //     zmp_pos(1) = (zmp_l(1) * Robot.ContactForce(2) + zmp_r(1) * Robot.ContactForce(8)) / (Robot.ContactForce(2) + Robot.ContactForce(8));
        // }
        // else if (Robot.ee_[0].contact) // left contact
        // {
        //     zmp_pos(0) = zmp_l(0);
        //     zmp_pos(1) = zmp_l(1);
        // }
        // else if (Robot.ee_[1].contact) // right contact
        // {
        //     zmp_pos(0) = zmp_r(0);
        //     zmp_pos(1) = zmp_r(1);
        // }
        return (zmp_pos);
    }

    Vector3d GetZMPpos_fromFT(RobotData &Robot) // Only Left/Right
    {
        Vector3d zmp_pos;
        Vector3d P_;
        zmp_pos.setZero();
        P_.setZero();

        Vector3d zmp_r, zmp_l;

        zmp_l(0) = Robot.ee_[0].xpos_contact(0) + (-Robot.LF_CF_FT(4) - Robot.LF_CF_FT(0) * (Robot.ee_[0].xpos_contact(2) - Robot.ee_[0].xpos_contact(2))) / Robot.LF_CF_FT(2);
        zmp_l(1) = Robot.ee_[0].xpos_contact(1) + (Robot.LF_CF_FT(3) - Robot.LF_CF_FT(1) * (Robot.ee_[0].xpos_contact(2) - Robot.ee_[0].xpos_contact(2))) / Robot.LF_CF_FT(2);

        zmp_r(0) = Robot.ee_[1].xpos_contact(0) + (-Robot.RF_CF_FT(4) - Robot.RF_CF_FT(0) * (Robot.ee_[1].xpos_contact(2) - Robot.ee_[1].xpos_contact(2))) / Robot.RF_CF_FT(2);
        zmp_r(1) = Robot.ee_[1].xpos_contact(1) + (Robot.RF_CF_FT(3) - Robot.RF_CF_FT(1) * (Robot.ee_[1].xpos_contact(2) - Robot.ee_[1].xpos_contact(2))) / Robot.RF_CF_FT(2);

        if (Robot.ee_[0].contact && Robot.ee_[1].contact)
        {
            zmp_pos(0) = (zmp_l(0) * Robot.LF_CF_FT(2) + zmp_r(0) * Robot.RF_CF_FT(2)) / (Robot.LF_CF_FT(2) + Robot.RF_CF_FT(2));
            zmp_pos(1) = (zmp_l(1) * Robot.LF_CF_FT(2) + zmp_r(1) * Robot.RF_CF_FT(2)) / (Robot.LF_CF_FT(2) + Robot.RF_CF_FT(2));
        }
        else if (Robot.ee_[0].contact) // left contact
        {
            zmp_pos(0) = zmp_l(0);
            zmp_pos(1) = zmp_l(1);
        }
        else if (Robot.ee_[1].contact) // right contact
        {
            zmp_pos(0) = zmp_r(0);
            zmp_pos(1) = zmp_r(1);
        }
        return (zmp_pos);
    }

    VectorXd getContactForce(RobotData &Robot, VectorQd command_torque)
    {
        VectorXd contactforce;
        contactforce = Robot.J_C_INV_T.rightCols(MODEL_DOF) * (command_torque)-Robot.P_C;

        return contactforce;
    }

    int TaskControlHQP(RobotData &rd_, TaskSpace &ts_, CQuadraticProgram &qp_h_, const VectorQd &torque_prev, const MatrixXd &prev_task_null_, bool init_trigger)
    {
        if (TaskControlHQP(rd_, qp_h_, ts_.J_task_, ts_.J_kt_, ts_.f_star_, ts_.Lambda_task_, torque_prev, prev_task_null_, ts_.f_star_qp_, ts_.contact_qp_, init_trigger))
        {
            ts_.torque_h_ = ts_.J_kt_ * ts_.Lambda_task_ * (ts_.f_star_ + ts_.f_star_qp_);
            return true;
        }
        else
        {
            return false;
        }
    }

    int TaskControlHQP(RobotData &rd_, CQuadraticProgram &qp_h_, const MatrixXd &J_task, const MatrixXd &Jkt_, const VectorXd &fstar_, const MatrixXd lambda_task_, const VectorQd &torque_prev, const MatrixXd &prev_task_null_, VectorXd &fstar_result, VectorXd &contact_result, bool init_trigger_)
    {
        // return fstar & contact force;
        int task_dof = fstar_.size();          // size of task
        int contact_index = rd_.contact_index; // size of contact link
        int total_contact_dof = 0;             // total size of contact dof
        int contact_dof = -6;                  // total_contact_dof - 6, (free contact space)
        int contact_constraint_size = 0;       // size of constraint by contact
        int model_size = MODEL_DOF;            // size of joint

        int contact_constraint_number = 10;

        bool torque_limit = true;

        int torque_limit_constraint_size = 2 * model_size;

        for (int i = 0; i < contact_index; i++)
        {
            if (rd_.ee_[rd_.ee_idx[i]].contact)
            {
                total_contact_dof += 6;
                contact_constraint_size += contact_constraint_number;
            }
        }
        contact_dof += total_contact_dof;

        if (contact_dof < 0)
        {
            contact_dof = 0;
        }

        int variable_size = task_dof + contact_dof;

        if (!torque_limit)
            torque_limit_constraint_size = 0;
        int total_constraint_size = contact_constraint_size + torque_limit_constraint_size; // total contact constraint size

        MatrixXd H;
        VectorXd g;
        H.setZero(variable_size, variable_size);
        H.block(0, 0, task_dof, task_dof).setIdentity();
        g.setZero(variable_size);

        Eigen::MatrixXd A;
        Eigen::VectorXd lbA, ubA;
        A.setZero(total_constraint_size, variable_size);
        lbA.setZero(total_constraint_size);
        ubA.setZero(total_constraint_size);
        Eigen::MatrixXd Ntorque_task = prev_task_null_ * Jkt_ * lambda_task_;

        if (torque_limit)
        {
            A.block(0, 0, model_size, task_dof) = Ntorque_task;
            if (contact_dof > 0)
                A.block(0, task_dof, model_size, contact_dof) = rd_.NwJw;
            // lbA.segment(0, model_size) = -torque_limit_ - torque_prev - Ntorque_task * ts_.f_star_;
            ubA.segment(0, model_size) = rd_.torque_limit - (torque_prev + Ntorque_task * fstar_);

            A.block(model_size, 0, model_size, task_dof) = -Ntorque_task;
            if (contact_dof > 0)
                A.block(model_size, task_dof, model_size, contact_dof) = -rd_.NwJw;
            // lbA.segment(model_size, model_size) = -torque_limit_ - torque_prev - Ntorque_task * ts_.f_star_;
            ubA.segment(model_size, model_size) = rd_.torque_limit + torque_prev + Ntorque_task * fstar_;

            lbA.segment(0, torque_limit_constraint_size).setConstant(-INFTY);
        }

        Eigen::MatrixXd A_const_a;
        A_const_a.setZero(contact_constraint_size, total_contact_dof);

        Eigen::MatrixXd A_rot;
        A_rot.setZero(total_contact_dof, total_contact_dof);

        int const_idx = 0;
        int contact_idx = 0;
        for (int i = 0; i < contact_index; i++)
        {
            if (rd_.ee_[rd_.ee_idx[i]].contact)
            {
                A_rot.block(contact_idx, contact_idx, 3, 3) = rd_.ee_[rd_.ee_idx[i]].rotm.transpose();
                A_rot.block(contact_idx + 3, contact_idx + 3, 3, 3) = rd_.ee_[rd_.ee_idx[i]].rotm.transpose();

                A_const_a.block(const_idx, contact_idx, 4, 6) = rd_.ee_[rd_.ee_idx[i]].GetZMPConstMatrix();
                A_const_a.block(const_idx + 4, contact_idx, 6, 6) = rd_.ee_[rd_.ee_idx[i]].GetForceConstMatrix();

                const_idx += 10;
                contact_idx += 6;
            }
        }

        Eigen::MatrixXd Atemp = A_const_a * A_rot * rd_.J_C_INV_T.rightCols(model_size);
        // t[3] = std::chrono::steady_clock::now();
        A.block(torque_limit_constraint_size, 0, contact_constraint_size, task_dof) = -Atemp * Ntorque_task;
        if (contact_dof > 0)
            A.block(torque_limit_constraint_size, task_dof, contact_constraint_size, contact_dof) = -Atemp * rd_.NwJw;
        // t[4] = std::chrono::steady_clock::now();

        Eigen::VectorXd bA = A_const_a * (A_rot * rd_.P_C) - Atemp * (torque_prev + Ntorque_task * fstar_);
        // Eigen::VectorXd ubA_contact;
        lbA.segment(torque_limit_constraint_size, contact_constraint_size).setConstant(-INFTY);

        // lbA.segment(total_constraint_size) = -ubA_contact;
        ubA.segment(torque_limit_constraint_size, contact_constraint_size) = -bA;

        // qp_.EnableEqualityCondition(0.0001);

        VectorXd qpres;

        qpres.setZero(variable_size);

        if (qp_h_.CheckProblemSize(variable_size, total_constraint_size))
        {
            if (init_trigger_)
            {
                qp_h_.InitializeProblemSize(variable_size, total_constraint_size);
            }
        }
        else
        {
            qp_h_.InitializeProblemSize(variable_size, total_constraint_size);
        }

        qp_h_.UpdateMinProblem(H, g);
        qp_h_.UpdateSubjectToAx(A, lbA, ubA);
        qp_h_.DeleteSubjectToX();

        if (qp_h_.SolveQPoases(300, qpres))
        {
            fstar_result = qpres.segment(0, task_dof);

            contact_result = qpres.segment(task_dof, contact_dof);
            return 1;
        }
        else
        {
            std::cout << "task solve failed" << std::endl;
            // fstar_result = VectorXd::Zero(task_dof);

            return 0;
        }
    }

    int CalcContactRedistributeHQP(RobotData &rd_, CQuadraticProgram &qp_contact_, VectorXd torque_input, VectorXd &torque_output, bool init)
    {
        int contact_index = rd_.contact_index; // size of contact link
        int total_contact_dof = 0;             // size of contact dof
        int contact_dof = -6;                  // total_contact_dof - 6, (free contact space)
        int contact_constraint_size = 0;       // size of constraint by contact
        int model_size = MODEL_DOF;            // size of joints
        int torque_limit_constraint_size = 2 * model_size;

        bool torque_limit_set_ = true;

        for (int i = 0; i < contact_index; i++)
        {
            if (rd_.ee_[rd_.ee_idx[i]].contact)
            {
                total_contact_dof += 6;
                contact_constraint_size += 10;
            }
        }
        contact_dof += total_contact_dof;

        if (!torque_limit_set_)
            torque_limit_constraint_size = 0;
        int variable_number = contact_dof;                                                  // total size of qp variable
        int total_constraint_size = contact_constraint_size + torque_limit_constraint_size; // total size of constraint

        if (contact_dof > 0)
        {
            MatrixXd H, H_temp;
            VectorXd g;

            Eigen::MatrixXd crot_matrix = Eigen::MatrixXd::Zero(total_contact_dof, total_contact_dof);
            Eigen::MatrixXd RotW = Eigen::MatrixXd::Identity(total_contact_dof, total_contact_dof);
            int acc_cdof = 0;
            for (int i = 0; i < contact_index; i++)
            {
                if (rd_.ee_[rd_.ee_idx[i]].contact)
                {
                    // Vector3d vec_origin, vec_target;
                    // vec_origin = rd_.ee_[rd_.ee_idx[i]].rotm.rightCols(1);
                    // vec_target = (rd_.link_[COM_id].xpos - rd_.ee_[rd_.ee_idx[i]].xpos_contact).normalized();
                    // Matrix3d cm = AxisTransform2V(vec_origin, vec_target);

                    Matrix3d cm = Matrix3d::Identity(); // .setIdentity(); // comment this line to redistribute contact force with COM based vector

                    // if (cc_[i].contact_type_ == CONTACT_6D)
                    // {
                    crot_matrix.block(acc_cdof, acc_cdof, 3, 3) = crot_matrix.block(acc_cdof + 3, acc_cdof + 3, 3, 3) = cm.transpose() * rd_.ee_[rd_.ee_idx[i]].rotm.transpose();
                    RotW(acc_cdof + 2, acc_cdof + 2) = 0;
                    acc_cdof += 6;
                    // }
                    // else if (cc_[i].contact_type_ == CONTACT_POINT)
                    // {
                    //     crot_matrix.block(acc_cdof, acc_cdof, 3, 3) = cm.transpose() * cc_[i].rotm.transpose();
                    //     RotW(acc_cdof + 2, acc_cdof + 2) = 0;
                    //     acc_cdof += cc_[i].contact_dof_;
                    // }
                }
            }
            // rd_.J_C_INV_T.rightCols(model_size) * rd_.NwJw;
            H_temp = RotW * crot_matrix * rd_.J_C_INV_T.rightCols(model_size) * rd_.NwJw;
            H = H_temp.transpose() * H_temp;
            g = (RotW * crot_matrix * (rd_.J_C_INV_T.rightCols(model_size) * torque_input - rd_.P_C)).transpose() * H_temp;

            MatrixXd A_qp;
            VectorXd lbA, ubA;
            A_qp.setZero(total_constraint_size, variable_number);
            lbA.setZero(total_constraint_size);
            ubA.setZero(total_constraint_size);

            if (torque_limit_set_)
            {
                A_qp.block(0, 0, model_size, contact_dof) = rd_.NwJw;
                // lbA.segment(0, model_size) = -torque_limit_ - control_torque;
                ubA.segment(0, model_size) = rd_.torque_limit - torque_input;

                A_qp.block(0, 0, model_size, contact_dof) = -rd_.NwJw;
                // lbA.segment(0, model_size) = -torque_limit_ - control_torque;
                ubA.segment(0, model_size) = rd_.torque_limit + torque_input;

                lbA.segment(0, torque_limit_constraint_size).setConstant(-INFTY);
            }

            MatrixXd A_const_a;
            A_const_a.setZero(contact_constraint_size, total_contact_dof);

            MatrixXd A_rot;
            A_rot.setZero(total_contact_dof, total_contact_dof);

            int const_idx = 0;
            int contact_idx = 0;
            for (int i = 0; i < contact_index; i++)
            {
                if (rd_.ee_[rd_.ee_idx[i]].contact)
                {
                    A_rot.block(contact_idx, contact_idx, 3, 3) = rd_.ee_[rd_.ee_idx[i]].rotm.transpose(); // rd_.ee_[i].rotm.transpose();
                    A_rot.block(contact_idx + 3, contact_idx + 3, 3, 3) = rd_.ee_[rd_.ee_idx[i]].rotm.transpose();

                    A_const_a.block(const_idx, contact_idx, 4, 6) = rd_.ee_[rd_.ee_idx[i]].GetZMPConstMatrix();
                    A_const_a.block(const_idx + 4, contact_idx, 6, 6) = rd_.ee_[rd_.ee_idx[i]].GetForceConstMatrix();

                    const_idx += 10;
                    contact_idx += 6;
                }

                // specific vector on Global axis
                // [0 0 -1]T *
                // Force Constraint
                // Will be added
            }

            Eigen::MatrixXd Atemp = A_const_a * A_rot * rd_.J_C_INV_T.rightCols(model_size);
            Eigen::VectorXd bA = A_const_a * (A_rot * rd_.P_C) - Atemp * torque_input;

            A_qp.block(torque_limit_constraint_size, 0, contact_constraint_size, contact_dof) = -Atemp * rd_.NwJw;

            lbA.segment(torque_limit_constraint_size, contact_constraint_size).setConstant(-INFTY);
            ubA.segment(torque_limit_constraint_size, contact_constraint_size) = -bA;

            Eigen::VectorXd qpres;

            if (qp_contact_.CheckProblemSize(variable_number, total_constraint_size))
            {
                if (init)
                {
                    qp_contact_.InitializeProblemSize(variable_number, total_constraint_size);
                }
            }
            else
            {
                qp_contact_.InitializeProblemSize(variable_number, total_constraint_size);
            }

            qp_contact_.UpdateMinProblem(H, g);
            qp_contact_.UpdateSubjectToAx(A_qp, lbA, ubA);
            qp_contact_.DeleteSubjectToX();
            if (qp_contact_.SolveQPoases(600, qpres))
            {
                torque_output = rd_.NwJw * qpres;
                return 1;
            }
            else
            {
                torque_output = VectorXd::Zero(model_size);
                return 0;
            }
        }
        else
        {
            torque_output = VectorXd::Zero(model_size);
            return 1;
        }
    }

    void CalcJKT(RobotData &rd_, TaskSpace &ts_)
    {
        ts_.Lambda_task_ = (ts_.J_task_ * rd_.A_inv_ * rd_.N_C * ts_.J_task_.transpose()).inverse();
        MatrixXd Q = (ts_.Lambda_task_ * ts_.J_task_ * rd_.A_inv_ * rd_.N_C).rightCols(ts_.J_task_.cols() - 6);
        ts_.J_kt_ = rd_.W_inv * Q.transpose() * DyrosMath::pinv_COD(Q * rd_.W_inv * Q.transpose());
    }
    void CalcTaskNull(RobotData &rd_, TaskSpace &ts_)
    {
        ts_.Null_task = MatrixXd::Identity(MODEL_DOF, MODEL_DOF) - ts_.J_kt_ * ts_.Lambda_task_ * ts_.J_task_ * rd_.A_inv_ * rd_.N_C.rightCols(MODEL_DOF);
    }
}
