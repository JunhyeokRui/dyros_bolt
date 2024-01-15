#include "dyros_bolt_controller/odrive_socketcan.h"
// std::ofstream outFile("/home/yong/data.txt");

namespace odrive {
    ODriveSocketCan::ODriveSocketCan(ros::NodeHandle &nh):
        node(nh)
    {
        node.getParam("axis_can_ids", axis_can_ids_list);
        const char* ifname = "can0"; // Replace with your CAN interface name
        struct sockaddr_can addr;
        struct ifreq ifr;

        // Create a socket for the CAN interface
        if ((socketcan = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            perror("socket");
            throw std::runtime_error("Failed to create socket");
        }

        // Get the interface index
        strcpy(ifr.ifr_name, ifname);
        if (ioctl(socketcan, SIOCGIFINDEX, &ifr) < 0) {
            perror("ioctl");
            close(socketcan); // Close the socket on error
            throw std::runtime_error("Failed to get interface index");
        }

        // Assign the CAN interface to the socket
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(socketcan, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            perror("bind");
            close(socketcan);
            throw std::runtime_error("Failed to bind socket");
        }    
        
        for(int i = 0; i < axis_can_ids_list.size(); i++)
        {
            setControllerModes(axis_can_ids_list[i], TORQUE_CONTROL, PASSTHROUGH);
        }
        std::thread canRecieveThread(&ODriveSocketCan::canReceiveMessages,this);
        canRecieveThread.detach();
    }

    void ODriveSocketCan::canReceiveMessages() {
        struct can_frame recv_frame;
        while (true) {
            int nbytes = read(socketcan, &recv_frame, sizeof(struct can_frame));
            if (nbytes < 0) {
                perror("read");
                break;
            } else if (nbytes == sizeof(struct can_frame)) {
                // Process received CAN frame here
                uint32_t can_axis_id_ = recv_frame.can_id >> 5;
                uint32_t can_cmd_id_ = recv_frame.can_id & 0x01F;
                switch(can_cmd_id_) {
                case ODriveCommandId::HEARTBEAT_MESSAGE:
                    axis_error[static_cast<int>(can_axis_id_)] = bytesToInt(recv_frame.data);
                    axis_current_state[static_cast<int>(can_axis_id_)] = static_cast<int>(recv_frame.data[4]);
                    axis_controller_state[static_cast<int>(can_axis_id_)] = static_cast<int>(recv_frame.data[7]);
                case ODriveCommandId::GET_ENCODER_ESTIMATE:
                    axis_angle[static_cast<int>(can_axis_id_)] = double(bytesToFloat(recv_frame.data)) * 2 * M_PI / 9;
                    axis_velocity[static_cast<int>(can_axis_id_)] = double(bytesToFloat(recv_frame.data + 4)) * 2 * M_PI / 9;
                case ODriveCommandId::GET_IQ:
                    axis_current[static_cast<int>(can_axis_id_)] = double(bytesToFloat(recv_frame.data + 4));
                }

                    
            }
        }
    }

    void ODriveSocketCan::requestBusVoltageAndCurrent(int axis_can_id_) {
        struct can_frame frame;
        frame.can_id = createCanId(axis_can_id_, ODriveCommandId::GET_BUS_VOLTAGE_AND_CURRENT);
        frame.can_dlc = 8;    // Data length code (number of data bytes)

        if (write(socketcan, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("write");
            close(socketcan);
            throw std::runtime_error("Failed to sent CAN frame");
        }
    }

    void ODriveSocketCan::requestODriveCmd(int axis_can_id_, ODriveCommandId cmd) {
        struct can_frame frame;
        frame.can_id = createCanId(axis_can_id_, cmd);
        frame.can_dlc = 8;    // Data length code (number of data bytes)

        if (write(socketcan, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("write");
            close(socketcan);
            throw std::runtime_error("Failed to sent CAN frame");
        }
    }

    void ODriveSocketCan::resetEncoder(int axis_can_id_, ODriveCommandId cmd) {
        struct can_frame frame;
        frame.can_id = createCanId(axis_can_id_, cmd);
        frame.can_dlc = 8;    // Data length code (number of data bytes)
        
        // Data to be sent
        unsigned char data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        std::memcpy(frame.data, data, sizeof(data));

        if (write(socketcan, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("write");
            close(socketcan);
            throw std::runtime_error("Failed to sent CAN frame");
        }
    }

    void ODriveSocketCan::setAxisRequestedState(int axis_can_id_, ODriveAxisState state) {
        struct can_frame frame;
        frame.can_id = createCanId(axis_can_id_, ODriveCommandId::SET_AXIS_REQUESTED_STATE);
        frame.can_dlc = 8;    // Data length code (number of data bytes)

        // Data to be sent
        unsigned char data[] = {state, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        std::memcpy(frame.data, data, sizeof(data));

        // Send the CAN frame
        if (write(socketcan, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("write");
            close(socketcan);
            throw std::runtime_error("Failed to sent CAN frame");
        }
    }

    void ODriveSocketCan::setControllerModes(int axis_can_id_, ODriveControlMode control_mode, ODriveInputMode input_mode) {
        struct can_frame frame;
        frame.can_id = createCanId(axis_can_id_, ODriveCommandId::SET_CONTROLLER_MODES);
        frame.can_dlc = 8;    // Data length code (number of data bytes)

        // Data to be sent
        unsigned char data[] = {control_mode, 0x00, 0x00, 0x00, input_mode, 0x00, 0x00, 0x00};
        std::memcpy(frame.data, data, sizeof(data));

        // Send the CAN frame
        if (write(socketcan, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("write");
            close(socketcan);
            throw std::runtime_error("Failed to sent CAN frame");
        }
    }

    void ODriveSocketCan::setInputPosition(int axis_can_id_, double position) {
        if (position < -3.40282347e+38 || position > 3.40282347e+38) {
            std::cerr << "Double value is out of range for 4-byte CAN data." << std::endl;
            exit(1);
        }
        float pos_f_ = static_cast<float>(position/M_PI);

        struct can_frame frame;
        frame.can_id = createCanId(axis_can_id_, ODriveCommandId::SET_INPUT_POS);
        frame.can_dlc = 8;    // Data length code (number of data bytes)

        // Data to be sent
        std::memcpy(frame.data, &pos_f_, sizeof(pos_f_));

        // Send the CAN frame
        if (write(socketcan, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("write");
            close(socketcan);
            throw std::runtime_error("Failed to sent CAN frame");
        }
    }
    

    void ODriveSocketCan::setInputVelocity(int axis_can_id_, double velocity) {
        if (velocity < -3.40282347e+38 || velocity > 3.40282347e+38) {
            std::cerr << "Double value is out of range for 4-byte CAN data." << std::endl;
            exit(1);
        }
        float vel_f_ = static_cast<float>(velocity);

        struct can_frame frame;
        frame.can_id = createCanId(axis_can_id_, ODriveCommandId::SET_INPUT_VELOCITY);
        frame.can_dlc = 8;    // Data length code (number of data bytes)

        // Data to be sent
        std::memcpy(frame.data, &vel_f_, sizeof(vel_f_));

        // Send the CAN frame
        if (write(socketcan, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("write");
            close(socketcan);
            throw std::runtime_error("Failed to sent CAN frame");
        }
    }

    void ODriveSocketCan::setInputTorque(int axis_can_id_, double torque) {
        if (torque < -3.40282347e+38 || torque > 3.40282347e+38) {
            std::cerr << "Double value is out of range for 4-byte CAN data." << std::endl;
            exit(1);
        }
        float torq_f_ = static_cast<float>(torque);

        struct can_frame frame;
        frame.can_id = createCanId(axis_can_id_, ODriveCommandId::SET_INPUT_TORQUE);
        frame.can_dlc = 8;    // Data length code (number of data bytes)

        // Data to be sent
        std::memcpy(frame.data, &torq_f_, sizeof(torq_f_));

        // Send the CAN frame
        if (write(socketcan, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("write");
            close(socketcan);
            throw std::runtime_error("Failed to sent CAN frame");
        }
    }

    void ODriveSocketCan::engage() {
        for (int i = 0; i < axis_can_ids_list.size(); i++)
        {
            setAxisRequestedState(axis_can_ids_list[i], ODriveAxisState::CLOSED_LOOP_CONTROL);
        }
    }

    void ODriveSocketCan::disengage() {
        for (int i = 0; i < axis_can_ids_list.size(); i++)
        {
            setAxisRequestedState(axis_can_ids_list[i], ODriveAxisState::IDLE);
        }
    }


    uint32_t ODriveSocketCan::createCanId(int axis_can_id, int command) {
        uint32_t can_id;
        can_id = (axis_can_id << 5) | command;
        return can_id;
    }

    float ODriveSocketCan::bytesToFloat(const unsigned char* data) {
        if (data == nullptr) {
            return 0.0f; // Handle error gracefully
        }

        float value;
        std::memcpy(&value, data, sizeof(float)); // Assuming little-endian byte order
        return value;
    }

    int ODriveSocketCan::bytesToInt(const unsigned char* data) {
        if (data == nullptr) {
            return 0; // Handle error gracefully
        }

        int value;
        std::memcpy(&value, data, sizeof(int)); // Assuming little-endian byte order
        return value;
    }

}
