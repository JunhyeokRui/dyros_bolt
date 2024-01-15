#ifndef REAL_ROBOT_INTERFACE_H
#define REAL_ROBOT_INTERFACE_H

#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <unordered_set>
#include "dyros_bolt_controller/odrive_socketcan.h"

#define DEFAULT_CAN_INTERFACE       "can0"
#define DEFAULT_CAN_BITRATE         500000
#define DEFAULT_ENGAGE_ON_STARTUP   false
#define DEFAULT_DISENGAGE_ON_SHUTDOWN   true

#include "control_base.h"

namespace dyros_bolt_controller
{
class RealRobotInterface : public ControlBase
{
public:
    SHMmsgs *tc_shm_;
    int shm_id_;

    RealRobotInterface(ros::NodeHandle &nh, double Hz);
    virtual ~RealRobotInterface() { deleteSharedMemory(shm_id_, tc_shm_); }

    virtual void readDevice() override;
    virtual void update() override; // update controller based on readdevice
    virtual void writeDevice() override; // publish to actuate devices
    virtual void wait() override;

    void axisRequestStateCallback(const std_msgs::Int16::ConstPtr& msg);
    
    odrive::ODriveSocketCan odrv;

private:
    ros::Rate rate_;

    ros::Subscriber axis_request_state_sub;
    ros::Publisher axis_current_state_pub;

    bool areMotorsReady();
    void axisCurrentPublish();
    

};

}
#endif // REAL_ROBOT_INTERFACE_H










