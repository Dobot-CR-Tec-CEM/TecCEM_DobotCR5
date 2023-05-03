/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/07
 *
 * <h2><center>&copy; COPYRIGHT 2021 DOBOT CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#include <ros/ros.h>
#include <ros/param.h>
#include <dobot_bringup/cr5_robot.h>

CR5Robot::CR5Robot(ros::NodeHandle& nh, std::string name)
    : ActionServer<FollowJointTrajectoryAction>(nh, std::move(name), false)
    , goal_{}
    , control_nh_(nh)
    , trajectory_duration_(1.0)
{
    index_ = 0;
    memset(goal_, 0, sizeof(goal_));
}

CR5Robot::~CR5Robot()
{
    ROS_INFO("~CR5Robot");
}

void CR5Robot::init()
{
    std::string ip = control_nh_.param<std::string>("robot_ip_address", "192.168.5.1");

    trajectory_duration_ = control_nh_.param("trajectory_duration", 0.3);
    ROS_INFO("trajectory_duration : %0.2f", trajectory_duration_);

    commander_ = std::make_shared<CR5Commander>(ip);
    commander_->init();

    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/EnableRobot", &CR5Robot::enableRobot, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/DisableRobot", &CR5Robot::disableRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/ClearError", &CR5Robot::clearError, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/ResetRobot", &CR5Robot::resetRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/SpeedFactor", &CR5Robot::speedFactor, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/User", &CR5Robot::user, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/Tool", &CR5Robot::tool, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/RobotMode", &CR5Robot::robotMode, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/PayLoad", &CR5Robot::payload, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/DO", &CR5Robot::DO, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/DOExecute", &CR5Robot::DOExecute, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/ToolDO", &CR5Robot::toolDO, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/ToolDOExecute", &CR5Robot::toolDOExecute, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/AO", &CR5Robot::AO, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/AOExecute", &CR5Robot::AOExecute, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/AccJ", &CR5Robot::accJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/AccL", &CR5Robot::accL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/SpeedJ", &CR5Robot::speedJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/SpeedL", &CR5Robot::speedL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/Arch", &CR5Robot::arch, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/CP", &CR5Robot::cp, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/LimZ", &CR5Robot::limZ, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/SetArmOrientation", &CR5Robot::setArmOrientation, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/PowerOn", &CR5Robot::powerOn, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/RunScript", &CR5Robot::runScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/StopScript", &CR5Robot::stopScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/PauseScript", &CR5Robot::pauseScript, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/ContinueScript", &CR5Robot::continueScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/SetSafeSkin", &CR5Robot::setSafeSkin, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/SetObstacleAvoid", &CR5Robot::setObstacleAvoid, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/SetCollisionLevel", &CR5Robot::setCollisionLevel, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/EmergencyStop", &CR5Robot::emergencyStop, this));

    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/MovJ", &CR5Robot::movJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/MovL", &CR5Robot::movL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/JointMovJ", &CR5Robot::jointMovJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/Jump", &CR5Robot::jump, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/RelMovJ", &CR5Robot::relMovJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/RelMovL", &CR5Robot::relMovL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/Arc", &CR5Robot::arc, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/Circle", &CR5Robot::circle, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/ServoJ", &CR5Robot::servoJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/ServoP", &CR5Robot::servoP, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/Sync", &CR5Robot::sync, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/StartTrace", &CR5Robot::startTrace, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/StartPath", &CR5Robot::startPath, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/StartFCTrace", &CR5Robot::startFCTrace, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/MoveJog", &CR5Robot::moveJog, this));

    registerGoalCallback(boost::bind(&CR5Robot::goalHandle, this, _1));
    registerCancelCallback(boost::bind(&CR5Robot::cancelHandle, this, _1));
    start();
}

void CR5Robot::feedbackHandle(const ros::TimerEvent& tm,
                              ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    control_msgs::FollowJointTrajectoryFeedback feedback;

    double current_joints[6];
    getJointState(current_joints);

    for (uint32_t i = 0; i < 6; i++)
    {
        feedback.joint_names.push_back(std::string("joint") + std::to_string(i + 1));
        feedback.actual.positions.push_back(current_joints[i]);
        feedback.desired.positions.push_back(goal_[i]);
    }

    handle.publishFeedback(feedback);
}

void CR5Robot::moveHandle(const ros::TimerEvent& tm,
                          ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    control_msgs::FollowJointTrajectoryGoalConstPtr trajectory = handle.getGoal();

    if (index_ < trajectory->trajectory.points.size())
    {
        auto point = trajectory->trajectory.points[index_].positions;
        double tmp[6];
        for (uint32_t i = 0; i < 6; i++)
        {
            tmp[i] = point[i] * 180.0 / 3.1415926;
        }

        commander_->servoJ(tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5]);
        index_++;
    }
    else
    {
#define OFFSET_VAL 0.01
        double current_joints[6];
        getJointState(current_joints);
        if ((current_joints[0] >= goal_[0] - OFFSET_VAL) && (current_joints[0] <= goal_[0] + OFFSET_VAL) &&
            (current_joints[1] >= goal_[1] - OFFSET_VAL) && (current_joints[1] <= goal_[1] + OFFSET_VAL) &&
            (current_joints[2] >= goal_[2] - OFFSET_VAL) && (current_joints[2] <= goal_[2] + OFFSET_VAL) &&
            (current_joints[3] >= goal_[3] - OFFSET_VAL) && (current_joints[3] <= goal_[3] + OFFSET_VAL) &&
            (current_joints[4] >= goal_[4] - OFFSET_VAL) && (current_joints[4] <= goal_[4] + OFFSET_VAL) &&
            (current_joints[5] >= goal_[5] - OFFSET_VAL) && (current_joints[5] <= goal_[5] + OFFSET_VAL))
        {
            timer_.stop();
            movj_timer_.stop();
            handle.setSucceeded();
        }
    }
}

void CR5Robot::goalHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    index_ = 0;
    for (uint32_t i = 0; i < 6; i++)
    {
        goal_[i] = handle.getGoal()->trajectory.points[handle.getGoal()->trajectory.points.size() - 1].positions[i];
    }
    timer_ = control_nh_.createTimer(ros::Duration(1.0), boost::bind(&CR5Robot::feedbackHandle, this, _1, handle));
    movj_timer_ = control_nh_.createTimer(ros::Duration(trajectory_duration_),
                                          boost::bind(&CR5Robot::moveHandle, this, _1, handle));
    timer_.start();
    movj_timer_.start();
    handle.setAccepted();
}

void CR5Robot::cancelHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    timer_.stop();
    movj_timer_.stop();
    handle.setSucceeded();
}

void CR5Robot::getJointState(double* point)
{
    commander_->getCurrentJointStatus(point);
}

bool CR5Robot::isEnable() const
{
    return commander_->isEnable();
}

bool CR5Robot::isConnected() const
{
    return commander_->isConnected();
}

void CR5Robot::getToolVectorActual(double* val)
{
    commander_->getToolVectorActual(val);
}

/*
 *----------------------------------------------------------------------------------------------------------------------
 *                                                  dashboard
 *----------------------------------------------------------------------------------------------------------------------
 */

bool CR5Robot::enableRobot(dobot_bringup::EnableRobot::Request& request, dobot_bringup::EnableRobot::Response& response)
{
    try
    {
        commander_->enableRobot();
        response.res = 0;
        return true;
    }
    catch (const std::exception& err)
    {
        commander_->clearError();
        response.res = -1;
        return false;
    }
}

bool CR5Robot::disableRobot(dobot_bringup::DisableRobot::Request& request,
                            dobot_bringup::DisableRobot::Response& response)
{
    try
    {
        commander_->disableRobot();
        response.res = 0;
        return true;
    }
    catch (const std::exception& err)
    {
        commander_->clearError();
        response.res = -1;
        return false;
    }
}

bool CR5Robot::clearError(dobot_bringup::ClearError::Request& request, dobot_bringup::ClearError::Response& response)
{
    try
    {
        commander_->clearError();
        response.res = 0;
        return true;
    }
    catch (const std::exception& err)
    {
        commander_->clearError();
        response.res = -1;
        return false;
    }
}

bool CR5Robot::resetRobot(dobot_bringup::ResetRobot::Request& request, dobot_bringup::ResetRobot::Response& response)
{
    try
    {
        commander_->resetRobot();
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::speedFactor(dobot_bringup::SpeedFactor::Request& request, dobot_bringup::SpeedFactor::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SpeedFactor(%d)", request.ratio);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::user(dobot_bringup::User::Request& request, dobot_bringup::User::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "User(%d)", request.index);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::tool(dobot_bringup::Tool::Request& request, dobot_bringup::Tool::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Tool(%d)", request.index);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::robotMode(dobot_bringup::RobotMode::Request& request, dobot_bringup::RobotMode::Response& response)
{
    try
    {
        const char* cmd = "RobotMode()";
        char result[100];
        memset(result, 0, sizeof(result));
        commander_->dashSendCmd(cmd, strlen(cmd));

        if (commander_->dashRecvCmd(result, sizeof(result), 100))
        {
            char* end_str;
            response.mode = strtol(result, &end_str, 10);
            if (*end_str != '\0')
            {
                ROS_ERROR("RobotMode() : Invalid result : %s", result);
                response.res = -1;
                return false;
            }
            response.res = 0;
        }
        else
        {
            ROS_ERROR("RobotMode() : Recv timeout");
            response.mode = -1;
            response.res = -1;
        }

        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::payload(dobot_bringup::PayLoad::Request& request, dobot_bringup::PayLoad::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "PayLoad(%0.3f, %0.3f)", request.weight, request.inertia);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::DO(dobot_bringup::DO::Request& request, dobot_bringup::DO::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "DO(%d, %d)", request.index, request.status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::DOExecute(dobot_bringup::DOExecute::Request& request, dobot_bringup::DOExecute::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "DO(%d, %d)", request.index, request.status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::toolDO(dobot_bringup::ToolDO::Request& request, dobot_bringup::ToolDO::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "ToolDO(%d, %d)", request.index, request.status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::toolDOExecute(dobot_bringup::ToolDOExecute::Request& request,
                             dobot_bringup::ToolDOExecute::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "ToolDOExecute(%d, %d)", request.index, request.status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::AO(dobot_bringup::AO::Request& request, dobot_bringup::AO::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AO(%d, %d)", request.index, request.status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::AOExecute(dobot_bringup::AOExecute::Request& request, dobot_bringup::AOExecute::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AO(%d, %0.3f)", request.index, static_cast<float>(request.value));
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::accJ(dobot_bringup::AccJ::Request& request, dobot_bringup::AccJ::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AccJ(%d)", request.r);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::accL(dobot_bringup::AccL::Request& request, dobot_bringup::AccL::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AccL(%d)", request.r);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::speedJ(dobot_bringup::SpeedJ::Request& request, dobot_bringup::SpeedJ::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SpeedJ(%d)", request.r);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::speedL(dobot_bringup::SpeedL::Request& request, dobot_bringup::SpeedL::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SpeedL(%d)", request.r);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::arch(dobot_bringup::Arch::Request& request, dobot_bringup::Arch::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Arch(%d)", request.index);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::cp(dobot_bringup::CP::Request& request, dobot_bringup::CP::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "CP(%d)", request.r);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::limZ(dobot_bringup::LimZ::Request& request, dobot_bringup::LimZ::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "LimZ(%d)", request.value);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::setArmOrientation(dobot_bringup::SetArmOrientation::Request& request,
                                 dobot_bringup::SetArmOrientation::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetArmOrientation(%d,%d,%d,%d)", request.LorR, request.UorD, request.ForN, request.Config6);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::powerOn(dobot_bringup::PowerOn::Request& request, dobot_bringup::PowerOn::Response& response)
{
    try
    {
        const char* cmd = "PowerOn()";
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::runScript(dobot_bringup::RunScript::Request& request, dobot_bringup::RunScript::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "RunScript(%s)", request.projectName.c_str());
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::stopScript(dobot_bringup::StopScript::Request& request, dobot_bringup::StopScript::Response& response)
{
    try
    {
        const char* cmd = "StopScript()";
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::pauseScript(dobot_bringup::PauseScript::Request& request, dobot_bringup::PauseScript::Response& response)
{
    try
    {
        const char* cmd = "PauseScript()";
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::continueScript(dobot_bringup::ContinueScript::Request& request,
                              dobot_bringup::ContinueScript::Response& response)
{
    try
    {
        const char* cmd = "ContinueScript()";
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::setSafeSkin(dobot_bringup::SetSafeSkin::Request& request, dobot_bringup::SetSafeSkin::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetSafeSkin(%d)", request.status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::setObstacleAvoid(dobot_bringup::SetObstacleAvoid::Request& request,
                                dobot_bringup::SetObstacleAvoid::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetObstacleAvoid(%d)", request.status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::setCollisionLevel(dobot_bringup::SetCollisionLevel::Request& request,
                                 dobot_bringup::SetCollisionLevel::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetCollisionLevel(%d)", request.level);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::emergencyStop(dobot_bringup::EmergencyStop::Request& request,
                             dobot_bringup::EmergencyStop::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "EmergencyStop()");
        commander_->dashSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

/*
 *----------------------------------------------------------------------------------------------------------------------
 *                                                  real time
 *----------------------------------------------------------------------------------------------------------------------
 */

bool CR5Robot::movJ(dobot_bringup::MovJ::Request& request, dobot_bringup::MovJ::Response& response)
{
    try
    {
        commander_->movJ(request.x, request.y, request.z, request.a, request.b, request.c);
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::movL(dobot_bringup::MovL::Request& request, dobot_bringup::MovL::Response& response)
{
    try
    {
        commander_->movL(request.x, request.y, request.z, request.a, request.b, request.c);
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::servoJ(dobot_bringup::ServoJ::Request& request, dobot_bringup::ServoJ::Response& response)
{
    try
    {
        commander_->servoJ(request.j1, request.j2, request.j3, request.j4, request.j5, request.j6);
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::jump(dobot_bringup::Jump::Request& request, dobot_bringup::Jump::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Jump(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request.offset1, request.offset2, request.offset3,
                request.offset4, request.offset5, request.offset6);
        commander_->realSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::arc(dobot_bringup::Arc::Request& request, dobot_bringup::Arc::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Arc(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request.x1,
                request.y1, request.z1, request.rx1, request.ry1, request.rz1, request.x2, request.y2, request.z2,
                request.rx2, request.ry2, request.rz2);
        commander_->realSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::circle(dobot_bringup::Circle::Request& request, dobot_bringup::Circle::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Circle(%d, %0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)",
                request.count, request.x1, request.y1, request.z1, request.rx1, request.ry1, request.rz1, request.x2,
                request.y2, request.z2, request.rx2, request.ry2, request.rz2);
        commander_->realSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::servoP(dobot_bringup::ServoP::Request& request, dobot_bringup::ServoP::Response& response)
{
    try
    {
        commander_->servoP(request.x, request.y, request.z, request.a, request.b, request.c);
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::relMovJ(dobot_bringup::RelMovJ::Request& request, dobot_bringup::RelMovJ::Response& response)
{
    try
    {
        commander_->relMovJ(request.offset1, request.offset2, request.offset3, request.offset4, request.offset5,
                            request.offset6);
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::relMovL(dobot_bringup::RelMovL::Request& request, dobot_bringup::RelMovL::Response& response)
{
    try
    {
        commander_->relMovL(request.x, request.y, request.z);
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::jointMovJ(dobot_bringup::JointMovJ::Request& request, dobot_bringup::JointMovJ::Response& response)
{
    try
    {
        commander_->jointMovJ(request.j1, request.j2, request.j3, request.j4, request.j5, request.j6);
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::sync(dobot_bringup::Sync::Request& request, dobot_bringup::Sync::Response& response)
{
    try
    {
        char result[50];
        const char* cmd = "Sync()";
        commander_->dashSendCmd(cmd, strlen(cmd));
        memset(result, 0, sizeof(result));
        if (commander_->dashRecvCmd(result, sizeof(result), 50000) && strstr(result, "done") != nullptr)
        {
            ROS_ERROR("sync execute failed");
            response.res = -1;
            return false;
        }
        else
        {
            response.res = 0;
            return true;
        }
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::startTrace(dobot_bringup::StartTrace::Request& request, dobot_bringup::StartTrace::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "StartTrace(%s)", request.trace_name.c_str());
        commander_->realSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::startPath(dobot_bringup::StartPath::Request& request, dobot_bringup::StartPath::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "StartPath(%s,%d,%d)", request.trace_name.c_str(), request.const_val, request.cart);
        commander_->realSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::startFCTrace(dobot_bringup::StartFCTrace::Request& request,
                            dobot_bringup::StartFCTrace::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "StartFCTrace(%s)", request.trace_name.c_str());
        commander_->realSendCmd(cmd, strlen(cmd));
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::moveJog(dobot_bringup::MoveJog::Request& request, dobot_bringup::MoveJog::Response& response)
{
    try
    {
        commander_->moveJog(request.axisID);
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}
