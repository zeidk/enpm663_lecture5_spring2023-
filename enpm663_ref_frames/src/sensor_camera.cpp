
#include <rclcpp/rclcpp.hpp>
#include "sensor_camera.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

void SensorCamera::CompetitionStateCallback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg)
{
    competition_state_ = msg->competition_state;
    // RCLCPP_INFO_STREAM(this->get_logger(), "competition_state_ = " << competition_state_);
}
// ================================
void SensorCamera::KitTrayTable1Callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    auto part_poses = msg->part_poses;
}
// ================================
void SensorCamera::KitTrayTable2Callback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    auto part_poses = msg->part_poses;
}
// ================================
void SensorCamera::LeftBinsCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    RCLCPP_INFO_ONCE(this->get_logger(), "------ LeftBinsCameraCallback");
    auto part_poses = msg->part_poses;
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Number of models: " << part_poses.size());
    for (auto part_pose : part_poses)
    {
        auto part = part_pose.part;
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Part type: " << (int)part.type);
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Part color: " << (int)part.color);
        auto pose = part_pose.pose;
        RCLCPP_INFO_ONCE(this->get_logger(), "Position: %f, %f, %f", pose.position.x, pose.position.y, pose.position.z);
        RCLCPP_INFO_ONCE(this->get_logger(), "Orientation: %f, %f, %f, %f", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    }
}
// ================================
void SensorCamera::RightBinsCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    RCLCPP_INFO_ONCE(this->get_logger(), "------ RightBinsCameraCallback");
    auto part_poses = msg->part_poses;
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Number of models: " << part_poses.size());
    for (auto part_pose : part_poses)
    {
        auto part = part_pose.part;
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Part type: " << (int)part.type);
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Part color: " << (int)part.color);
        auto pose = part_pose.pose;
        RCLCPP_INFO_ONCE(this->get_logger(), "Position: %f, %f, %f", pose.position.x, pose.position.y, pose.position.z);
        RCLCPP_INFO_ONCE(this->get_logger(), "Orientation: %f, %f, %f, %f", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    }
}

// ================================
void SensorCamera::StartCompetitionServiceCallback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
{
    if (start_competition_client_done_)
    {
        return;
    }
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready)
    {
        RCLCPP_INFO(this->get_logger(), "Result: success: %i, message: %s",
                    future.get()->success, future.get()->message.c_str());
        start_competition_client_done_ = true;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}

// ================================
bool SensorCamera::StartCompetition()
{
    // Wait for competition state to be ready
    while (competition_state_ != ariac_msgs::msg::CompetitionState::READY)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for competition state to be ready...");
        // RCLCPP_INFO_ONCE(this->get_logger(), "Competiton state: %i", competition_state_);
        if (!rclcpp::ok())
        {
            // Show an error if the user types CTRL + C
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for competition state. Exiting.");
            return false;
        }
    }
    RCLCPP_INFO(this->get_logger(), "Competition state is ready.");

    while (!start_competition_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            // Show an error if the user types CTRL + C
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    RCLCPP_INFO(this->get_logger(), "service /ariac/start_competition is available.");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    auto result = start_competition_client_->async_send_request(request);
    result.wait();

    return result.get()->success;
}
// ================================
// bool SensorCamera::StartCompetition()
// {
//     // Wait for competition state to be ready
//     while (competition_state_ != ariac_msgs::msg::CompetitionState::READY)
//     {
//         RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for competition state to be ready...");
//     }
//     RCLCPP_INFO(this->get_logger(), "Competition state is ready.");

//     while (!start_competition_client_->wait_for_service(1s))
//     {
//         if (!rclcpp::ok())
//         {
//             // Show an error if the user types CTRL + C
//             RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
//             return false;
//         }
//         RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
//     }
//     RCLCPP_INFO(this->get_logger(), "service /ariac/start_competition is available.");

//     auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
//     auto result_future = start_competition_client_->async_send_request(
//         request, std::bind(&SensorCamera::StartCompetitionServiceCallback, this,
//                            std::placeholders::_1));
//     return result_future.get()->success;
// }
