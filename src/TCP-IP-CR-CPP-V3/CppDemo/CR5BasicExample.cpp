#include "api/Dashboard.h"
#include "api/DobotClient.h"
#include "api/DobotMove.h"
#include "api/Feedback.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

namespace
{
constexpr unsigned short kDashboardPort = 29999;
constexpr unsigned short kMovePort = 30003;
constexpr unsigned short kFeedbackPort = 30004;

class CR5Connection
{
public:
    explicit CR5Connection(std::string ip) : ip_(std::move(ip)) {}

    ~CR5Connection()
    {
        feedback_.Disconnect();
        move_.Disconnect();
        dashboard_.Disconnect();
        Dobot::CDobotClient::UinitNet();
    }

    bool connect()
    {
        if (!Dobot::CDobotClient::InitNet()) {
            std::cerr << "network initialization failed\n";
            return false;
        }

        const bool dashboard_ok = dashboard_.Connect(ip_, kDashboardPort);
        const bool move_ok = move_.Connect(ip_, kMovePort);
        const bool feedback_ok = feedback_.Connect(ip_, kFeedbackPort);

        if (!(dashboard_ok && move_ok && feedback_ok)) {
            std::cerr << "connection failed: dashboard=" << dashboard_ok
                      << " move=" << move_ok
                      << " feedback=" << feedback_ok << '\n';
            return false;
        }
        return true;
    }

    bool waitForFeedback(std::chrono::milliseconds timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (feedback_.IsDataHasRead()) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
    }

    void printReadOnlyStatus()
    {
        std::cout << "RobotMode: " << dashboard_.RobotMode() << '\n';
        std::cout << "GetPose:   " << dashboard_.GetPose() << '\n';
        std::cout << "GetAngle:  " << dashboard_.GetAngle() << '\n';
        std::cout << "GetErrorID:" << dashboard_.GetErrorID() << '\n';

        if (!waitForFeedback(std::chrono::seconds(3))) {
            std::cerr << "no valid 30004 feedback received within 3 seconds\n";
            return;
        }

        const Dobot::CFeedbackData data = feedback_.GetFeedbackData();
        std::cout << "feedback mode=" << data.RobotMode
                  << " enable=" << static_cast<int>(data.EnableStatus)
                  << " running=" << static_cast<int>(data.RunningStatus)
                  << " error=" << static_cast<int>(data.ErrorStatus) << '\n';
        std::cout << "feedback TCP=["
                  << data.ToolVectorActual[0] << ", "
                  << data.ToolVectorActual[1] << ", "
                  << data.ToolVectorActual[2] << ", "
                  << data.ToolVectorActual[3] << ", "
                  << data.ToolVectorActual[4] << ", "
                  << data.ToolVectorActual[5] << "]\n";
    }

    Dobot::CDashboard& dashboard() { return dashboard_; }
    Dobot::CDobotMove& move() { return move_; }
    Dobot::CFeedback& feedback() { return feedback_; }

private:
    std::string ip_;
    Dobot::CDashboard dashboard_;
    Dobot::CDobotMove move_;
    Dobot::CFeedback feedback_;
};
}  // namespace

int main(int argc, char* argv[])
{
    const std::string robot_ip = argc > 1 ? argv[1] : "192.168.1.6";
    CR5Connection robot(robot_ip);
    if (!robot.connect()) {
        return EXIT_FAILURE;
    }

    // This example is intentionally read-only. It never enables or moves the robot.
    robot.printReadOnlyStatus();
    return EXIT_SUCCESS;
}
