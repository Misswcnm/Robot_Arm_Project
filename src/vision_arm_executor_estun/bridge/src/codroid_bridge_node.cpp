#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <json/json.h>

#include "estun_codroid_bridge/srv/get_joints.hpp"
#include "estun_codroid_bridge/srv/get_pose.hpp"
#include "estun_codroid_bridge/srv/get_robot_state.hpp"
#include "estun_codroid_bridge/srv/move.hpp"
#include "estun_codroid_bridge/srv/robot_command.hpp"
#include "estun_codroid_bridge/srv/set_do.hpp"
#include "estun_codroid_bridge/srv/solve_pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

namespace
{
std::string json_string(const Json::Value & value)
{
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "";
  return Json::writeString(builder, value);
}

Json::Value parse_json(const std::string & text)
{
  Json::CharReaderBuilder builder;
  Json::Value value;
  std::string error;
  std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
  if (!reader->parse(text.data(), text.data() + text.size(), &value, &error)) {
    throw std::runtime_error("invalid controller JSON: " + error);
  }
  return value;
}

std::string response_message(const Json::Value & value)
{
  std::string message = "controller rejected request";
  if (value.isObject() && value.isMember("msg") && value["msg"].isString()) {
    message = value["msg"].asString();
  }
  if (value.isObject() && value.isMember("code")) {
    return "code=" + std::to_string(value["code"].asInt()) +
           " msg=" + message;
  }
  return message;
}
}  // namespace

class CodroidClient
{
public:
  CodroidClient(std::string host, std::string port, double timeout_sec)
  : host_(std::move(host)), port_(std::move(port)), timeout_sec_(timeout_sec), resolver_(ioc_)
  {
  }

  Json::Value call(
    const std::string & action, const Json::Value & data,
    double response_timeout_sec = -1.0)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool read_only =
      action == "getparam" || action == "getRobotStates" ||
      action == "getCurCPos" || action == "getCurAPos";
    const int attempts = read_only ? 2 : 1;
    for (int attempt = 0; attempt < attempts; ++attempt) {
      try {
        ensure_connected();
        return call_connected(action, data, response_timeout_sec);
      } catch (const std::exception &) {
        socket_.reset();
        if (attempt + 1 >= attempts) {
          throw;
        }
      }
    }
    throw std::runtime_error("unreachable Codroid retry state");
  }

private:
  void ensure_connected()
  {
    if (socket_ && socket_->is_open()) {
      return;
    }
    ioc_.restart();
    auto results = resolver_.resolve(host_, port_);
    if (results.empty()) {
      throw std::runtime_error("Codroid host resolved to no endpoints");
    }
    socket_ = std::make_unique<websocket::stream<beast::tcp_stream>>(ioc_);
    bool connect_done = false;
    beast::error_code connect_error;
    beast::get_lowest_layer(*socket_).socket().async_connect(
      results.begin()->endpoint(),
      [&connect_done, &connect_error](beast::error_code error) {
        connect_error = error;
        connect_done = true;
      });
    run_bounded(connect_done, "connect");
    if (connect_error) {
      throw beast::system_error(connect_error);
    }
    socket_->set_option(websocket::stream_base::timeout::suggested(beast::role_type::client));
    bool handshake_done = false;
    beast::error_code handshake_error;
    ioc_.restart();
    socket_->async_handshake(
      host_, "/",
      [&handshake_done, &handshake_error](beast::error_code error) {
        handshake_error = error;
        handshake_done = true;
      });
    run_bounded(handshake_done, "handshake");
    if (handshake_error) {
      throw beast::system_error(handshake_error);
    }
  }

  void run_bounded(
    bool & completed, const char * operation,
    double timeout_sec = -1.0)
  {
    ioc_.restart();
    const double bounded_timeout_sec =
      timeout_sec > 0.0 ? timeout_sec : timeout_sec_;
    const auto timeout = std::chrono::milliseconds(
      static_cast<int64_t>(bounded_timeout_sec * 1000.0));
    ioc_.run_for(timeout);
    if (completed) {
      return;
    }
    if (socket_) {
      beast::get_lowest_layer(*socket_).cancel();
    }
    // Drain the cancellation callback before the stack variables captured by
    // that callback go out of scope.
    ioc_.restart();
    ioc_.run();
    throw std::runtime_error(std::string("Codroid ") + operation + " timed out");
  }

  Json::Value call_connected(
    const std::string & action, const Json::Value & data,
    double response_timeout_sec)
  {
    Json::Value request(Json::objectValue);
    const auto id = ++request_id_;
    request["id"] = Json::UInt64(id);
    request["type"] = "common";
    request["action"] = action;
    request["data"] = data;

    const std::string payload = json_string(request);
    bool write_done = false;
    beast::error_code write_error;
    ioc_.restart();
    socket_->async_write(
      net::buffer(payload),
      [&write_done, &write_error](beast::error_code error, std::size_t) {
        write_error = error;
        write_done = true;
      });
    run_bounded(write_done, "write");
    if (write_error) {
      throw beast::system_error(write_error);
    }

    for (int attempt = 0; attempt < 8; ++attempt) {
      beast::flat_buffer buffer;
      bool read_done = false;
      beast::error_code read_error;
      ioc_.restart();
      socket_->async_read(
        buffer,
        [&read_done, &read_error](beast::error_code error, std::size_t) {
          read_error = error;
          read_done = true;
        });
      run_bounded(read_done, "read", response_timeout_sec);
      if (read_error) {
        throw beast::system_error(read_error);
      }
      Json::Value response = parse_json(beast::buffers_to_string(buffer.data()));
      if (!response.isMember("id") || response["id"].asUInt64() != id) {
        continue;
      }
      if (!response.isMember("code") || response["code"].asInt() != 200) {
        throw std::runtime_error("Codroid RPC failed: " + response_message(response));
      }
      const Json::Value inner = response["data"];
      if (!inner.isObject() || !inner.isMember("code") || inner["code"].asInt() != 0) {
        throw std::runtime_error("ESTUN command failed: " + response_message(inner));
      }
      return inner.isMember("data") ? inner["data"] : Json::Value(Json::nullValue);
    }
    throw std::runtime_error("no matching Codroid response received");
  }

  std::string host_;
  std::string port_;
  double timeout_sec_;
  net::io_context ioc_;
  tcp::resolver resolver_;
  std::unique_ptr<websocket::stream<beast::tcp_stream>> socket_;
  uint64_t request_id_{0};
  std::mutex mutex_;
};

class EstunCodroidBridge : public rclcpp::Node
{
public:
  EstunCodroidBridge()
  : Node("estun_codroid_bridge")
  {
    const auto host = declare_parameter<std::string>("robot_ip", "192.168.2.5");
    const auto port_number = declare_parameter<int>("robot_port", 9000);
    const auto port = std::to_string(port_number);
    namespace_ = declare_parameter<std::string>("service_namespace", "/estun_codroid");
    const auto timeout = declare_parameter<double>("request_timeout_sec", 5.0);
    motion_request_timeout_sec_ = declare_parameter<double>(
      "motion_request_timeout_sec", 30.0);
    if (motion_request_timeout_sec_ <= 0.0) {
      throw std::runtime_error("motion_request_timeout_sec must be positive");
    }
    if (namespace_.empty() || namespace_.front() != '/') {
      namespace_ = "/" + namespace_;
    }
    while (namespace_.size() > 1 && namespace_.back() == '/') {
      namespace_.pop_back();
    }
    client_ = std::make_unique<CodroidClient>(host, port, std::max(0.5, timeout));

    get_pose_service_ = create_service<estun_codroid_bridge::srv::GetPose>(
      endpoint("get_pose"),
      [this](
        const std::shared_ptr<estun_codroid_bridge::srv::GetPose::Request>,
        const std::shared_ptr<estun_codroid_bridge::srv::GetPose::Response> response) {
        get_pose(response);
      });
    get_joints_service_ = create_service<estun_codroid_bridge::srv::GetJoints>(
      endpoint("get_joints"),
      [this](
        const std::shared_ptr<estun_codroid_bridge::srv::GetJoints::Request>,
        const std::shared_ptr<estun_codroid_bridge::srv::GetJoints::Response> response) {
        get_joints(response);
      });
    get_state_service_ = create_service<estun_codroid_bridge::srv::GetRobotState>(
      endpoint("get_robot_state"),
      [this](
        const std::shared_ptr<estun_codroid_bridge::srv::GetRobotState::Request>,
        const std::shared_ptr<estun_codroid_bridge::srv::GetRobotState::Response> response) {
        get_state(response);
      });
    command_service_ = create_service<estun_codroid_bridge::srv::RobotCommand>(
      endpoint("command"),
      [this](
        const std::shared_ptr<estun_codroid_bridge::srv::RobotCommand::Request> request,
        const std::shared_ptr<estun_codroid_bridge::srv::RobotCommand::Response> response) {
        command(request, response);
      });
    move_service_ = create_service<estun_codroid_bridge::srv::Move>(
      endpoint("move"),
      [this](
        const std::shared_ptr<estun_codroid_bridge::srv::Move::Request> request,
        const std::shared_ptr<estun_codroid_bridge::srv::Move::Response> response) {
        move(request, response);
      });
    set_do_service_ = create_service<estun_codroid_bridge::srv::SetDo>(
      endpoint("set_do"),
      [this](
        const std::shared_ptr<estun_codroid_bridge::srv::SetDo::Request> request,
        const std::shared_ptr<estun_codroid_bridge::srv::SetDo::Response> response) {
        set_do(request, response);
      });
    solve_pose_service_ = create_service<estun_codroid_bridge::srv::SolvePose>(
      endpoint("solve_pose"),
      [this](
        const std::shared_ptr<estun_codroid_bridge::srv::SolvePose::Request> request,
        const std::shared_ptr<estun_codroid_bridge::srv::SolvePose::Response> response) {
        solve_pose(request, response);
      });

    RCLCPP_INFO(
      get_logger(),
      "Codroid bridge ROS services ready: controller=%s:%s services=%s/* pose=mm/deg "
      "request_timeout=%.1fs motion_timeout=%.1fs",
      host.c_str(), port.c_str(), namespace_.c_str(), timeout,
      motion_request_timeout_sec_);
    try {
      const int state = query_state();
      RCLCPP_INFO(
        get_logger(), "Codroid controller connection verified: state=%d", state);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(),
        "Codroid controller preflight failed: %s. ROS services remain "
        "available and will retry on the next request.", error.what());
    }
  }

private:
  std::string endpoint(const std::string & name) const
  {
    return namespace_ + "/" + name;
  }

  Json::Value array_data() const
  {
    return Json::Value(Json::arrayValue);
  }

  Json::Value controller_call(
    const std::string & action, const Json::Value & data,
    double response_timeout_sec = -1.0)
  {
    return client_->call(action, data, response_timeout_sec);
  }

  int query_state()
  {
    Json::Value paths(Json::arrayValue);
    paths.append("Robot/Control/state");
    const auto data = controller_call("getparam", paths);
    return data["Robot/Control/state"].asInt();
  }

  uint32_t query_status_flag()
  {
    const auto data = controller_call("getRobotStates", array_data());
    return data.isMember("statusFlag") ? data["statusFlag"].asUInt() : 0U;
  }

  template<typename ResponseT, typename FunctionT>
  void guarded(const std::shared_ptr<ResponseT> & response, FunctionT operation)
  {
    try {
      operation();
      response->success = true;
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "%s", error.what());
    }
  }

  void get_pose(const std::shared_ptr<estun_codroid_bridge::srv::GetPose::Response> & response)
  {
    guarded(response, [this, response]() {
      const auto data = controller_call("getCurCPos", array_data());
      // This controller firmware reports millimetres/degrees and a/b/c are
      // the actual Rx/Ry/Rz Euler fields (confirmed against fixed-board
      // hand-eye consistency). Keep the shared vision pose in XYZ-RPY order.
      response->pose = {
        data["x"].asDouble(), data["y"].asDouble(), data["z"].asDouble(),
        data["a"].asDouble(), data["b"].asDouble(), data["c"].asDouble()};
      response->message = "pose read";
    });
  }

  void get_joints(const std::shared_ptr<estun_codroid_bridge::srv::GetJoints::Response> & response)
  {
    guarded(response, [this, response]() {
      const auto data = controller_call("getCurAPos", array_data());
      for (std::size_t index = 0; index < response->joints.size(); ++index) {
        response->joints[index] = data["jntpos" + std::to_string(index + 1)].asDouble();
      }
      response->message = "joints read";
    });
  }

  void get_state(const std::shared_ptr<estun_codroid_bridge::srv::GetRobotState::Response> & response)
  {
    guarded(response, [this, response]() {
      response->state = query_state();
      response->status_flag = query_status_flag();
      response->message = "state read";
    });
  }

  void command(
    const std::shared_ptr<estun_codroid_bridge::srv::RobotCommand::Request> & request,
    const std::shared_ptr<estun_codroid_bridge::srv::RobotCommand::Response> & response)
  {
    try {
      if (request->command == estun_codroid_bridge::srv::RobotCommand::Request::STOP_MOTION) {
        controller_call("stopMov", array_data());
      } else {
        Json::Value data(Json::arrayValue);
        Json::Value item(Json::objectValue);
        item["path"] = "Robot/Control/command";
        item["value"] = request->command;
        data.append(item);
        controller_call("setparam", data);
      }
      response->state = query_state();
      response->success = true;
      response->message = "command accepted";
    } catch (const std::exception & error) {
      response->success = false;
      response->state = -1;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "%s", error.what());
    }
  }

  Json::Value position_config() const
  {
    Json::Value config(Json::objectValue);
    config["mode"] = -1;
    for (int index = 1; index <= 7; ++index) {
      config["cf" + std::to_string(index)] = 0;
    }
    return config;
  }

  Json::Value cpos_from_pose(const std::array<double, 6> & pose) const
  {
    Json::Value cpos(Json::objectValue);
    cpos["x"] = pose[0];
    cpos["y"] = pose[1];
    cpos["z"] = pose[2];
    cpos["a"] = pose[3];
    cpos["b"] = pose[4];
    cpos["c"] = pose[5];
    cpos["e"] = 0.0;
    cpos["poscfg"] = position_config();
    return cpos;
  }

  void solve_pose(
    const std::shared_ptr<estun_codroid_bridge::srv::SolvePose::Request> & request,
    const std::shared_ptr<estun_codroid_bridge::srv::SolvePose::Response> & response)
  {
    try {
      Json::Value data(Json::objectValue);
      data["cpos"] = cpos_from_pose(request->target);
      if (request->use_reference) {
        Json::Value reference(Json::objectValue);
        for (std::size_t index = 0; index < request->reference_joints.size(); ++index) {
          reference["jntpos" + std::to_string(index + 1)] =
            request->reference_joints[index];
        }
        reference["jntpos7"] = 0.0;
        data["apos"] = reference;
      } else {
        data["apos"] = controller_call("getCurAPos", array_data());
      }
      const auto result = controller_call("cpostoapos", data);
      const auto apos = result.isMember("apos") ? result["apos"] : result;
      for (std::size_t index = 0; index < response->joints.size(); ++index) {
        const auto key = "jntpos" + std::to_string(index + 1);
        if (!apos.isObject() || !apos.isMember(key)) {
          throw std::runtime_error("cpostoapos returned no joint target");
        }
        response->joints[index] = apos[key].asDouble();
      }
      response->success = true;
      response->message = "pose solved";
    } catch (const std::exception & error) {
      // An unreachable candidate is expected during orientation search. Return
      // it as data without flooding the controller log with ERROR messages.
      response->success = false;
      response->message = error.what();
    }
  }

  void move(
    const std::shared_ptr<estun_codroid_bridge::srv::Move::Request> & request,
    const std::shared_ptr<estun_codroid_bridge::srv::Move::Response> & response)
  {
    try {
      if (request->target.size() != 6) {
        throw std::runtime_error("ESTUN move target must contain exactly 6 values");
      }
      if (
        request->move_type != estun_codroid_bridge::srv::Move::Request::MOVE_JOINT &&
        request->move_type != estun_codroid_bridge::srv::Move::Request::MOVE_CARTESIAN_J &&
        request->move_type != estun_codroid_bridge::srv::Move::Request::MOVE_CARTESIAN_L)
      {
        throw std::runtime_error("unsupported ESTUN move type");
      }
      Json::Value data(Json::objectValue);
      data["type"] = request->move_type == estun_codroid_bridge::srv::Move::Request::MOVE_CARTESIAN_L ?
        "movl" : "movj";
      Json::Value & target = data["target"];
      if (request->move_type == estun_codroid_bridge::srv::Move::Request::MOVE_JOINT) {
        target["type"] = "apos";
        for (std::size_t index = 0; index < request->target.size(); ++index) {
          target["apos"]["jntpos" + std::to_string(index + 1)] = request->target[index];
        }
        target["apos"]["jntpos7"] = 0.0;
      } else {
        Json::Value cpos = cpos_from_pose(request->target);

        // Preserve the PointType::Cart contract implemented by CodroidApi:
        // movJ(PointType::Cart) and movL(PointType::Cart) both send
        // target.cpos directly.
        target["type"] = "cpos";
        target["cpos"] = cpos;
      }
      Json::Value speed(Json::objectValue);
      Json::Value acceleration(Json::objectValue);
      if (request->move_type == estun_codroid_bridge::srv::Move::Request::MOVE_CARTESIAN_L) {
        speed["sper"] = 0.0;
        speed["stcp"] = request->speed;
        speed["sori"] = request->speed;
        acceleration["aper"] = 0.0;
        acceleration["atcp"] = request->acceleration;
        acceleration["aori"] = request->acceleration;
      } else {
        speed["sper"] = request->speed;
        speed["stcp"] = 0.0;
        speed["sori"] = 0.0;
        acceleration["aper"] = request->acceleration;
        acceleration["atcp"] = 0.0;
        acceleration["aori"] = 0.0;
      }
      speed["sexjl"] = 0.0;
      speed["sexjr"] = 0.0;
      acceleration["aexjl"] = 0.0;
      acceleration["aexjr"] = 0.0;
      data["speed"] = speed;
      data["acc"] = acceleration;
      RCLCPP_INFO(
        get_logger(), "ESTUN Codroid move request: %s", json_string(data).c_str());
      // CodroidApi::movJ/movL use a 30 second request timeout by default.  A
      // controller may acknowledge only after completing a long motion, so the
      // generic 5 second query timeout would report failure after acceptance.
      controller_call("mov", data, motion_request_timeout_sec_);
      response->success = true;
      response->message = "motion accepted";
    } catch (const std::exception & error) {
      const std::string detail = error.what();
      // Some controller versions execute movJ/movL but never send the final
      // WebSocket reply. A completed write followed by this read timeout is an
      // indeterminate acknowledgement, not a motion rejection. The Python
      // executor verifies actual joint/Cartesian arrival after this service.
      if (detail == "Codroid read timed out") {
        response->success = true;
        response->message =
          "motion dispatched; Codroid reply timed out; verify arrival";
        RCLCPP_WARN(
          get_logger(),
          "Codroid motion reply timed out after dispatch; actual arrival "
          "verification is required");
      } else {
        response->success = false;
        response->message = detail;
        RCLCPP_ERROR(get_logger(), "%s", detail.c_str());
      }
    }
  }

  void set_do(
    const std::shared_ptr<estun_codroid_bridge::srv::SetDo::Request> & request,
    const std::shared_ptr<estun_codroid_bridge::srv::SetDo::Response> & response)
  {
    try {
      Json::Value data(Json::objectValue);
      data["port"] = request->port;
      data["val"] = request->value ? 1 : 0;
      controller_call("setDO", data);
      response->success = true;
      response->message = "DO set";
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "%s", error.what());
    }
  }

  std::string namespace_;
  double motion_request_timeout_sec_{30.0};
  std::unique_ptr<CodroidClient> client_;
  rclcpp::Service<estun_codroid_bridge::srv::GetPose>::SharedPtr get_pose_service_;
  rclcpp::Service<estun_codroid_bridge::srv::GetJoints>::SharedPtr get_joints_service_;
  rclcpp::Service<estun_codroid_bridge::srv::GetRobotState>::SharedPtr get_state_service_;
  rclcpp::Service<estun_codroid_bridge::srv::RobotCommand>::SharedPtr command_service_;
  rclcpp::Service<estun_codroid_bridge::srv::Move>::SharedPtr move_service_;
  rclcpp::Service<estun_codroid_bridge::srv::SetDo>::SharedPtr set_do_service_;
  rclcpp::Service<estun_codroid_bridge::srv::SolvePose>::SharedPtr solve_pose_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EstunCodroidBridge>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
