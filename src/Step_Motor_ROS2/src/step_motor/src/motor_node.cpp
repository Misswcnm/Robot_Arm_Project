#include "rclcpp/rclcpp.hpp"
#include "step_motor/msg/motor.hpp"
#include "serial/serial.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

class Step_Motor:public rclcpp::Node
{
private:
  uint8_t dir{0};
  uint8_t sub_divide{0};
  uint8_t mode{0};
  std::string usart_port_name;
  uint32_t baudrate{115200};
  rclcpp::Subscription<step_motor::msg::Motor>::SharedPtr sub;
  rclcpp::Publisher<step_motor::msg::Motor>::SharedPtr pub;
  rclcpp::TimerBase::SharedPtr startup_close_timer;
  uint16_t speed{0};
  uint32_t angle{0};

  bool auto_close_on_startup{true};
  int startup_close_delay_ms{1000};
  int gripper_id{1};
  int close_speed{200};
  int close_dir{1};
  int close_mode{2};
  int close_angle{30000};
  int close_sub_divide{32};
  int closed_angle_min{29500};

  void fill_move_frame(const step_motor::msg::Motor &msg);
  void fill_query_frame(uint8_t id);
  bool write_frame();
  bool query_motor(uint8_t id, uint8_t &motor_state,
                   uint16_t &actual_speed, uint32_t &actual_angle);
  void publish_motor_state(uint8_t id, uint8_t motor_state,
                           uint16_t actual_speed, uint32_t actual_angle);
  void startup_close_once();
public:
  serial::Serial Stm32_Serial;
  uint8_t send_data[11];
  uint8_t recv_data[9];

  uint8_t check_rcc(uint8_t *data,uint8_t num);
  void motor_control_callback(
    const step_motor::msg::Motor::SharedPtr msg);
  Step_Motor(const std::string &node_name);
};

Step_Motor::Step_Motor(const std::string &node_name):Node(node_name)
{
  this->declare_parameter<std::string>("usart_port_name", "/dev/motor_serial");
  this->get_parameter("usart_port_name", usart_port_name);

  this->declare_parameter("serial_baud_rate",115200);
  this->get_parameter("serial_baud_rate", baudrate);

  auto_close_on_startup = this->declare_parameter<bool>(
    "auto_close_on_startup", true);
  startup_close_delay_ms = this->declare_parameter<int>(
    "startup_close_delay_ms", 1000);
  gripper_id = this->declare_parameter<int>("gripper_id", 1);
  close_speed = this->declare_parameter<int>("close_speed", 200);
  close_dir = this->declare_parameter<int>("close_dir", 1);
  close_mode = this->declare_parameter<int>("close_mode", 2);
  close_angle = this->declare_parameter<int>("close_angle", 30000);
  close_sub_divide = this->declare_parameter<int>("close_sub_divide", 32);
  closed_angle_min = this->declare_parameter<int>("closed_angle_min", 29500);


  pub=this->create_publisher<step_motor::msg::Motor>("motor_state",10);
  sub=this->create_subscription<step_motor::msg::Motor>("motor_control",10,std::bind(&Step_Motor::motor_control_callback,this,std::placeholders::_1));
  try
  { 
    //Attempts to initialize and open the serial port //尝试初始化与开启串口
    Stm32_Serial.setPort(usart_port_name); //Select the serial port number to enable //选择要开启的串口号
    Stm32_Serial.setBaudrate(baudrate); //Set the baud rate //设置波特率
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000); //Timeout //超时等待
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open(); //Open the serial port //开启串口
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR(this->get_logger(),"wheeltec_robot can not open serial port,Please check the serial port cable! "); //If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
  }
  if(Stm32_Serial.isOpen())
  {
    RCLCPP_INFO(this->get_logger(),"wheeltec_robot serial port opened"); //Serial port opened successfully //串口开启成功提示
    if (auto_close_on_startup) {
      startup_close_timer = this->create_wall_timer(
        std::chrono::milliseconds(std::max(0, startup_close_delay_ms)),
        std::bind(&Step_Motor::startup_close_once, this));
    }
  }
}

void Step_Motor::fill_move_frame(const step_motor::msg::Motor &msg)
{
  send_data[0]=0x7b;
  send_data[1]=msg.id;
  send_data[2]=msg.mode;
  mode=msg.mode;
  send_data[3]=msg.dir;
  dir=msg.dir;
  send_data[4]=msg.sub_divide;
  sub_divide=msg.sub_divide;
  send_data[5]=msg.angle>>8;
  send_data[6]=msg.angle;
  send_data[7]=msg.speed>>8;
  send_data[8]=msg.speed;
  send_data[9]=check_rcc(send_data,9);
  send_data[10]=0x7d;
}

void Step_Motor::fill_query_frame(uint8_t id)
{
  send_data[0]=0x7b;
  send_data[1]=id;
  std::fill(send_data + 2, send_data + 9, 0);
  send_data[9]=check_rcc(send_data,9);
  send_data[10]=0x7d;
}

bool Step_Motor::write_frame()
{
  if (!Stm32_Serial.isOpen()) {
    RCLCPP_ERROR(this->get_logger(), "motor serial port is not open");
    return false;
  }
  try {
    return Stm32_Serial.write(send_data, sizeof(send_data)) == sizeof(send_data);
  } catch (const serial::IOException &) {
    RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port");
    return false;
  }
}

bool Step_Motor::query_motor(uint8_t id, uint8_t &motor_state,
                             uint16_t &actual_speed, uint32_t &actual_angle)
{
  fill_query_frame(id);
  try {
    Stm32_Serial.flushInput();
    if (!write_frame()) {
      return false;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(20));
    const std::size_t received = Stm32_Serial.read(recv_data, sizeof(recv_data));
    if (received != sizeof(recv_data)) {
      RCLCPP_WARN(this->get_logger(),
                  "motor state response is incomplete: %zu/%zu bytes",
                  received, sizeof(recv_data));
      return false;
    }
    if (check_rcc(recv_data, 8) != recv_data[8]) {
      RCLCPP_WARN(this->get_logger(), "motor state response checksum failed");
      return false;
    }
    motor_state = recv_data[1];
    actual_speed = static_cast<uint16_t>(
      (static_cast<uint16_t>(recv_data[2]) << 8) | recv_data[3]);
    actual_angle =
      (static_cast<uint32_t>(recv_data[4]) << 24) |
      (static_cast<uint32_t>(recv_data[5]) << 16) |
      (static_cast<uint32_t>(recv_data[6]) << 8) |
      static_cast<uint32_t>(recv_data[7]);
    return true;
  } catch (const serial::IOException &) {
    RCLCPP_ERROR(this->get_logger(), "Unable to query motor state");
    return false;
  }
}

void Step_Motor::publish_motor_state(uint8_t id, uint8_t motor_state,
                                     uint16_t actual_speed, uint32_t actual_angle)
{
  step_motor::msg::Motor motor;
  motor.id=id;
  // Motor.msg currently exposes uint16 angle. Keep the wire-compatible field
  // while using the complete uint32 value internally for the startup decision.
  motor.angle=static_cast<uint16_t>(
    std::min<uint32_t>(actual_angle, UINT16_MAX));
  motor.state=motor_state;
  motor.speed=actual_speed;
  motor.dir=dir;
  motor.mode=mode;
  motor.sub_divide=sub_divide;
  pub->publish(motor);
}

void Step_Motor::startup_close_once()
{
  if (startup_close_timer) {
    startup_close_timer->cancel();
  }

  uint8_t motor_state=0;
  uint16_t actual_speed=0;
  uint32_t actual_angle=0;
  if (!query_motor(static_cast<uint8_t>(gripper_id), motor_state,
                   actual_speed, actual_angle)) {
    RCLCPP_ERROR(this->get_logger(),
      "[GRIPPER_STARTUP] state query failed; close command was NOT sent");
    return;
  }

  publish_motor_state(static_cast<uint8_t>(gripper_id), motor_state,
                      actual_speed, actual_angle);
  RCLCPP_INFO(this->get_logger(),
    "[GRIPPER_STARTUP] state=%u speed=%u angle=%u closed_angle_min=%d",
    motor_state, actual_speed, actual_angle, closed_angle_min);

  if (actual_angle >= static_cast<uint32_t>(std::max(0, closed_angle_min))) {
    RCLCPP_INFO(this->get_logger(),
      "[GRIPPER_STARTUP] gripper is already closed; no command sent");
    return;
  }

  step_motor::msg::Motor close;
  close.id=static_cast<uint8_t>(gripper_id);
  close.speed=static_cast<uint16_t>(close_speed);
  close.dir=static_cast<uint8_t>(close_dir);
  close.mode=static_cast<uint8_t>(close_mode);
  close.angle=static_cast<uint16_t>(close_angle);
  close.state=0;
  close.sub_divide=static_cast<uint8_t>(close_sub_divide);
  fill_move_frame(close);
  if (write_frame()) {
    RCLCPP_INFO(this->get_logger(),
      "[GRIPPER_STARTUP] gripper was not closed; one close command sent: "
      "id=%u speed=%u dir=%u mode=%u angle=%u sub_divide=%u",
      close.id, close.speed, close.dir, close.mode,
      close.angle, close.sub_divide);
  }
}

void Step_Motor::motor_control_callback(
  const step_motor::msg::Motor::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(),"%d \n",msg->dir); //Serial port opened successfully //串口开启成功提示
  if(msg->state == 0)
  {
    fill_move_frame(*msg);
    write_frame();
  }
  else if(msg->state == 1)
  {
    uint8_t motor_state=0;
    if (query_motor(msg->id, motor_state, speed, angle)) {
      publish_motor_state(msg->id, motor_state, speed, angle);
    }
  }
  else if(msg->state == 2)
  {
    startup_close_once();
  }
  else
  {
    RCLCPP_WARN(this->get_logger(),
      "unsupported motor_control state=%u (0=move, 1=query, 2=ensure closed)",
      msg->state);
  }
}

uint8_t Step_Motor::check_rcc(uint8_t *data,uint8_t num)
{
  uint8_t check=0;
  for(uint8_t i=0;i<num;i++)
  {
    check^=data[i];
  }
  return check;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Step_Motor>("motor_node");
  rclcpp::spin(node);  
  rclcpp::shutdown();
}
