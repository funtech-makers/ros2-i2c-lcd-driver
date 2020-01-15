#include "lcd_driver.hpp"


I2C_LCD::I2C_LCD() : Node("lcd_driver") {
  /* Init parametrers from YAML */
  init_parameters();

  /* Give variables their initial values */
  init_lcd();

  /* Open I2C connection */
  #ifndef SIMULATION
  std::string i2c_device = "/dev/i2c-" + std::to_string(i2c_bus);
  lcd = std::make_shared<LiquidCrystal_I2C>(i2c_device.c_str(), i2c_addr);
  #endif /* SIMULATION */

  /* Init ROS Publishers and Subscribers */
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  lcd_driver_text_sub_ = this->create_subscription<lcd_msgs::msg::Lcd>("lcd", qos, std::bind(&I2C_LCD::msg_text_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "LCD I2C driver initialised");
}


void I2C_LCD::init_parameters() {
  // Declare parameters that may be set on this node
  this->declare_parameter("i2c_bus");
  this->declare_parameter("i2c_addr");

  // Get parameters from yaml
  this->get_parameter_or<int>("i2c_bus", i2c_bus, 6);
  this->get_parameter_or<int>("i2c_addr", i2c_addr, 0x27);
}


void I2C_LCD::init_lcd() {
  /* Initialize LCD */
  this->lcd.begin(16, 2);

  this->lcd.on();
  this->lcd.clear();
}

void I2C_LCD::msg_text_callback(const lcd_msgs::msg::Lcd::SharedPtr msg) {
  #ifndef SIMULATION
  (msg->autoscroll)?this->lcd->autoscroll():this->lcd->noAutoscroll();
  this->lcd->setCursor(msg->line, msg->column);
  this->lcd->print(msg->text.c_str());
  #else
  RCLCPP_INFO(this->get_logger(), "cursor(%d, %d) : text(%s)", msg->line, msg->column, msg->text.c_str());
  #endif /* SIMULATION */
}

I2C_LCD::~I2C_LCD() {
  RCLCPP_INFO(this->get_logger(), "I2C_LCD Node Terminated");
}
