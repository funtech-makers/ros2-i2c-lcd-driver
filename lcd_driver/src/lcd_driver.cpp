#include "lcd_driver.hpp"


I2C_LCD::I2C_LCD() : Node("lcd_driver") {
  /* Init parametrers from YAML */
  init_parameters();

  /* Open I2C connection */
  #ifndef SIMULATION
  std::string i2c_device = "/dev/i2c-" + std::to_string(i2c_bus);
  lcd = std::make_shared<LiquidCrystal_I2C>(i2c_device.c_str(), i2c_addr, p_en, p_rw, p_rs, p_d4, p_d5, p_d6, p_d7, p_bl, POSITIVE);

  /* Initialize LCD */
  this->lcd->begin(lcd_cols, lcd_rows);
  this->lcd->on();
  this->lcd->clear();
  this->lcd->print(banner.c_str());
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
  this->declare_parameter("pins.en");
  this->declare_parameter("pins.rw");
  this->declare_parameter("pins.rs");
  this->declare_parameter("pins.d4");
  this->declare_parameter("pins.d5");
  this->declare_parameter("pins.d6");
  this->declare_parameter("pins.d7");
  this->declare_parameter("pins.backlight");
  this->declare_parameter("lcd.rows");
  this->declare_parameter("lcd.cols");
  this->declare_parameter("banner");


  // Get parameters from yaml
  this->get_parameter_or<int>("i2c_bus", i2c_bus, 6);
  this->get_parameter_or<int>("i2c_addr", i2c_addr, 0x27);

  this->get_parameter_or<int>("pins.en", p_en, 2);
  this->get_parameter_or<int>("pins.rw", p_rw, 1);
  this->get_parameter_or<int>("pins.rs", p_rs, 0);
  this->get_parameter_or<int>("pins.d4", p_d4, 4);
  this->get_parameter_or<int>("pins.d5", p_d5, 5);
  this->get_parameter_or<int>("pins.d6", p_d6, 6);
  this->get_parameter_or<int>("pins.d7", p_d7, 7);

  this->get_parameter_or<int>("pins.backlight", p_bl, 3);

  this->get_parameter_or<int>("lcd.rows", lcd_rows, 2);
  this->get_parameter_or<int>("lcd.cols", lcd_cols, 16);

  this->get_parameter_or<std::string>("banner", banner, "Hello Bot !");
}


void I2C_LCD::msg_text_callback(const lcd_msgs::msg::Lcd::SharedPtr msg) {
  #ifndef SIMULATION
  (msg->autoscroll)?this->lcd->autoscroll():this->lcd->noAutoscroll();
  this->lcd->setCursor(msg->column, msg->line);
  this->lcd->print(msg->text.c_str());
  #endif /* SIMULATION */
  RCLCPP_INFO(this->get_logger(), "cursor(%d, %d) : text(%s)", msg->column, msg->line, msg->text.c_str());
}

I2C_LCD::~I2C_LCD() {
  this->lcd->off();
  RCLCPP_INFO(this->get_logger(), "I2C_LCD Node Terminated");
}
