#ifndef I2C_LCD_HEADER_HPP
#define I2C_LCD_HEADER_HPP

#include <string>
#include <rclcpp/rclcpp.hpp>
#include "lcd_msgs/msg/lcd.hpp"

#ifndef SIMULATION
#include "LiquidCrystal_I2C.h"
#endif /* SIMULATION */



class I2C_LCD : public rclcpp::Node {
public:
  I2C_LCD();
  ~I2C_LCD();

private:
  int i2c_bus;
  int i2c_addr;
  #ifndef SIMULATION
  // For communicating with I2C_LCD over I2C
  std::shared_ptr<LiquidCrystal_I2C> lcd;
  #endif /* SIMULAtION */

  // ROS topic publishers
  rclcpp::Subscription<lcd_msgs::msg::Lcd>::SharedPtr lcd_driver_text_sub_;

  void init_variables();
  void init_lcd();
  void msg_text_callback(const lcd_msgs::msg::Lcd::SharedPtr msg);
};

#endif /* I2C_LCD_HEADER_HPP */
