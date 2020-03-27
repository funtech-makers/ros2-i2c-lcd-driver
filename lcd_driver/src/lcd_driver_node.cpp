#include "lcd_driver.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto lcd_node = std::make_shared<I2C_LCD>();

  rclcpp::spin(lcd_node);
  rclcpp::shutdown();
  return 0;
}
