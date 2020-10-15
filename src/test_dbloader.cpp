#include <warehouse_ros/database_loader.h>
#include <rclcpp/rclcpp.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("warehouse_ros.test_dbloader");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("db_loader_test", node_options);

  warehouse_ros::DatabaseLoader dbloader(node);
  warehouse_ros::DatabaseConnection::Ptr conn = dbloader.loadDatabase();
  // conn->setParams("localhost", 27017, 10.0);
  conn->setTimeout(10.0);
  if (!conn->connect())
    RCLCPP_ERROR(LOGGER, "Failed to connect to DB");

  rclcpp::spin(node);
}