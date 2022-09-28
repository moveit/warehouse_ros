// Copyright 2015 Fetch Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Fetch Robotics nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/* Author: Connor Brew */

#include <warehouse_ros/database_loader.h>

namespace warehouse_ros
{
namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("warehouse_ros.database_loader");

// Parameter names
const std::string WAREHOUSE_PLUGIN = "warehouse_plugin";
const std::string WAREHOUSE_HOST = "warehouse_host";
const std::string WAREHOUSE_PORT = "warehouse_port";

// Default values
const std::string WAREHOUSE_PLUGIN_DEFAULT = "warehouse_ros_mongo::MongoDatabaseConnection";
const std::string WAREHOUSE_HOST_DEFAULT = "localhost";
constexpr auto WAREHOUSE_PORT_DEFAULT = 33829;
}  // namespace

DatabaseLoader::DatabaseLoader(const rclcpp::Node::SharedPtr& node) : node_(node)
{
  // Create the plugin loader.
  try
  {
    db_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<DatabaseConnection>>("warehouse_ros", "warehouse_ros::"
                                                                                                      "DatabaseConnecti"
                                                                                                      "on");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_FATAL_STREAM(LOGGER, "Exception while creating database_connection plugin loader " << ex.what());
  }
}

typename DatabaseConnection::Ptr DatabaseLoader::loadDatabase()
{
  if (!db_plugin_loader_)
  {
    return typename DatabaseConnection::Ptr(new DBConnectionStub());
  }

  // Search for the warehouse_plugin parameter in the local namespace of the node, and up the tree of namespaces.
  // If the desired param is not found, make a final attempt to look for the param in the default namespace
  std::string db_plugin = WAREHOUSE_PLUGIN_DEFAULT;
  if (!node_->get_parameter_or(WAREHOUSE_PLUGIN, db_plugin, WAREHOUSE_PLUGIN_DEFAULT))
  {
    RCLCPP_ERROR(LOGGER, "Could not find parameter '%s' using default '%s'", WAREHOUSE_PLUGIN.c_str(),
                 db_plugin.c_str());
  }
  try
  {
    DatabaseConnection::Ptr db = db_plugin_loader_->createUniqueInstance(db_plugin);

    // Get and set host name and port
    std::string host = WAREHOUSE_HOST_DEFAULT;
    if (!node_->get_parameter_or(WAREHOUSE_HOST, host, WAREHOUSE_HOST_DEFAULT))
    {
      RCLCPP_ERROR(LOGGER, "Could not find parameter '%s' using default '%s'", WAREHOUSE_HOST.c_str(), host.c_str());
    }
    int port = WAREHOUSE_PORT_DEFAULT;
    if (!node_->get_parameter_or(WAREHOUSE_PORT, port, WAREHOUSE_PORT_DEFAULT))
    {
      RCLCPP_ERROR(LOGGER, "Could not find parameter '%s' using default '%i'", WAREHOUSE_PORT.c_str(), port);
    }

    // If successful return database pointer
    if (db->setParams(host, port))
    {
      return db;
    }
    return typename DatabaseConnection::Ptr(new DBConnectionStub());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR_STREAM(LOGGER,
                        "Exception while loading database plugin '" << db_plugin << "': " << ex.what() << std::endl);
    return typename DatabaseConnection::Ptr(new DBConnectionStub());
  }
}

MessageCollectionHelper::Ptr DBConnectionStub::openCollectionHelper(const std::string& /*db_name*/,
                                                                    const std::string& /*collection_name*/)
{
  return MessageCollectionHelper::Ptr();
}
}  // namespace warehouse_ros
