/***********************************************************************
Copyright (c) 2022, www.guyuehome.com

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "take_pictures_node/take_pictures_node.h"

#include "stdio.h"

using take_pictures::TakePicturesNode;

int main(int argc, char** argv) {
    RCLCPP_WARN(rclcpp::get_logger("take_pictures"), "Start take pictures");

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opt;
    auto node = std::make_shared<TakePicturesNode>(opt);
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
