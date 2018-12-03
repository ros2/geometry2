// Copyright (c) 2018 Robert Bosch GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>
#include <memory>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <tf2_filter/tf2_filter.hpp>
#include <tf2_filter/tf2_filter_config.hpp>
#include <tf2_filter/tf2_filter_node.hpp>
#include <iostream>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::vector<std::string> frames;
  for (int i = 1; i < argc; ++i) {
    frames.emplace_back(argv[i]);
  }

  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("frames", rclcpp::ParameterValue(frames))
  };
  std::cout << "Initial parameters from CLI: " << params << std::endl;
  rclcpp::spin(std::make_shared<tf2_filter::TF2FilterNode>(
      "tf2_filter", "", false, params));

  rclcpp::shutdown();

  return 0;
}
