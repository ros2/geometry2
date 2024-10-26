/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "tf2_ros/static_transform_broadcaster_node.hpp"

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Vector3.hpp"

#include "rclcpp/rclcpp.hpp"

struct Option
{
  explicit Option(bool has_arg)
  : has_argument(has_arg)
  {
  }

  bool has_argument{false};

  virtual std::string visit(const std::string & optname, const std::string & stringval) = 0;

  virtual ~Option() {}
};

struct DoubleOption final : Option
{
  explicit DoubleOption(bool has_arg, std::function<void(double)> cb)
  : Option(has_arg),
    callback(cb)
  {
  }

  std::function<void(double)> callback;

  std::string visit(const std::string & optname, const std::string & stringval) override
  {
    double value;
    try {
      value = std::stod(stringval);
    } catch (const std::invalid_argument &) {
      return "Failed to parse " + optname + " argument as float";
    }

    callback(value);

    return "";
  }
};

struct StringOption final : Option
{
  explicit StringOption(bool has_arg, std::function<void(const std::string &)> cb)
  : Option(has_arg),
    callback(cb)
  {
  }

  std::function<void(const std::string &)> callback;

  std::string visit(const std::string & optname, const std::string & stringval) override
  {
    (void)optname;
    callback(stringval);
    return "";
  }
};

static std::string parse_args(
  const std::vector<std::string> & args,
  bool & help,
  tf2::Quaternion & quat,
  tf2::Vector3 & trans,
  std::string & frame_id,
  std::string & child_frame_id)
{
  size_t size = args.size();

  help = false;

  if (size < 1) {
    return "Not enough arguments to parse";
  }

  size_t last_index = size - 1;

  bool saw_frame_flag = false;
  bool saw_quat_flag = false;
  bool saw_rpy_flag = false;
  bool saw_trans_flag = false;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  auto qx_opt = std::make_shared<DoubleOption>(
    true, [&quat, &saw_quat_flag](double value) {
      quat.setX(value);
      saw_quat_flag = true;
    });

  auto qy_opt = std::make_shared<DoubleOption>(
    true, [&quat, &saw_quat_flag](double value) {
      quat.setY(value);
      saw_quat_flag = true;
    });

  auto qz_opt = std::make_shared<DoubleOption>(
    true, [&quat, &saw_quat_flag](double value) {
      quat.setZ(value);
      saw_quat_flag = true;
    });

  auto qw_opt = std::make_shared<DoubleOption>(
    true, [&quat, &saw_quat_flag](double value) {
      quat.setW(value);
      saw_quat_flag = true;
    });

  auto roll_opt = std::make_shared<DoubleOption>(
    true, [&roll, &saw_rpy_flag](double value) {
      roll = value;
      saw_rpy_flag = true;
    });

  auto pitch_opt = std::make_shared<DoubleOption>(
    true, [&pitch, &saw_rpy_flag](double value) {
      pitch = value;
      saw_rpy_flag = true;
    });

  auto yaw_opt = std::make_shared<DoubleOption>(
    true, [&yaw, &saw_rpy_flag](double value) {
      yaw = value;
      saw_rpy_flag = true;
    });

  auto trans_x_opt = std::make_shared<DoubleOption>(
    true, [&trans, &saw_trans_flag](double value) {
      trans.setX(value);
      saw_trans_flag = true;
    });

  auto trans_y_opt = std::make_shared<DoubleOption>(
    true, [&trans, &saw_trans_flag](double value) {
      trans.setY(value);
      saw_trans_flag = true;
    });

  auto trans_z_opt = std::make_shared<DoubleOption>(
    true, [&trans, &saw_trans_flag](double value) {
      trans.setZ(value);
      saw_trans_flag = true;
    });

  auto frame_id_opt = std::make_shared<StringOption>(
    true, [&frame_id, &saw_frame_flag](const std::string & value) {
      frame_id = value;
      saw_frame_flag = true;
    });

  auto child_frame_id_opt = std::make_shared<StringOption>(
    true, [&child_frame_id, &saw_frame_flag](const std::string & value) {
      child_frame_id = value;
      saw_frame_flag = true;
    });

  auto help_opt = std::make_shared<StringOption>(
    false, [&help](const std::string & value) {
      (void)value;
      help = true;
    });

  std::unordered_map<std::string, std::shared_ptr<Option>> options = {
    {"--qx", qx_opt},
    {"--qy", qy_opt},
    {"--qz", qz_opt},
    {"--qw", qw_opt},
    {"--roll", roll_opt},
    {"--pitch", pitch_opt},
    {"--yaw", yaw_opt},
    {"--x", trans_x_opt},
    {"--y", trans_y_opt},
    {"--z", trans_z_opt},
    {"--frame-id", frame_id_opt},
    {"--child-frame-id", child_frame_id_opt},
    {"--help", help_opt},
    {"-h", help_opt},
  };

  std::vector<std::string> no_flag_args;

  size_t i = 1;
  while (i < size) {
    const std::string & optname = args[i];
    if (options.count(optname) == 0) {
      no_flag_args.push_back(optname);
    } else {
      std::shared_ptr<Option> opt = options[optname];
      if (opt->has_argument) {
        if (i == last_index) {
          return "Not enough arguments for " + optname;
        }

        ++i;
      }
      std::string result = opt->visit(optname, args[i]);
      if (result != "") {
        return result;
      }
    }
    ++i;
  }

  if (help) {
    return "";
  }

  if (saw_rpy_flag && saw_quat_flag) {
    return "Cannot specify both quaternion and Euler rotations";
  } else if (saw_rpy_flag) {
    quat.setRPY(roll, pitch, yaw);
  }

  if (no_flag_args.size() == 8 || no_flag_args.size() == 9) {
    RCUTILS_LOG_WARN("Old-style arguments are deprecated; see --help for new-style arguments");
    if (saw_frame_flag || saw_trans_flag || saw_quat_flag) {
      return "Cannot specify both new-style (flags) and old-style (arguments)";
    }

    std::string ret;

    ret = trans_x_opt->visit("x", no_flag_args[0]);
    if (ret != "") {
      return ret;
    }
    ret = trans_y_opt->visit("y", no_flag_args[1]);
    if (ret != "") {
      return ret;
    }
    ret = trans_z_opt->visit("z", no_flag_args[2]);
    if (ret != "") {
      return ret;
    }

    if (no_flag_args.size() == 8) {
      ret = yaw_opt->visit("yaw", no_flag_args[3]);
      if (ret != "") {
        return ret;
      }
      ret = pitch_opt->visit("pitch", no_flag_args[4]);
      if (ret != "") {
        return ret;
      }
      ret = roll_opt->visit("roll", no_flag_args[5]);
      if (ret != "") {
        return ret;
      }

      quat.setRPY(roll, pitch, yaw);
      frame_id = no_flag_args[6];
      child_frame_id = no_flag_args[7];
    } else {
      ret = qx_opt->visit("qx", no_flag_args[3]);
      if (ret != "") {
        return ret;
      }
      ret = qy_opt->visit("qy", no_flag_args[4]);
      if (ret != "") {
        return ret;
      }
      ret = qz_opt->visit("qz", no_flag_args[5]);
      if (ret != "") {
        return ret;
      }
      ret = qw_opt->visit("qw", no_flag_args[6]);
      if (ret != "") {
        return ret;
      }
      frame_id = no_flag_args[7];
      child_frame_id = no_flag_args[8];
    }
  } else if (no_flag_args.size() != 0) {
    return "Extra unparsed arguments on command-line";
  }

  if (frame_id == "") {
    return "Frame id must not be empty";
  }

  if (child_frame_id == "") {
    return "Child frame id must not be empty";
  }

  return "";
}

static void print_usage()
{
  const char * usage =
    "usage: static_transform_publisher [--x X] [--y Y] [--z Z] [--qx QX] [--qy QY] "
    "[--qz QZ] [--qw QW] [--roll ROLL] [--pitch PITCH] [--yaw YAW] --frame-id FRAME_ID "
    "--child-frame-id CHILD_FRAME_ID\n\n"
    "A command line utility for manually sending a static transform.\n\nIf no translation or"
    " orientation is provided, the identity transform will be published.\n\nThe translation offsets"
    " are in meters.\n\nThe rotation may be provided with roll, pitch, yaw euler angles in radians,"
    " or as a quaternion.\n\n"
    "required arguments:\n"
    "  --frame-id FRAME_ID parent frame\n"
    "  --child-frame-id CHILD_FRAME_ID child frame id\n\n"
    "optional arguments:\n"
    "  --x X                 x component of translation\n"
    "  --y Y                 y component of translation\n"
    "  --z Z                 z component of translation\n"
    "  --qx QX               x component of quaternion rotation\n"
    "  --qy QY               y component of quaternion rotation\n"
    "  --qz QZ               z component of quaternion rotation\n"
    "  --qw QW               w component of quaternion rotation\n"
    "  --roll ROLL           roll component Euler rotation\n"
    "  --pitch PITCH         pitch component Euler rotation\n"
    "  --yaw YAW             yaw component Euler rotation";
  printf("%s\n", usage);
}

int main(int argc, char ** argv)
{
  // Initialize ROS
  std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  bool help = false;
  tf2::Quaternion rotation(0.0, 0.0, 0.0, 1.0);
  tf2::Vector3 translation(0.0, 0.0, 0.0);
  std::string frame_id;
  std::string child_frame_id;

  std::string ret = parse_args(args, help, rotation, translation, frame_id, child_frame_id);
  if (ret != "") {
    RCUTILS_LOG_ERROR("error parsing command line arguments: %s", ret.c_str());
    print_usage();
    return 1;
  }
  if (help) {
    print_usage();
    return 0;
  }

  rclcpp::NodeOptions options;
  // override default parameters with the desired transform
  options.parameter_overrides(
  {
    {"translation.x", translation.x()},
    {"translation.y", translation.y()},
    {"translation.z", translation.z()},
    {"rotation.x", rotation.x()},
    {"rotation.y", rotation.y()},
    {"rotation.z", rotation.z()},
    {"rotation.w", rotation.w()},
    {"frame_id", frame_id},
    {"child_frame_id", child_frame_id},
  });

  std::shared_ptr<tf2_ros::StaticTransformBroadcasterNode> node;

  node = std::make_shared<tf2_ros::StaticTransformBroadcasterNode>(options);

  RCLCPP_INFO(
    node->get_logger(),
    "Spinning until stopped - publishing transform\ntranslation: ('%lf', '%lf', '%lf')\n"
    "rotation: ('%lf', '%lf', '%lf', '%lf')\nfrom '%s' to '%s'",
    translation.x(), translation.y(), translation.z(),
    rotation.x(), rotation.y(), rotation.z(), rotation.w(),
    frame_id.c_str(), child_frame_id.c_str());

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
