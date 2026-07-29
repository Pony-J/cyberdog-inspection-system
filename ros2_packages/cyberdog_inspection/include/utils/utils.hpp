#pragma once

#include <filesystem>
#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>

namespace utils {

inline int8_t get_pgm_from_yaml_ros(const std::string & yaml_path, std::string & pgm_path)
{
  try {
    YAML::Node config = YAML::LoadFile(yaml_path);
    if (!config["image"]) {
      std::cerr << "Error: 'image' field not found in YAML file." << std::endl;
      return -1;
    }

    std::string image_path = config["image"].as<std::string>();
    if (!image_path.empty() && image_path.front() != '/') {
      std::string dir;
      const size_t last_slash = yaml_path.find_last_of("/\\");
      if (last_slash != std::string::npos) {
        dir = yaml_path.substr(0, last_slash + 1);
      }
      pgm_path = dir + image_path;
    } else {
      pgm_path = image_path;
    }
    return 0;
  } catch (const YAML::Exception & e) {
    std::cerr << "YAML parsing error: " << e.what() << std::endl;
    return -2;
  }
}

inline int8_t check_file_existence(const std::string & file_path)
{
  if (std::filesystem::exists(file_path) && std::filesystem::is_regular_file(file_path)) {
    return 0;
  }
  return -1;
}

}  // namespace utils
