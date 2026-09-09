#pragma once

#include <yaml-cpp/yaml.h>

#include <functional>
#include <stdexcept> //standard exceptions
#include <string>


namespace user_common_330371063_324976703 {
using namespace common;


//key - specific field in yaml, node - chunk of yaml file, default_val - what to use if the specific key is missing
template<typename T>
T get_with_default(const YAML::Node& node, const std::string& key, const T& default_val) {
    if (node[key]) {
        return node[key].as<T>(); //as - built-in tool in yaml for conversions
    }
    return default_val;
}

template<typename T>
T get_with_check(const YAML::Node& node, 
                 const std::string& key, 
                 const T& default_val, 
                 const std::function<bool(const T&)>& check_fn, //custom validation functions. the function returns bool and takes const T&
                 const std::string& err_msg) {
    T val = get_with_default(node, key, default_val);
    if (!check_fn(val)) {
        throw std::invalid_argument("Config value check failed for '" + key + "': " + err_msg);
    }
    return val;
}



} // namespace user_common_330371063_324976703

//used to parse all configurations (drone, lidar, mission...)