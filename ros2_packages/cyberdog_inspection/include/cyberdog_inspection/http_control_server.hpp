#pragma once

#include <nlohmann/json.hpp>

#include "communication/http/http_utils.hpp"
#include "cyberdog_inspection/local_inspection.hpp"

namespace cyberdog_inspection {

class HttpControlServer : public communication::robot_http::http_server {
public:
  HttpControlServer(
    LocalInspectionService & service, unsigned short port,
    const std::vector<std::string> & allowed_ips = {})
  : communication::robot_http::http_server(port, allowed_ips), service_(service) {}

private:
  using Request = communication::robot_http::Request;
  using Response = communication::robot_http::Response;
  using Router = communication::robot_http::Router;

  void add_routes(Router & router) override
  {
    router.add(communication::robot_http::http::verb::get, "/inspection/internal/maps", [this](const Request & req) {
      return get_maps(req);
    });
    router.add(communication::robot_http::http::verb::get, "/inspection/internal/planned_path", [this](const Request & req) {
      return get_planned_path(req);
    });
    router.add(communication::robot_http::http::verb::get, "/inspection/internal/map_artifacts", [this](const Request & req) {
      return get_map_artifacts(req);
    });
    router.add(communication::robot_http::http::verb::post, "/inspection/internal/start_initialization", [this](const Request & req) {
      return start_initialization(req);
    });
    router.add(communication::robot_http::http::verb::post, "/inspection/internal/start_inspection", [this](const Request & req) {
      return call_noarg(req, [this](std::string * msg) { return service_.start_inspection(msg); });
    });
    router.add(communication::robot_http::http::verb::post, "/inspection/internal/pause_inspection", [this](const Request & req) {
      return call_noarg(req, [this](std::string * msg) { return service_.pause_inspection(msg); });
    });
    router.add(communication::robot_http::http::verb::post, "/inspection/internal/resume_inspection", [this](const Request & req) {
      return call_noarg(req, [this](std::string * msg) { return service_.resume_inspection(msg); });
    });
    router.add(communication::robot_http::http::verb::post, "/inspection/internal/stop_inspection", [this](const Request & req) {
      return call_noarg(req, [this](std::string * msg) { return service_.stop_inspection(msg); });
    });
    router.add(communication::robot_http::http::verb::post, "/inspection/internal/reset", [this](const Request & req) {
      return call_noarg(req, [this](std::string * msg) { return service_.reset(msg); });
    });
    router.add(communication::robot_http::http::verb::get, "/inspection/internal/status", [this](const Request & req) {
      return get_status(req);
    });
    router.add(communication::robot_http::http::verb::post, "/inspection/internal/status", [this](const Request & req) {
      return get_status(req);
    });
  }

  Response start_initialization(const Request & req)
  {
    try {
      std::string scene_name = "map";
      if (!req.body().empty()) {
        auto j = nlohmann::json::parse(req.body());
        scene_name = j.value("scene_name", std::string("map"));
      }
      if (scene_name.empty()) {
        scene_name = "map";
      }
      std::string message;
      bool ok = service_.initialize(scene_name, &message);
      return communication::robot_http::make_json_response(
        {{"success", ok}, {"message", message}, {"scene_name", scene_name}},
        communication::robot_http::http::status::ok, req);
    } catch (const std::exception & e) {
      return communication::robot_http::make_json_response(
        {{"success", false}, {"message", e.what()}},
        communication::robot_http::http::status::bad_request, req);
    }
  }

  template<typename Fn>
  Response call_noarg(const Request & req, Fn fn)
  {
    try {
      std::string message;
      bool ok = fn(&message);
      return communication::robot_http::make_json_response(
        {{"success", ok}, {"message", message}},
        communication::robot_http::http::status::ok, req);
    } catch (const std::exception & e) {
      return communication::robot_http::make_json_response(
        {{"success", false}, {"message", e.what()}},
        communication::robot_http::http::status::bad_request, req);
    }
  }

  Response get_status(const Request & req)
  {
    return communication::robot_http::make_json_response(
      {
        {"success", true},
        {"inspection_status", static_cast<int>(service_.status())},
        {"inspection_status_name", service_.status_name()},
        {"message", service_.status_message()},
        {"nav2_status", service_.nav2_status()},
        {"active_scene_name", service_.active_scene_name()},
      },
      communication::robot_http::http::status::ok, req);
  }

  Response get_maps(const Request & req)
  {
    nlohmann::json maps = nlohmann::json::array();
    for (const auto & map : service_.list_maps()) {
      maps.push_back({
        {"scene_name", map.scene_name},
        {"yaml_path", map.yaml_path},
        {"pgm_path", map.pgm_path},
        {"available", map.available},
        {"resolution", map.resolution},
        {"origin", map.origin},
        {"width", map.width},
        {"height", map.height},
        {"error_message", map.error_message},
      });
    }

    return communication::robot_http::make_json_response(
      {
        {"success", true},
        {"maps", maps},
      },
      communication::robot_http::http::status::ok, req);
  }

  Response get_planned_path(const Request & req)
  {
    return communication::robot_http::make_json_response(
      {
        {"success", true},
        {"active_scene_name", service_.active_scene_name()},
        {"planned_path", service_.planned_path()},
      },
      communication::robot_http::http::status::ok, req);
  }

  Response get_map_artifacts(const Request & req)
  {
    const auto artifacts = service_.map_artifacts();
    return communication::robot_http::make_json_response(
      {
        {"success", true},
        {"scene_name", artifacts.scene_name},
        {"latest_gif_path", artifacts.latest_gif_path},
        {"debug_map_paths", artifacts.debug_map_paths},
      },
      communication::robot_http::http::status::ok, req);
  }

  LocalInspectionService & service_;
};

}  // namespace cyberdog_inspection
