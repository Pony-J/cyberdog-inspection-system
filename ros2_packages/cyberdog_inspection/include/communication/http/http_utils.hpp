#pragma once

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/core/string.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <functional>
#include <iostream>
#include <memory>
#include <regex>
#include <string>
#include <thread>
#include <vector>

namespace communication::robot_http {

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;

template<class Allocator>
http::response<http::string_body> make_json_response(
  const nlohmann::json & body, http::status status,
  const http::request<http::string_body, http::basic_fields<Allocator>> & req)
{
  http::response<http::string_body> res{status, req.version()};
  res.set(http::field::server, "cyberdog-inspection-http");
  res.set(http::field::content_type, "application/json");
  res.keep_alive(req.keep_alive());
  res.body() = body.dump();
  res.prepare_payload();
  return res;
}

using Request = http::request<http::string_body>;
using Response = http::response<http::string_body>;
using Handler = std::function<Response(const Request &)>;

struct Route {
  std::regex path_regex;
  http::verb method;
  Handler handler;
};

class Router {
public:
  void add(http::verb method, const std::string & path_pattern, Handler handler)
  {
    routes_.push_back(Route{std::regex(path_pattern), method, std::move(handler)});
  }

  Response route(const Request & req) const
  {
    for (const auto & r : routes_) {
      if (r.method == req.method() && std::regex_match(std::string(req.target()), r.path_regex)) {
        return r.handler(req);
      }
    }
    return make_json_response(
      {{"success", false}, {"error", "Not Found"}, {"path", std::string(req.target())}},
      http::status::not_found, req);
  }

private:
  std::vector<Route> routes_;
};

class session : public std::enable_shared_from_this<session> {
public:
  session(tcp::socket socket, const Router & r)
  : socket_(std::move(socket)), router_(r) {}

  void run() { do_read(); }

private:
  void do_read()
  {
    auto self = shared_from_this();
    req_ = {};
    http::async_read(
      socket_, buffer_, req_,
      [self](beast::error_code ec, std::size_t) {
        if (ec == http::error::end_of_stream) {
          self->do_close();
          return;
        }
        if (ec) {
          std::cerr << "read error: " << ec.message() << "\n";
          return;
        }
        Response res = self->router_.route(self->req_);
        auto sp = std::make_shared<Response>(std::move(res));
        http::async_write(
          self->socket_, *sp,
          [self, sp](beast::error_code ec, std::size_t) {
            if (ec) {
              std::cerr << "write error: " << ec.message() << "\n";
              return;
            }
            if (!sp->keep_alive()) {
              self->do_close();
            } else {
              self->do_read();
            }
          });
      });
  }

  void do_close()
  {
    beast::error_code ec;
    socket_.shutdown(tcp::socket::shutdown_send, ec);
  }

  tcp::socket socket_;
  beast::flat_buffer buffer_;
  Request req_;
  const Router & router_;
};

class listener : public std::enable_shared_from_this<listener> {
public:
  listener(
    net::io_context & ioc, tcp::endpoint endpoint, const Router & r,
    const std::vector<std::string> & allowed_ips = {})
  : ioc_(ioc), acceptor_(ioc), router_(r), allowed_ips_(allowed_ips)
  {
    beast::error_code ec;
    acceptor_.open(endpoint.protocol(), ec);
    acceptor_.set_option(net::socket_base::reuse_address(true), ec);
    acceptor_.bind(endpoint, ec);
    acceptor_.listen(net::socket_base::max_listen_connections, ec);
  }

  void run() { do_accept(); }
  void stop()
  {
    beast::error_code ec;
    acceptor_.close(ec);
  }

private:
  void do_accept()
  {
    auto self = shared_from_this();
    acceptor_.async_accept(
      net::make_strand(ioc_),
      [self](beast::error_code ec, tcp::socket socket) {
        if (!ec) {
          const auto remote_ip = socket.remote_endpoint().address().to_string();
          if (self->allowed_ips_.empty() ||
            std::find(self->allowed_ips_.begin(), self->allowed_ips_.end(), remote_ip) !=
            self->allowed_ips_.end())
          {
            std::make_shared<session>(std::move(socket), self->router_)->run();
          } else {
            beast::error_code ec_shutdown;
            socket.shutdown(tcp::socket::shutdown_both, ec_shutdown);
            socket.close();
          }
        }
        if (self->acceptor_.is_open()) {
          self->do_accept();
        }
      });
  }

  net::io_context & ioc_;
  tcp::acceptor acceptor_;
  const Router & router_;
  std::vector<std::string> allowed_ips_;
};

class http_server {
  static constexpr int kNumThreads = 2;

public:
  explicit http_server(unsigned short port, const std::vector<std::string> & allowed_ips = {})
  : allowed_ips_(allowed_ips), port_(port) {}

  bool start()
  {
    try {
      ioc_ = std::make_shared<net::io_context>(kNumThreads);
      add_routes(router_);
      auto addr = net::ip::make_address("0.0.0.0");
      listener_ = std::make_shared<listener>(*ioc_, tcp::endpoint{addr, port_}, router_, allowed_ips_);
      listener_->run();
      for (int i = 0; i < kNumThreads; ++i) {
        server_threads_.emplace_back([this]() { ioc_->run(); });
      }
      return true;
    } catch (const std::exception & e) {
      std::cerr << "HTTP server start failed: " << e.what() << std::endl;
      return false;
    }
  }

  void stop()
  {
    if (ioc_) {
      ioc_->stop();
    }
    if (listener_) {
      listener_->stop();
    }
    for (auto & t : server_threads_) {
      if (t.joinable()) {
        t.join();
      }
    }
  }

protected:
  virtual void add_routes(Router & router) = 0;

private:
  std::vector<std::string> allowed_ips_;
  unsigned short port_;
  Router router_;
  std::vector<std::thread> server_threads_;
  std::shared_ptr<listener> listener_;
  std::shared_ptr<net::io_context> ioc_;
};

}  // namespace communication::robot_http
