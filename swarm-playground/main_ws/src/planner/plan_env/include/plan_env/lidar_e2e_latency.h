// LiDAR-E2E per-cycle compute records on /lidar_e2e/latency as {"record":{kind,node,rec,...}};
// `rec` counts each publisher's records from 0 so a gap means a dropped message. Nothing is
// advertised, timed into a message, or published unless /lidar_e2e/publish_latency is true.
#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>

namespace lidar_e2e
{
  using Clock = std::chrono::steady_clock;

  inline double msBetween(Clock::time_point from, Clock::time_point to)
  {
    return std::chrono::duration<double, std::milli>(to - from).count();
  }

  // Comma-joined JSON members; string values come from fixed vocabularies and are not escaped.
  class Json
  {
  public:
    Json &num(const char *key, double value) { return add(key, "%.6f", value); }
    Json &integer(const char *key, long long value) { return add(key, "%lld", value); }
    Json &u64(const char *key, uint64_t value) { return add(key, "%llu", (unsigned long long)value); }
    Json &boolean(const char *key, bool value) { return add(key, "%s", value ? "true" : "false"); }
    Json &str(const char *key, const char *value) { return add(key, "\"%s\"", value); }
    Json &raw(const char *key, const std::string &json) { return add(key, "%s", json.c_str()); }
    const std::string &body() const { return body_; }

  private:
    template <typename T>
    Json &add(const char *key, const char *format, T value)
    {
      char buf[64];
      snprintf(buf, sizeof(buf), format, value);
      if (!body_.empty())
        body_ += ',';
      body_ += '"';
      body_ += key;
      body_ += "\":";
      body_ += buf;
      return *this;
    }
    Json &add(const char *key, const char *format, const char *value)
    {
      std::string formatted(snprintf(nullptr, 0, format, value), '\0');
      snprintf(&formatted[0], formatted.size() + 1, format, value);
      if (!body_.empty())
        body_ += ',';
      body_ += '"';
      body_ += key;
      body_ += "\":";
      body_ += formatted;
      return *this;
    }
    std::string body_;
  };

  class Publisher
  {
  public:
    void init(ros::NodeHandle &nh, const char *node)
    {
      ros::param::param("/lidar_e2e/publish_latency", enabled_, false);
      node_ = node;
      if (enabled_)
        pub_ = nh.advertise<std_msgs::String>("/lidar_e2e/latency", 1000);
    }
    bool enabled() const { return enabled_; }
    void publish(const char *kind, const Json &fields)
    {
      if (!enabled_)
        return;
      Json head;
      head.str("kind", kind).str("node", node_.c_str()).u64("rec", rec_++);
      std_msgs::String msg;
      msg.data = "{\"record\":{" + head.body();
      if (!fields.body().empty())
        msg.data += "," + fields.body();
      msg.data += "}}";
      pub_.publish(msg);
    }

  private:
    bool enabled_ = false;
    std::string node_;
    std::atomic<uint64_t> rec_{0};
    ros::Publisher pub_;
  };

  // Runs `fn` on every scope exit (early returns included); the record publishes from there.
  template <typename F>
  class ScopeExit
  {
  public:
    explicit ScopeExit(F fn) : fn_(std::move(fn)) {}
    ScopeExit(ScopeExit &&other) : fn_(std::move(other.fn_)), active_(other.active_) { other.active_ = false; }
    ScopeExit(const ScopeExit &) = delete;
    ScopeExit &operator=(const ScopeExit &) = delete;
    ~ScopeExit()
    {
      if (active_)
        fn_();
    }

  private:
    F fn_;
    bool active_ = true;
  };

  template <typename F>
  ScopeExit<F> onExit(F fn) { return ScopeExit<F>(std::move(fn)); }

  inline uint64_t stampNs(const ros::Time &stamp) { return stamp.toNSec(); }
} // namespace lidar_e2e
