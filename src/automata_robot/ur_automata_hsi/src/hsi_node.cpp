// hsi_node: hyperspectral measurement after the 3D scan. From home only, it
// puts the HSI frame above the scan center, asks the HSI app for the
// measurement, waits for it and goes back home. See ur_automata_hsi/README.md.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "ur_automata_scan/planning_scene_client.hpp"

using MoveGroup = moveit::planning_interface::MoveGroupInterface;

static std::atomic<bool> g_running{false};       // a measurement is in progress
static std::atomic<bool> g_hsi_done{false};      // the HSI app finished the measurement we wait for
static std::mutex g_mutex;
static std::string g_start_object;               // object name from start_topic, "" = none
static std::string g_wait_object;                // object of the measurement we wait for, "" = none

// HSI app state (JSON): sets g_hsi_done on measure_complete for g_wait_object.
static void on_hsi_status(const std_msgs::msg::String::SharedPtr msg)
{
  nlohmann::json status = nlohmann::json::parse(msg->data, nullptr, false);   // false: no exceptions
  if (status.is_discarded() || !status.is_object()) {
    std::printf("Stato HSI non valido (non JSON): %s\n", msg->data.c_str());
    std::fflush(stdout);
    return;
  }
  if (!status.contains("state") || !status["state"].is_string()) return;
  if (status["state"].get<std::string>() != "measure_complete") return;
  if (!status.contains("object") || !status["object"].is_string()) return;

  std::lock_guard<std::mutex> lock(g_mutex);
  if (!g_wait_object.empty() && status["object"].get<std::string>() == g_wait_object) {
    g_hsi_done = true;
  }
}

// Pilz PTP (straight line in joint space), OMPL if that line collides.
static bool move_to_joints(MoveGroup & move_group, const std::vector<double> & target)
{
  move_group.setStartStateToCurrentState();
  move_group.setJointValueTarget(target);
  const std::string pipelines[2][2] = {{"pilz_industrial_motion_planner", "PTP"},
                                       {"ompl", "RRTConnectkConfigDefault"}};
  for (const auto & pipeline : pipelines) {
    move_group.setPlanningPipelineId(pipeline[0]);
    move_group.setPlannerId(pipeline[1]);
    MoveGroup::Plan plan;
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      std::printf("  pianificato con %s %s\n", pipeline[0].c_str(), pipeline[1].c_str());
      std::fflush(stdout);
      return move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }
  }
  return false;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node   = rclcpp::Node::make_shared("hsi_node");
  auto logger = node->get_logger();
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_WARN);

  node->declare_parameter<std::string>("planning_group",    "ur_manipulator");
  node->declare_parameter<std::string>("end_effector_link", "ee_automata_tcp");   // IK tip (camera TCP)
  node->declare_parameter<std::string>("hsi_link",          "ee_automata_hsi");
  node->declare_parameter<std::string>("global_frame",      "world");
  node->declare_parameter<std::string>("home_pose_name",    "home");
  node->declare_parameter<std::vector<double>>("scan_center", {0.0, 0.0, 0.0});
  node->declare_parameter<double>("distance",               0.30);   // m, HSI frame above scan_center
  // Same speed as the scan, which slows a plan made at trajectory_scaling_factor:
  // velocities x speed, accelerations x speed^2.
  node->declare_parameter<double>("trajectory_scaling_factor", 0.4);
  node->declare_parameter<double>("speed",                  0.4);    // 1.0 = trajectory_scaling_factor (max 1.0)
  node->declare_parameter<std::string>("start_topic",       "/hsi/start");
  node->declare_parameter<std::string>("acquire_topic",     "/hsi/acquire");
  node->declare_parameter<std::string>("status_topic",      "/hsi/status");
  node->declare_parameter<std::string>("done_topic",        "/hsi/done");
  node->declare_parameter<bool>("wait_for_hsi",             true);   // false: tests without the HSI app
  node->declare_parameter<double>("wait_s",                 3.0);    // stop at the pose with wait_for_hsi false

  const std::string planning_group = node->get_parameter("planning_group").as_string();
  const std::string ee_link        = node->get_parameter("end_effector_link").as_string();
  const std::string hsi_link       = node->get_parameter("hsi_link").as_string();
  const std::string home_pose_name = node->get_parameter("home_pose_name").as_string();
  const std::vector<double> center = node->get_parameter("scan_center").as_double_array();
  const double distance            = node->get_parameter("distance").as_double();
  const double tsf                 = node->get_parameter("trajectory_scaling_factor").as_double();
  const double speed               = node->get_parameter("speed").as_double();
  const std::string start_topic    = node->get_parameter("start_topic").as_string();
  const std::string acquire_topic  = node->get_parameter("acquire_topic").as_string();
  const std::string status_topic   = node->get_parameter("status_topic").as_string();
  const std::string done_topic     = node->get_parameter("done_topic").as_string();
  const bool wait_for_hsi          = node->get_parameter("wait_for_hsi").as_bool();
  const double wait_s              = node->get_parameter("wait_s").as_double();

  const double home_tolerance = 0.02;   // rad, max joint difference to count as "in home"
  const double yaw_step_deg   = 10.0;   // rotations about the vertical axis tried at the pole
  const double ik_timeout     = 0.05;   // s per IK call

  if (speed <= 0.0 || speed > 1.0) {
    std::fprintf(stderr, "speed deve essere tra 0 (escluso) e 1.0, non %.2f.\n", speed);
    rclcpp::shutdown();
    return 1;
  }
  const double velocity_scaling     = tsf * speed;
  const double acceleration_scaling = tsf * speed * speed;

  if (center.size() != 3) {
    std::fprintf(stderr, "scan_center deve avere 3 valori.\n");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  MoveGroup move_group(node, planning_group);
  move_group.setMaxVelocityScalingFactor(velocity_scaling);
  move_group.setMaxAccelerationScalingFactor(acceleration_scaling);
  move_group.setPlanningTime(10.0);

  const moveit::core::RobotModelConstPtr model = move_group.getRobotModel();
  const moveit::core::JointModelGroup * jmg = model->getJointModelGroup(planning_group);
  if (!model->hasLinkModel(hsi_link) || !model->hasLinkModel(ee_link)) {
    std::fprintf(stderr, "Link %s o %s assente nel modello del robot.\n", hsi_link.c_str(), ee_link.c_str());
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }
  planning_scene::PlanningScenePtr scene = fetch_planning_scene(node, model, logger);

  moveit::core::RobotState home_state(model);
  home_state.setToDefaultValues();
  if (!home_state.setToDefaultValues(jmg, home_pose_name)) {
    std::fprintf(stderr, "Posa '%s' assente nello SRDF.\n", home_pose_name.c_str());
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }
  home_state.update();
  std::vector<double> home_joints;
  home_state.copyJointGroupPositions(jmg, home_joints);

  // The IK solver works on the camera TCP: the HSI target is converted with
  // this fixed transform (both frames are on tool0).
  const Eigen::Isometry3d hsi_to_tcp =
    home_state.getGlobalLinkTransform(hsi_link).inverse() * home_state.getGlobalLinkTransform(ee_link);

  auto acquire_pub = node->create_publisher<std_msgs::msg::String>(acquire_topic, 10);
  auto done_pub    = node->create_publisher<std_msgs::msg::String>(done_topic, 10);
  auto start_sub = node->create_subscription<std_msgs::msg::String>(
    start_topic, 10, [](const std_msgs::msg::String::SharedPtr msg) {
      if (g_running.load()) {
        std::printf("Start HSI '%s' ignorato: misura in corso.\n", msg->data.c_str());
        std::fflush(stdout);
        return;
      }
      std::lock_guard<std::mutex> lock(g_mutex);
      g_start_object = msg->data;
    });
  auto status_sub = node->create_subscription<std_msgs::msg::String>(status_topic, 10, on_hsi_status);

  std::printf("HSI: %.2f m sopra il centro [%.3f, %.3f, %.3f], link %s, speed %.2f (velocita' %.2f, "
              "accelerazione %.3f), wait_for_hsi %s\n"
              "Topic: start %s | acquire %s | status %s | done %s\n",
              distance, center[0], center[1], center[2], hsi_link.c_str(), speed, velocity_scaling,
              acceleration_scaling,
              wait_for_hsi ? "true" : "false",
              start_topic.c_str(), acquire_topic.c_str(), status_topic.c_str(), done_topic.c_str());

  while (rclcpp::ok()) {
    std::printf("\nIn attesa dell'oggetto su %s\n", start_topic.c_str());
    std::fflush(stdout);
    std::string object_name;
    while (rclcpp::ok() && object_name.empty()) {
      {
        std::lock_guard<std::mutex> lock(g_mutex);
        object_name = g_start_object;
        g_start_object.clear();
      }
      if (object_name.empty()) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!rclcpp::ok()) break;
    g_running = true;
    std::printf("\n=== Misura HSI di '%s' ===\n", object_name.c_str());
    std::fflush(stdout);
    const auto t0 = std::chrono::steady_clock::now();
    bool ok = true;
    bool moved = false;

    // 1. Only from home: never in the middle of a 3D scan.
    moveit::core::RobotStatePtr now_ptr = move_group.getCurrentState(5.0);
    if (!now_ptr) {
      std::fprintf(stderr, "Stato attuale del robot non disponibile (/joint_states?).\n");
      ok = false;
    } else {
      std::vector<double> now_joints;
      now_ptr->copyJointGroupPositions(jmg, now_joints);
      double worst = 0.0;
      for (size_t k = 0; k < now_joints.size() && k < home_joints.size(); ++k) {
        worst = std::max(worst, std::abs(now_joints[k] - home_joints[k]));
      }
      if (worst > home_tolerance) {
        std::fprintf(stderr, "Il robot non e' in '%s' (giunto piu' lontano: %.3f rad): misura rifiutata.\n",
                     home_pose_name.c_str(), worst);
        ok = false;
      }
    }

    // 2. Above the pole, rotation about the vertical closest to home. The HSI
    // looks along its Y axis: Y down, X horizontal at angle yaw, Z = X x Y.
    std::vector<double> best_joints;
    double best_cost = std::numeric_limits<double>::infinity();
    double best_yaw_deg = 0.0;
    if (ok) {
      auto collision_free = [&](moveit::core::RobotState * st, const moveit::core::JointModelGroup * group,
                                const double * values) {
        st->setJointGroupPositions(group, values);
        st->update();
        return !scene || !scene->isStateColliding(*st, group->getName());
      };
      for (double yaw_deg = 0.0; yaw_deg < 360.0; yaw_deg += yaw_step_deg) {
        const double yaw = yaw_deg * M_PI / 180.0;
        Eigen::Matrix3d axes;
        axes.col(0) = Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
        axes.col(1) = Eigen::Vector3d(0.0, 0.0, -1.0);
        axes.col(2) = axes.col(0).cross(axes.col(1));
        Eigen::Isometry3d hsi_pose = Eigen::Isometry3d::Identity();
        hsi_pose.linear() = axes;
        hsi_pose.translation() = Eigen::Vector3d(center[0], center[1], center[2] + distance);

        moveit::core::RobotState candidate(home_state);   // IK seeded from home
        if (!candidate.setFromIK(jmg, hsi_pose * hsi_to_tcp, ee_link, ik_timeout, collision_free)) continue;
        std::vector<double> joints;
        candidate.copyJointGroupPositions(jmg, joints);
        double cost = 0.0;
        for (size_t k = 0; k < joints.size(); ++k) cost += std::abs(joints[k] - home_joints[k]);
        if (cost < best_cost) {
          best_cost = cost;
          best_joints = joints;
          best_yaw_deg = yaw_deg;
        }
      }
      if (best_joints.empty()) {
        std::fprintf(stderr, "Nessuna configurazione senza collisioni a %.2f m sopra il centro.\n", distance);
        ok = false;
      } else {
        std::printf("Posa HSI: rotazione %.0f gradi, movimento dei giunti da %s:", best_yaw_deg,
                    home_pose_name.c_str());
        for (size_t k = 0; k < best_joints.size(); ++k) {
          std::printf(" %+.2f", best_joints[k] - home_joints[k]);
        }
        std::printf(" rad\n");
        std::fflush(stdout);
      }
    }

    // 3. Move above the pole.
    if (ok) {
      std::printf("Vado sopra il polo nord...\n");
      std::fflush(stdout);
      moved = true;
      if (!move_to_joints(move_group, best_joints)) {
        std::fprintf(stderr, "Movimento verso la posa HSI non riuscito.\n");
        ok = false;
      }
    }

    // 4. Measurement request and wait. The wait is armed before the request,
    // so an older measure_complete does not count.
    if (ok) {
      moveit::core::RobotStatePtr at_ptr = move_group.getCurrentState(5.0);
      Eigen::Isometry3d hsi = Eigen::Isometry3d::Identity();
      if (at_ptr) hsi = at_ptr->getGlobalLinkTransform(hsi_link);
      const Eigen::Quaterniond q(hsi.linear());
      {
        std::lock_guard<std::mutex> lock(g_mutex);
        g_wait_object = object_name;
        g_hsi_done = false;
      }
      char text[512];
      std::snprintf(text, sizeof(text), "acquire: %s; hsi pose: [%.4f, %.4f, %.4f, %.5f, %.5f, %.5f, %.5f]",
                    object_name.c_str(), hsi.translation().x(), hsi.translation().y(), hsi.translation().z(),
                    q.x(), q.y(), q.z(), q.w());
      std_msgs::msg::String msg;
      msg.data = text;
      acquire_pub->publish(msg);
      std::printf("  -> %s\n", text);
      if (wait_for_hsi) {
        std::printf("  attendo measure_complete su %s ...\n", status_topic.c_str());
        std::fflush(stdout);
        const auto wait_t0 = std::chrono::steady_clock::now();
        while (!g_hsi_done.load() && rclcpp::ok()) std::this_thread::sleep_for(std::chrono::milliseconds(20));
        std::printf("  misura fatta (%.1f s)\n",
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - wait_t0).count());
      } else {
        std::printf("  wait_for_hsi false: resto in posa %.1f s\n", wait_s);
        std::fflush(stdout);
        std::this_thread::sleep_for(std::chrono::duration<double>(wait_s));
      }
      {
        std::lock_guard<std::mutex> lock(g_mutex);
        g_wait_object.clear();
      }
      if (!rclcpp::ok()) ok = false;
    }

    // 5. Back home (also after a failed motion), then done.
    if (moved && rclcpp::ok()) {
      std::printf("Torno in %s...\n", home_pose_name.c_str());
      std::fflush(stdout);
      if (!move_to_joints(move_group, home_joints)) {
        std::fprintf(stderr, "Ritorno in %s non riuscito.\n", home_pose_name.c_str());
        ok = false;
      }
    }
    if (!rclcpp::ok()) break;   // Ctrl+C: nothing can be published any more
    std_msgs::msg::String done_msg;
    done_msg.data = "hsi " + object_name;
    if (!ok) done_msg.data += " ERROR";
    done_pub->publish(done_msg);
    std::printf("Misura HSI di '%s' %s in %.1f s -> %s\n", object_name.c_str(), ok ? "completata" : "NON riuscita",
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(),
                done_msg.data.c_str());
    std::fflush(stdout);
    g_running = false;
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
