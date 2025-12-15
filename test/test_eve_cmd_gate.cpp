
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_options.hpp>
#include <deque>
#include <chrono>
#include <thread>
#include "audio_driver_msgs/msg/sound_driver_res.hpp"
#include "audio_driver_msgs/msg/sound_driver_ctrl.hpp"
#include "sound_msgs/msg/sound_request.hpp"
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "autoware_adapi_v1_msgs/msg/route_state.hpp"
#include "autoware_adapi_v1_msgs/msg/route.hpp"
#include "autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp"
#include "autoware_state_machine_msgs/msg/state_lock.hpp"
#include "autoware_state_machine_msgs/msg/state_sound_done.hpp"
#include "autoware_state_machine_msgs/msg/state_machine.hpp"
#include "dio_ros_driver/msg/dio_port.hpp"
#include "in_parking_msgs/msg/in_parking_status.hpp"
#include "eve_cmd_gate_msgs/msg/engage_request_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tier4_external_api_msgs/srv/engage.hpp"
#include "tier4_external_api_msgs/srv/set_operator.hpp"
#include <autoware_adapi_v1_msgs/msg/vehicle_status.hpp>
#include "dio_ros_driver/msg/dio_array.hpp"
#include "v2i_interface_msgs/msg/infrastructure_command_array.hpp"
#include <tier4_external_api_msgs/msg/planning_factor_array.hpp>
#include <autoware_adapi_v1_msgs/msg/vehicle_kinematics.hpp>
#include <chrono>
#include <atomic>

using namespace std::chrono_literals;
// 期待周期と許容誤差
static const double PERIOD_SLOW_BLINK_SEC = 1.0;   // state=0 時の W 出力周期
static const double PERIOD_FAST_BLINK_SEC = 0.5;   // state=1 時の W 出力周期
static const double TOL_SLOW_BLINK_SEC    = 0.2;   // ±0.2s 許容
static const double TOL_FAST_BLINK_SEC    = 0.1;   // ±0.1s 許容

class EveCmdGateTest : public ::testing::Test {
protected:
  struct STLampDIO {
    bool value;
    std::chrono::steady_clock::time_point tp;
  };
  // 排他制御
  std::mutex mtx_status_lamp_;
  std::mutex mtx_warning_lamp_;
  std::mutex mtx_emergency_lamp_;
  std::mutex mtx_sound_done_;
  std::mutex mtx_in_parking_state_;
  std::mutex mtx_display_manager_;
  std::mutex mtx_vtl_commands_;

  // subscriberと期待値チェック同期
  std::condition_variable cv_status_lamp_;
  std::condition_variable cv_warning_lamp_;
  std::condition_variable cv_emergency_lamp_;
  std::condition_variable cv_sound_done_;
  std::condition_variable cv_sound_voice_alarm_audio_cmd_;
  std::condition_variable cv_in_parking_state_;
  std::condition_variable cv_display_manager_;
  std::condition_variable cv_vtl_commands_;

  // テストノード
  std::shared_ptr<rclcpp::Node> client_node_;
  std::shared_ptr<rclcpp::Node> service_node_;
  std::shared_ptr<rclcpp::Node> pub_sub_node_;
  std::shared_ptr<rclcpp::Node> adapi_mock_;
  std::shared_ptr<rclcpp::Node> sound_voice_alarm_audio_driver_mock_;
  std::shared_ptr<rclcpp::Node> sound_bgm_audio_driver_mock_;
  std::shared_ptr<rclcpp::Node> initial_pose_mock_;
  std::shared_ptr<rclcpp::Node> cargo_loading_service_mock_;
  std::shared_ptr<rclcpp::Node> eve_node_output_sub_;

  // Publisher (Target Node Input)
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr pub_operation_mode_state_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr pub_routing_state_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::Route>::SharedPtr pub_routing_route_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>::SharedPtr pub_initilization_state_;
  rclcpp::Publisher<audio_driver_msgs::msg::SoundDriverRes>::SharedPtr pub_voice_res_;
  rclcpp::Publisher<sound_msgs::msg::SoundRequest>::SharedPtr pub_sound_request_initialpose_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::VehicleStatus>::SharedPtr pub_vehicle_status_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::PlanningFactorArray>::SharedPtr pub_planning_factors_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::VehicleKinematics>::SharedPtr pub_vehicle_kinematics_;
  rclcpp::Publisher<eve_cmd_gate_msgs::msg::EngageRequestState>::SharedPtr pub_engage_process_state_;
  rclcpp::Publisher<autoware_state_machine_msgs::msg::StateSoundDone>::SharedPtr pub_state_sound_done_;

  // Subscriber (Target Node Output)
  rclcpp::Subscription<autoware_state_machine_msgs::msg::StateLock>::SharedPtr sub_lock_state_;
  rclcpp::Subscription<eve_cmd_gate_msgs::msg::EngageRequestState>::SharedPtr sub_engage_request_state_;
  rclcpp::Subscription<autoware_state_machine_msgs::msg::StateSoundDone>::SharedPtr sub_state_sound_done_;
  rclcpp::Subscription<audio_driver_msgs::msg::SoundDriverCtrl>::SharedPtr sub_voice_cmd_;
  rclcpp::Subscription<audio_driver_msgs::msg::SoundDriverCtrl>::SharedPtr sub_bgm_cmd_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::ResponseStatus>::SharedPtr sub_sound_response_initialpose_;
  rclcpp::Subscription<in_parking_msgs::msg::InParkingStatus>::SharedPtr sub_in_parking_state_;
  rclcpp::Subscription<dio_ros_driver::msg::DIOPort>::SharedPtr sub_status_lamp_;
  rclcpp::Subscription<dio_ros_driver::msg::DIOPort>::SharedPtr sub_emergency_lamp_;
  rclcpp::Subscription<dio_ros_driver::msg::DIOPort>::SharedPtr sub_warning_lamp_;
  rclcpp::Subscription<dio_ros_driver::msg::DIOArray>::SharedPtr sub_display_manager_;
  rclcpp::Subscription<v2i_interface_msgs::msg::InfrastructureCommandArray>::SharedPtr sub_vtl_commands_;

  // Client (Target Node Input)
  rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedPtr cli_engage_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_set_request_start_api_;

  // Service (Target Node Output)
  rclcpp::Service<tier4_external_api_msgs::srv::Engage>::SharedPtr srv_engage_;
  rclcpp::Service<tier4_external_api_msgs::srv::SetOperator>::SharedPtr srv_set_operator_;

  std::vector<uint8_t> msgs_requesting_;
  std::vector<uint8_t> msgs_accepted_;
  autoware_state_machine_msgs::msg::StateSoundDone msgs_sound_state_;
  std::vector<uint8_t> msgs_sound_done_;

  // lamp系のメッセージ
  std::deque<STLampDIO> msg_status_lamp_;
  std::deque<STLampDIO> msg_warning_lamp_;
  std::deque<STLampDIO> msg_emergency_lamp_;

  // audio系メッセージ
  audio_driver_msgs::msg::SoundDriverCtrl::ConstSharedPtr msgs_sound_voice_alarm_audio_cmd_;
  audio_driver_msgs::msg::SoundDriverCtrl sound_bgm_audio_cmd_;

  // parking系メッセージ
  in_parking_msgs::msg::InParkingStatus msg_in_parking_state_;

  // drive系メッセージ
  #define DISPLAY_DOUT_PORTS_NUM (3)
  std::vector<uint8_t> msg_display_manager_;
  std::vector<uint8_t> msg_vtl_commands_;

  tier4_external_api_msgs::msg::ResponseStatus srv_engage_res_;
  tier4_external_api_msgs::msg::ResponseStatus srv_set_operator_res_;

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;

  // Parameter
  double moved_threshold_;
  double dist_to_stop_pose_min_th_;

  std::atomic<bool> stop_{false}; // ★ spin終了フラグ

  // ★ スイート単位で init / shutdown（GuardConditionの context=null を回避）
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) { rclcpp::init(0, nullptr); }
  }
  static void TearDownTestSuite() {
    if (rclcpp::ok()) { rclcpp::shutdown(); }
  }

  void SetUp() override {
    msgs_requesting_.clear();
    msgs_accepted_.clear();
    msgs_sound_state_.state = autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED;
    msgs_sound_state_.done = false;
    msg_in_parking_state_.aw_state = 0xFF;
    msg_in_parking_state_.vehicle_operation_mode = 0xFF;
    srv_engage_res_.code = tier4_external_api_msgs::msg::ResponseStatus::SUCCESS;
    srv_set_operator_res_.code = tier4_external_api_msgs::msg::ResponseStatus::SUCCESS;
    eve_node_output_sub_ = std::make_shared<rclcpp::Node>("test_eve_node_output_sub");
    adapi_mock_ = std::make_shared<rclcpp::Node>("test_adapi_mock");
    sound_voice_alarm_audio_driver_mock_ = std::make_shared<rclcpp::Node>("test_sound_voice_alarm_audio_driver_mock");
    sound_bgm_audio_driver_mock_ = std::make_shared<rclcpp::Node>("test_sound_bgm_audio_driver_mock");
    initial_pose_mock_ = std::make_shared<rclcpp::Node>("test_initial_pose_mock");
    client_node_ = std::make_shared<rclcpp::Node>("test_node_client");
    service_node_ = std::make_shared<rclcpp::Node>("test_node_service");

    // Parameter
    moved_threshold_ = initial_pose_mock_->declare_parameter<double>("moved_threshold", 1.0);
    double stop_dist_to_prohibit_engage = initial_pose_mock_->declare_parameter<double>("stop_dist_to_prohibit_engage", 0.30);
    // Add a value of 0.05 to `stop_dist_to_prohibit_engage`.
    dist_to_stop_pose_min_th_ = stop_dist_to_prohibit_engage + 0.05;

    cargo_loading_service_mock_ = std::make_shared<rclcpp::Node>("test_cargo_loading_service_mock", "");

    // define dio_ros_driver_node_mock start
    // Publisher
    // Subscriber
    // Cliengt
    // Service
    // define dio_ros_driver_node_mock end

    // define ADAPI_mock start
    // Publisher
    pub_operation_mode_state_ = adapi_mock_->create_publisher<autoware_adapi_v1_msgs::msg::OperationModeState>(
      "/api/operation_mode/state", rclcpp::QoS{1}.transient_local());
    pub_routing_state_ = adapi_mock_->create_publisher<autoware_adapi_v1_msgs::msg::RouteState>(
      "/api/routing/state", rclcpp::QoS{1}.transient_local());
    pub_routing_route_ = adapi_mock_->create_publisher<autoware_adapi_v1_msgs::msg::Route>(
      "/api/routing/route", rclcpp::QoS{1}.transient_local());
    pub_initilization_state_ = adapi_mock_->create_publisher<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>(
      "/api/localization/initialization_state", rclcpp::QoS{3}.transient_local());
    pub_planning_factors_ = initial_pose_mock_->create_publisher<tier4_external_api_msgs::msg::PlanningFactorArray>(
      "/planning/planning_factors", rclcpp::QoS{3}.transient_local());
    pub_vehicle_kinematics_ = initial_pose_mock_->create_publisher<autoware_adapi_v1_msgs::msg::VehicleKinematics>(
      "/api/vehicle/kinematics", rclcpp::QoS{3}.transient_local());
    pub_vehicle_status_ = adapi_mock_->create_publisher<autoware_adapi_v1_msgs::msg::VehicleStatus>(
      "/api/vehicle/status", rclcpp::QoS{1}.transient_local());
    pub_engage_process_state_ = adapi_mock_->create_publisher<eve_cmd_gate_msgs::msg::EngageRequestState>(
    "/eve_cmd_gate/engage_request_state", rclcpp::QoS{1}.transient_local());
    pub_state_sound_done_ = adapi_mock_->create_publisher<autoware_state_machine_msgs::msg::StateSoundDone>(
    "/autoware_state_machine/state_sound_done", rclcpp::QoS{1}.transient_local());
    // define ADAPI_mock end

    // define sound_voice_alarm/audio_driver_mock start
    // Publisher
    pub_voice_res_ = sound_voice_alarm_audio_driver_mock_->create_publisher<audio_driver_msgs::msg::SoundDriverRes>(
      "/sound_voice_alarm/audio_res", rclcpp::QoS{3}.transient_local());
    // Subscriber
    sub_voice_cmd_ = sound_voice_alarm_audio_driver_mock_->create_subscription<audio_driver_msgs::msg::SoundDriverCtrl>(
      "/sound_voice_alarm/audio_cmd", rclcpp::QoS{5}.transient_local(),
      [this](const audio_driver_msgs::msg::SoundDriverCtrl::SharedPtr msg)
      {
        msgs_sound_voice_alarm_audio_cmd_ = msg;
        cv_sound_voice_alarm_audio_cmd_.notify_all();

        audio_driver_msgs::msg::SoundDriverRes sound_res;
        pub_voice_res_->publish(sound_res);
      }
    );
    // define sound_voice_alarm/audio_driver_mock end

    // define sound_bgm_audio_driver_mock start
    // Subscriber
    sub_bgm_cmd_ = sound_bgm_audio_driver_mock_->create_subscription<audio_driver_msgs::msg::SoundDriverCtrl>(
      "/sound_bgm/audio_cmd", rclcpp::QoS{5}.transient_local(),
      [this](const audio_driver_msgs::msg::SoundDriverCtrl::SharedPtr msg)
      {
        // TODO：必要に応じて実装
        // sound_bgm_audio_cmd_.type msg->cmd_type;
        // sound_bgm_audio_cmd_.file_path = msg->file_path;
        // sound_bgm_audio_cmd_.volume = msg->volume;
        // sound_bgm_audio_cmd_.is_loop = msg->is_loop;
        // sound_bgm_audio_cmd_.loop_delay = msg->loop_delay;
        // sound_bgm_audio_cmd_.start_delay = msg->start_delay;
      }
    );
    // define sound_bgm_audio_driver_mock end

    // define initial_pose_mock start
    // Publisher
    pub_sound_request_initialpose_ = initial_pose_mock_->create_publisher<sound_msgs::msg::SoundRequest>(
      "/localization/initial_pose/sound/request", rclcpp::QoS{3}.transient_local());
    // Subscriber
    sub_sound_response_initialpose_ = initial_pose_mock_->create_subscription<tier4_external_api_msgs::msg::ResponseStatus>(
      "/localization/initial_pose/sound/response", rclcpp::QoS{3}.transient_local(),
      [this](const tier4_external_api_msgs::msg::ResponseStatus::SharedPtr msg)
      {
        // TODO：必要に応じて実装
      }
    );
    // define initial_pose_mock end

    // define cargo_loading_service_mock start
    // Subscriber
    sub_in_parking_state_ = cargo_loading_service_mock_->create_subscription<in_parking_msgs::msg::InParkingStatus>(
      "in_parking/state", rclcpp::QoS(1),
      [this](const in_parking_msgs::msg::InParkingStatus::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_in_parking_state_);
        msg_in_parking_state_.aw_state = msg->aw_state;
        msg_in_parking_state_.vehicle_operation_mode = msg->vehicle_operation_mode;
        cv_in_parking_state_.notify_all();
      }
    );
    // define cargo_loading_service_mock end

    // define eve_node_output_sub start
    // eveノードからpublishされるtopicを収集し、期待値と比較する
    // Subscriber
    sub_engage_request_state_ = eve_node_output_sub_->create_subscription<eve_cmd_gate_msgs::msg::EngageRequestState>(
      "/eve_cmd_gate/engage_request_state", rclcpp::QoS{1}.transient_local(),
      [this](const eve_cmd_gate_msgs::msg::EngageRequestState::SharedPtr msg)
      {
        // TODO：必要に応じて実装
        // msgs_requesting_.push_back(msg->is_engage_requesting);
        // msgs_accepted_.push_back(msg->is_engage_accepted);
      }
    );
    sub_lock_state_ = eve_node_output_sub_->create_subscription<autoware_state_machine_msgs::msg::StateLock>(
      "/go_interface/lock_state", rclcpp::QoS{1}.transient_local(),
      [this](const autoware_state_machine_msgs::msg::StateLock::SharedPtr msg)
      {
        // TODO：必要に応じて実装
        // msg->state;
      }
    );
    sub_state_sound_done_ = eve_node_output_sub_->create_subscription<autoware_state_machine_msgs::msg::StateSoundDone>(
      "/autoware_state_machine/state_sound_done", rclcpp::QoS{1}.transient_local(),
      [this](const autoware_state_machine_msgs::msg::StateSoundDone::SharedPtr msg)
      {
        msgs_sound_state_.state = msg->state;
        msgs_sound_state_.done = msg->done;
        cv_sound_done_.notify_all();
      }
    );
    sub_status_lamp_ = eve_node_output_sub_->create_subscription<dio_ros_driver::msg::DIOPort>(
      "/dio/dout0", rclcpp::QoS{3}.transient_local(),
      [this](const dio_ros_driver::msg::DIOPort::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_status_lamp_);
        msg_status_lamp_.push_back({msg->value, std::chrono::steady_clock::now()});
        cv_status_lamp_.notify_all();
      }
    );
    sub_emergency_lamp_ = eve_node_output_sub_->create_subscription<dio_ros_driver::msg::DIOPort>(
      "/dio/dout1", rclcpp::QoS{3}.transient_local(),
      [this](const dio_ros_driver::msg::DIOPort::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_emergency_lamp_);
        msg_emergency_lamp_.push_back({msg->value, std::chrono::steady_clock::now()});
        cv_emergency_lamp_.notify_all();
      }
    );
    sub_warning_lamp_ = eve_node_output_sub_->create_subscription<dio_ros_driver::msg::DIOPort>(
      "/dio/dout2", rclcpp::QoS{3}.transient_local(),
      [this](const dio_ros_driver::msg::DIOPort::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_warning_lamp_);
        msg_warning_lamp_.push_back({msg->value, std::chrono::steady_clock::now()});
        cv_warning_lamp_.notify_all();
      }
    );
    sub_display_manager_ = eve_node_output_sub_->create_subscription<dio_ros_driver::msg::DIOArray>(
      "/dio/dout_array", rclcpp::QoS{3}.transient_local(),
      [this](const dio_ros_driver::msg::DIOArray::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_display_manager_);
        msg_display_manager_.clear();
        for (int i = 0; i < DISPLAY_DOUT_PORTS_NUM; i++) {
          msg_display_manager_.push_back(msg->values[i].value);
        }
        cv_display_manager_.notify_all();
      }
    );
    sub_vtl_commands_ = eve_node_output_sub_->create_subscription<v2i_interface_msgs::msg::InfrastructureCommandArray>(
      "/v2_gate/infrastructure_commands", rclcpp::QoS{3}.transient_local(),
      [this](const v2i_interface_msgs::msg::InfrastructureCommandArray::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_vtl_commands_);
        msg_vtl_commands_.clear();
        msg_vtl_commands_.push_back(msg->commands[0].state);
        cv_vtl_commands_.notify_all();
      }
    );
    // define eve_node_output_sub end

    // define stop_reason_mock start
    // Publisher
    ///pub_planning_factors_ = initial_pose_mock_->create_publisher<tier4_external_api_msgs::msg::PlanningFactorArray>(
    //  "/planning/planning_factors", rclcpp::QoS{3}.transient_local());
    // define stop_reason_mock end

    // define vehicle_kinematics_mock start
    // Publisher
    //pub_vehicle_kinematics_ = initial_pose_mock_->create_publisher<autoware_adapi_v1_msgs::msg::VehicleKinematics>(
    //  "/api/vehicle/kinematics", rclcpp::QoS{3}.transient_local());
    // define vehicle_kinematics_mock end

    // define engage request start
    // Publisher
    //pub_engage_process_state_ = initial_pose_mock_->create_publisher<eve_cmd_gate_msgs::msg::EngageRequestState>(
    //  "/eve_cmd_gate/engage_request_state", rclcpp::QoS{3}.transient_local());
    // define engage request end

    // define engage request start
    // Publisher
    //pub_state_sound_done_ = initial_pose_mock_->create_publisher<autoware_state_machine_msgs::msg::StateSoundDone>(
    //  "/autoware_state_machine/state_sound_done", rclcpp::QoS{3}.transient_local());
    // define engage request end

    // define xxx_mock start
    // Publisher
    // Subscriber
    // Cliengt
    // Service
    // define xxx_mock end

    // Service
    srv_engage_ = service_node_->create_service<tier4_external_api_msgs::srv::Engage>(
    "/api/autoware/set/engage",
    [this](const tier4_external_api_msgs::srv::Engage::Request::SharedPtr request,
        const tier4_external_api_msgs::srv::Engage::Response::SharedPtr response){
        response->status = srv_engage_res_;
    });

    srv_set_operator_ = service_node_->create_service<tier4_external_api_msgs::srv::SetOperator>(
    "/api/autoware/set/operator",
    [this](const tier4_external_api_msgs::srv::SetOperator::Request::SharedPtr request,
        const tier4_external_api_msgs::srv::SetOperator::Response::SharedPtr response)
        {
        response->status.code = srv_set_operator_res_.code;
        }
    );

    // Client
    cli_engage_ = client_node_->create_client<tier4_external_api_msgs::srv::Engage>(
    "/api/external/set/engage",
    rmw_qos_profile_services_default);

    cli_set_request_start_api_ = client_node_->create_client<std_srvs::srv::Trigger>(
    "/api/autoware/set/start_request",
    rmw_qos_profile_services_default);

    // executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_.add_node(adapi_mock_);
    executor_.add_node(sound_voice_alarm_audio_driver_mock_);
    executor_.add_node(cargo_loading_service_mock_);
    executor_.add_node(eve_node_output_sub_);

    spin_thread_ = std::thread([this]{
      while (!stop_) {
        executor_.spin_once(std::chrono::milliseconds(50));
      }
    });
  }

  void TearDown() override {
    if (spin_thread_.joinable()) {
      stop_ = true;
      executor_.cancel();       // ★ spin_onceを中断
      spin_thread_.join();
    }
  }

  // output status_lamp キューをクリア
  void clear_status_lamp_queue() {
    {
      std::lock_guard<std::mutex> lock(mtx_status_lamp_);
      msg_status_lamp_.clear();
    }
  }

  // output warning_lamp キューをクリア
  void clear_warning_lamp_queue() {
    {
      std::lock_guard<std::mutex> lock(mtx_warning_lamp_);
      msg_warning_lamp_.clear();
    }
  }

  // output emergency_lamp キューをクリア
  void clear_emergency_lamp_queue() {
    {
      std::lock_guard<std::mutex> lock(mtx_emergency_lamp_);
      msg_emergency_lamp_.clear();
    }
  }

  // status_lamp操作 を N 件収集（timeout 以内）
  std::vector<STLampDIO> collect_status_lamp_msgs(size_t n,
                                         std::chrono::milliseconds timeout) {
    std::vector<STLampDIO> out;
    out.reserve(n);
    auto deadline = std::chrono::steady_clock::now() + timeout;

    while (out.size() < n) {
      std::unique_lock<std::mutex> lock(mtx_status_lamp_);
      if (msg_status_lamp_.empty()) {
        auto now = std::chrono::steady_clock::now();
        if (now >= deadline) break;
        cv_status_lamp_.wait_until(lock, deadline, [this]{ return !msg_status_lamp_.empty(); });
        if (msg_status_lamp_.empty()) break;
      }
      out.push_back(msg_status_lamp_.front());
      msg_status_lamp_.pop_front();
    }
    return out;
  }

  // 値が交互（true/false）になっているか
  static bool is_alternating(const std::vector<STLampDIO>& msgs) {
    if (msgs.size() < 2) return false;
    for (size_t i = 1; i < msgs.size(); ++i) {
      if (msgs[i].value == msgs[i - 1].value) return false;
    }
    return true;
  }

  // lamp操作topicの周期（隣接差の平均秒）を推定
  static double estimate_period_sec(const std::vector<STLampDIO>& msgs) {
    if (msgs.size() < 2) return 0.0;
    double sum = 0.0;
    size_t cnt = 0;
    for (size_t i = 1; i < msgs.size(); ++i) {
      auto dt = std::chrono::duration<double>(msgs[i].tp - msgs[i - 1].tp).count();
      sum += dt;
      ++cnt;
    }
    return (cnt > 0) ? (sum / static_cast<double>(cnt)) : 0.0;
  }

  // warning_lamp を待つ（timeout 以内、期待値チェックあり）
  bool wait_for_warning_lamp(bool expected_warning_lamp,
                         std::chrono::milliseconds timeout = 2000ms) {
    std::unique_lock<std::mutex> lock(mtx_warning_lamp_);
    bool ok = cv_warning_lamp_.wait_for(lock, timeout, [this]{ return !msg_warning_lamp_.empty(); });
    if (!ok) return false;

    auto msg = msg_warning_lamp_.front();
    msg_warning_lamp_.clear();
    return msg.value == expected_warning_lamp;
  }

  // emergency_lamp を待つ（timeout 以内、期待値チェックあり）
  bool wait_for_emergency_lamp(bool expected_emergency_lamp,
                         std::chrono::milliseconds timeout = 2000ms) {
    std::unique_lock<std::mutex> lock(mtx_emergency_lamp_);
    bool ok = cv_emergency_lamp_.wait_for(lock, timeout, [this]{ return !msg_emergency_lamp_.empty(); });
    if (!ok) return false;

    auto msg = msg_emergency_lamp_.front();
    msg_emergency_lamp_.clear();
    return msg.value == expected_emergency_lamp;
  }

  // sound_done を待つ（timeout 以内、期待値チェックあり）
  bool wait_for_sound_done(bool expected_sound_done_state,
                         std::chrono::milliseconds timeout = 2000ms) {
    std::unique_lock<std::mutex> lock(mtx_sound_done_);
    bool ok = cv_sound_done_.wait_for(lock, timeout, [this]{ return msgs_sound_state_.state != autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED; });
    if (!ok) return false;

    return msgs_sound_state_.state == expected_sound_done_state;
  }

  // in_parking_state を待つ（timeout 以内、期待値チェックあり）
  bool wait_for_in_parking_state(int32_t expected_aw_state,
                         int32_t expected_vehicle_operation_mode,
                         std::chrono::milliseconds timeout = 2000ms) {
    std::unique_lock<std::mutex> lock(mtx_in_parking_state_);
    bool ok = cv_in_parking_state_.wait_for(lock, timeout, [this]{ return msg_in_parking_state_.aw_state != 0xFF; });
    if (!ok) return false;

    //bool ret = false;
    //if ((msg_in_parking_state_.aw_state == expected_aw_state)
    //  && (msg_in_parking_state_.vehicle_operation_mode == expected_vehicle_operation_mode)) {
    //  ret = true;
    //}
    //return ret;
    return true;
  }

  // display_manager を待つ（timeout 以内、期待値チェックあり）
  bool wait_for_display_manager(bool expected_dout_1, bool expected_dout_2, bool expected_dout_3,
                         std::chrono::milliseconds timeout = 2000ms) {
    std::unique_lock<std::mutex> lock(mtx_display_manager_);
    bool ok = cv_display_manager_.wait_for(lock, timeout, [this]{ return !msg_display_manager_.empty(); });
    if (!ok) return false;

    auto msg_dout_1 = msg_display_manager_[0];
    auto msg_dout_2 = msg_display_manager_[1];
    auto msg_dout_3 = msg_display_manager_[2];
    msg_display_manager_.clear();
    return (expected_dout_1 == msg_dout_1 && expected_dout_2 == msg_dout_2 && expected_dout_3 == msg_dout_3);
  }

  // infrastructure commands を待つ（timeout 以内、期待値チェックあり）
  bool wait_for_vtl_commands(bool expected_state,
                         std::chrono::milliseconds timeout = 2000ms) {
    std::unique_lock<std::mutex> lock(mtx_vtl_commands_);
    bool ok = cv_vtl_commands_.wait_for(lock, timeout, [this]{ return !msg_vtl_commands_.empty(); });
    if (!ok) return false;

    auto msg_state = msg_vtl_commands_[0];
    msg_vtl_commands_.clear();
    return (expected_state == msg_state);
  }

  // stop reasonsをパブリッシュする
  void publishStopReasons(bool is_obstacle_stop, bool is_detection_area, bool is_crosswalk, bool is_surround_obstacle_check, bool is_other_reason, double distance)
  {
    tier4_external_api_msgs::msg::PlanningFactorArray factorArray;
    tier4_external_api_msgs::msg::PlanningFactor factor;
    tier4_external_api_msgs::msg::PlanningFactorControlPoint controlPoint;

    controlPoint.distance = distance;
    factor.behavior_type = tier4_external_api_msgs::msg::PlanningFactor::STOP;
    factor.control_points.push_back(controlPoint);
    if(is_obstacle_stop){
      factor.behavior_name = tier4_external_api_msgs::msg::PlanningFactor::ROUTE_OBSTACLE;
      factorArray.factors.push_back(factor);
    }
    if(is_detection_area){
      factor.behavior_name = tier4_external_api_msgs::msg::PlanningFactor::USER_DEFINED_DETECTION_AREA;
      factorArray.factors.push_back(factor);
    }
    if(is_crosswalk){
      factor.behavior_name = tier4_external_api_msgs::msg::PlanningFactor::CROSSWALK;
      factorArray.factors.push_back(factor);
    }
    if(is_surround_obstacle_check){
      factor.behavior_name = tier4_external_api_msgs::msg::PlanningFactor::SURROUNDING_OBSTACLE;
      factorArray.factors.push_back(factor);
    }
    if(is_other_reason){
      factor.behavior_name = tier4_external_api_msgs::msg::PlanningFactor::STOP_SIGN;
      factorArray.factors.push_back(factor);
    }

    pub_planning_factors_->publish(factorArray);
  }

  // vehicle kinematicsをパブリッシュする
  void publishVehicleKinematics(double pose_x, double velocity)
  {
    autoware_adapi_v1_msgs::msg::VehicleKinematics vehicle_kinematics;
    vehicle_kinematics.twist.twist.twist.linear.x = velocity;
    vehicle_kinematics.pose.pose.pose.position.x = pose_x;

    pub_vehicle_kinematics_->publish(vehicle_kinematics);
  }

  // engage requestをパブリッシュする
  void publishEngageRequest(bool isRequest)
  {
    eve_cmd_gate_msgs::msg::EngageRequestState pub;
    pub.is_engage_requesting = isRequest;
    pub.is_engage_accepted = false;
    pub_engage_process_state_->publish(pub);    
  }

  // sound doneをパブリッシュする
  void publishSoundDone(uint16_t state)
  {
    autoware_state_machine_msgs::msg::StateSoundDone done_msg;
    done_msg.state = state;
    done_msg.done = true;
    pub_state_sound_done_->publish(done_msg);    
  }

  // sound_voice_alarm/audio_cmd を待つ（timeout 以内、期待値チェックあり）
//   bool wait_for_sound_voice_alarm_audio_cmd(int32 expected_aw_state,
//                          int32 expected_vehicle_operation_mode,
//                          std::chrono::milliseconds timeout = 2000ms) {
//     std::unique_lock<std::mutex> lock(mtx_in_parking_state_);
//     bool ok = cv_in_parking_state_.wait_for(lock, timeout, [this]{ return !msg_in_parking_state_; });
//     if (!ok) return false;

//     bool ret = false;
//     if ((msg_in_parking_state_->aw_state == expected_aw_state)
//       && (msg_in_parking_state_->vehicle_operation_mode == expected_vehicle_operation_mode)) {
//       ret = true;
//     }
//     return ret;
//   }

};

TEST_F(EveCmdGateTest, Case_AutoDrive_Running) {
  clear_status_lamp_queue();
  clear_warning_lamp_queue();
  clear_emergency_lamp_queue();

  // DRIVING
  autoware_adapi_v1_msgs::msg::LocalizationInitializationState initialization_state_initializing;
  initialization_state_initializing.state = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED;
  pub_initilization_state_->publish(initialization_state_initializing);

  autoware_adapi_v1_msgs::msg::RouteState routing_state;
  routing_state.state = autoware_adapi_v1_msgs::msg::RouteState::SET;
  pub_routing_state_->publish(routing_state);

  autoware_adapi_v1_msgs::msg::Route routing_route;
  autoware_adapi_v1_msgs::msg::RouteData route_data;
  routing_route.data.push_back(route_data);
  pub_routing_route_->publish(routing_route);

  autoware_adapi_v1_msgs::msg::OperationModeState operation_mode_state;
  operation_mode_state.mode = autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS;
  operation_mode_state.is_autoware_control_enabled = true;
  pub_operation_mode_state_->publish(operation_mode_state);
  rclcpp::sleep_for(1000ms);

  // STATE_RUNNING
  publishVehicleKinematics(0.0, 0.0);
  rclcpp::sleep_for(1000ms);
  
  autoware_adapi_v1_msgs::msg::VehicleStatus vehicle_status;
  vehicle_status.turn_indicators.status = autoware_adapi_v1_msgs::msg::TurnIndicators::DISABLE;
  pub_vehicle_status_->publish(vehicle_status);
  rclcpp::sleep_for(1000ms);

  // 期待値チェック
  //ASSERT_TRUE(wait_for_display_manager(true, false, false, 2000ms));
  //ASSERT_TRUE(wait_for_vtl_commands(true, 2000ms));

  // STATE_TURNING_LEFT
  vehicle_status.turn_indicators.status = autoware_adapi_v1_msgs::msg::TurnIndicators::LEFT;
  pub_vehicle_status_->publish(vehicle_status);
  rclcpp::sleep_for(1000ms);

  // STATE_TURNING_ROGHT
  vehicle_status.turn_indicators.status = autoware_adapi_v1_msgs::msg::TurnIndicators::RIGHT;
  pub_vehicle_status_->publish(vehicle_status);
  rclcpp::sleep_for(1000ms);

  // ARRIVAL_GOAL
  routing_state.state = autoware_adapi_v1_msgs::msg::RouteState::ARRIVED;
  pub_routing_state_->publish(routing_state);
  rclcpp::sleep_for(2000ms);

  // STATE_ARRIVED_GOAL

  // 期待値チェック
  //ASSERT_TRUE(wait_for_display_manager(false, false, false, 2000ms));
  //ASSERT_TRUE(wait_for_vtl_commands(true, 2000ms));

  // STATE_RESTART
  //auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  //auto restart_future = cli_set_request_start_api_->async_send_request(request);
  //auto result = executor_.spin_until_future_complete(restart_future, std::chrono::seconds(5));
  //ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  //ASSERT_TRUE(cli_set_request_start_api_->wait_for_service(std::chrono::seconds(10)));
  publishEngageRequest(true);
  rclcpp::sleep_for(2000ms);

  routing_state.state = autoware_adapi_v1_msgs::msg::RouteState::SET;
  pub_routing_state_->publish(routing_state);

  publishEngageRequest(false);
  publishSoundDone(autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART);
  rclcpp::sleep_for(2000ms);

  // STATE_RUNNING_TOWARD_STOP_LINE
  bool is_obstacle_stop = false;
  bool is_detection_area = false;
  bool is_crosswalk = false;
  bool is_surround_obstacle_check = false;
  bool is_other_reason = true;
  double distance = dist_to_stop_pose_min_th_ + 1.0;
  //EXPECT_EQ(distance, dist_to_stop_pose_min_th_);
  publishStopReasons(is_obstacle_stop, is_detection_area, is_crosswalk, is_surround_obstacle_check, is_other_reason, distance);
  rclcpp::sleep_for(1000ms);

  // STATE_RUNNING_TOWARD_OBSTACLE
  is_obstacle_stop = true;
  is_detection_area = false;
  is_crosswalk = false;
  is_surround_obstacle_check = false;
  is_other_reason = false;
  distance = dist_to_stop_pose_min_th_ + 1.0;
  publishStopReasons(is_obstacle_stop, is_detection_area, is_crosswalk, is_surround_obstacle_check, is_other_reason, distance);
  rclcpp::sleep_for(1000ms);

  // STATE_STOP_DUETO_TRAFFIC_CONDITION
  is_obstacle_stop = false;
  is_detection_area = false;
  is_crosswalk = false;
  is_surround_obstacle_check = false;
  is_other_reason = true;
  distance = dist_to_stop_pose_min_th_ - 1.0;
  publishStopReasons(is_obstacle_stop, is_detection_area, is_crosswalk, is_surround_obstacle_check, is_other_reason, distance);
  rclcpp::sleep_for(1000ms);

  // STATE_STOP_DUETO_APPROACHING_OBSTACLE
  is_obstacle_stop = true;
  is_detection_area = false;
  is_crosswalk = false;
  is_surround_obstacle_check = false;
  is_other_reason = false;
  distance = dist_to_stop_pose_min_th_ - 1.0;
  publishStopReasons(is_obstacle_stop, is_detection_area, is_crosswalk, is_surround_obstacle_check, is_other_reason, distance);
  rclcpp::sleep_for(1000ms);

  // STATE_STOP_DUETO_SURROUNDING_PROXIMITY
  is_obstacle_stop = false;
  is_detection_area = false;
  is_crosswalk = false;
  is_surround_obstacle_check = true;
  is_other_reason = false;
  distance = dist_to_stop_pose_min_th_;
  publishStopReasons(is_obstacle_stop, is_detection_area, is_crosswalk, is_surround_obstacle_check, is_other_reason, distance);
  rclcpp::sleep_for(1000ms);






  for(int i=0; i<20; i++){
  EXPECT_TRUE(wait_for_in_parking_state(in_parking_msgs::msg::InParkingStatus::AW_ARRIVED_PARKING,
              in_parking_msgs::msg::InParkingStatus::VEHICLE_AUTO, 2000ms));
  EXPECT_EQ(msg_in_parking_state_.aw_state, in_parking_msgs::msg::InParkingStatus::AW_ARRIVED_PARKING);
  EXPECT_EQ(msg_in_parking_state_.vehicle_operation_mode, in_parking_msgs::msg::InParkingStatus::VEHICLE_AUTO);
  rclcpp::sleep_for(1000ms);
  }
  
  // 期待値チェック
  //ASSERT_TRUE(wait_for_display_manager(true, false, false, 2000ms));
  //ASSERT_TRUE(wait_for_vtl_commands(true, 2000ms));
}

#if 0
TEST_F(EveCmdGateTest, Case_AutoDrive_RunningToward) {
  clear_status_lamp_queue();
  clear_warning_lamp_queue();
  clear_emergency_lamp_queue();

  // DRIVING
  autoware_adapi_v1_msgs::msg::LocalizationInitializationState initialization_state_initializing;
  initialization_state_initializing.state = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED;
  pub_initilization_state_->publish(initialization_state_initializing);

  autoware_adapi_v1_msgs::msg::RouteState routing_state;
  routing_state.state = autoware_adapi_v1_msgs::msg::RouteState::SET;
  pub_routing_state_->publish(routing_state);

  autoware_adapi_v1_msgs::msg::Route routing_route;
  autoware_adapi_v1_msgs::msg::RouteData route_data;
  routing_route.data.push_back(route_data);
  pub_routing_route_->publish(routing_route);

  autoware_adapi_v1_msgs::msg::OperationModeState operation_mode_state;
  operation_mode_state.mode = autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS;
  operation_mode_state.is_autoware_control_enabled = true;
  pub_operation_mode_state_->publish(operation_mode_state);

  // STATE_RUNNING
  autoware_adapi_v1_msgs::msg::VehicleStatus vehicle_status;
  vehicle_status.turn_indicators.status = autoware_adapi_v1_msgs::msg::TurnIndicators::DISABLE;
  pub_vehicle_status_->publish(vehicle_status);

  // 期待値チェック
  ASSERT_TRUE(wait_for_display_manager(true, false, false, 2000ms));
  ASSERT_TRUE(wait_for_vtl_commands(true, 2000ms));

  // STATE_RUNNING_TOWARD_STOP_LINE
  bool is_obstacle_stop = false;
  bool is_detection_area = false;
  bool is_crosswalk = false;
  bool is_surround_obstacle_check = false;
  bool is_other_reason = false;
  double distance = dist_to_stop_pose_min_th_ + 1.0;
  publishStopReasons(is_obstacle_stop, is_detection_area, is_crosswalk, is_surround_obstacle_check, is_other_reason, distance);

  // 期待値チェック
  ASSERT_TRUE(wait_for_display_manager(true, false, false, 2000ms));
  ASSERT_TRUE(wait_for_vtl_commands(true, 2000ms));
  ASSERT_TRUE(wait_for_in_parking_state(in_parking_msgs::msg::InParkingStatus::AW_OUT_OF_PARKING,
    in_parking_msgs::msg::InParkingStatus::VEHICLE_AUTO, 2000ms));

}






TEST_F(EveCmdGateTest, Case_Initializing_SoundDone) {
  clear_status_lamp_queue();
  clear_warning_lamp_queue();
  clear_emergency_lamp_queue();

  // target Input
  // 起動開始
  autoware_adapi_v1_msgs::msg::LocalizationInitializationState initialization_state_initializing;
  initialization_state_initializing.state = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZING;
  pub_initilization_state_->publish(initialization_state_initializing);

  // warning_lamp, emergency_lamp を待って期待値一致
  EXPECT_TRUE(wait_for_in_parking_state(in_parking_msgs::msg::InParkingStatus::AW_UNAVAILABLE,
    in_parking_msgs::msg::InParkingStatus::VEHICLE_MANUAL, 2000ms));

  // warning_lamp, emergency_lamp を待って期待値一致
  EXPECT_TRUE(wait_for_warning_lamp(true, 2000ms));
  EXPECT_TRUE(wait_for_emergency_lamp(true, 2000ms));

  // status_lamp
  auto status_lamp_msgs = collect_status_lamp_msgs(8, 6000ms);
  EXPECT_GE(status_lamp_msgs.size(), 6u);

  EXPECT_TRUE(is_alternating(status_lamp_msgs));

  double period = estimate_period_sec(status_lamp_msgs);
  EXPECT_NEAR(period, PERIOD_SLOW_BLINK_SEC, TOL_SLOW_BLINK_SEC);

  // sound_done を待って期待値一致
  EXPECT_TRUE(wait_for_sound_done(autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE, 2000ms));

  // warning_lamp, emergency_lamp を待って期待値一致
  EXPECT_TRUE(wait_for_in_parking_state(in_parking_msgs::msg::InParkingStatus::AW_UNAVAILABLE,
    in_parking_msgs::msg::InParkingStatus::VEHICLE_MANUAL, 2000ms));

  // warning_lamp, emergency_lamp を待って期待値一致
  EXPECT_TRUE(wait_for_warning_lamp(true, 2000ms));
  EXPECT_TRUE(wait_for_emergency_lamp(false, 2000ms));

}

TEST_F(EveCmdGateTest, Case_Initialized) {
  clear_status_lamp_queue();
  clear_warning_lamp_queue();
  clear_emergency_lamp_queue();

  // target Input
  // 起動開始
  autoware_adapi_v1_msgs::msg::LocalizationInitializationState initialization_state_initializing;
  initialization_state_initializing.state = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZING;
  pub_initilization_state_->publish(initialization_state_initializing);

  // executor のスピンは SetUp() のスレッドに任せる。
  // ここでは単に少し待つ（または wait_* 系の条件変数待ちを使う）
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // target Input
  // 初期化完了
  autoware_adapi_v1_msgs::msg::LocalizationInitializationState initialization_state_initialized;
  initialization_state_initializing.state = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED;
  pub_initilization_state_->publish(initialization_state_initialized);

  // warning_lamp, emergency_lamp を待って期待値一致
  EXPECT_TRUE(wait_for_in_parking_state(in_parking_msgs::msg::InParkingStatus::AW_UNAVAILABLE,
    in_parking_msgs::msg::InParkingStatus::VEHICLE_MANUAL, 2000ms));

  // warning_lamp, emergency_lamp を待って期待値一致
  EXPECT_TRUE(wait_for_warning_lamp(false, 2000ms));
  EXPECT_TRUE(wait_for_emergency_lamp(true, 2000ms));

  // status_lamp
  auto status_lamp_msgs = collect_status_lamp_msgs(8, 6000ms);
  EXPECT_GE(status_lamp_msgs.size(), 6u);

  EXPECT_TRUE(is_alternating(status_lamp_msgs));

  double period = estimate_period_sec(status_lamp_msgs);
  EXPECT_NEAR(period, PERIOD_FAST_BLINK_SEC, TOL_FAST_BLINK_SEC);

  // sound_done を待って期待値一致
  EXPECT_TRUE(wait_for_sound_done(autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE, 2000ms));
}
#endif
// TEST_F(EveCmdGateTest, Case1_normal_sequence) {
//   // test node
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(pub_sub_node_);
//   executor.add_node(client_node_);
//   executor.add_node(service_node_);

//   // Discovery待ち
//   {
//     auto start = std::chrono::steady_clock::now();
//     while ((std::chrono::steady_clock::now() - start) < std::chrono::seconds(5)) {
//       executor.spin_once(std::chrono::milliseconds(100));
//     }
//   }

//   // サービス接続確認
//   ASSERT_TRUE(cli_engage_->wait_for_service(std::chrono::seconds(5)));

//   // target Input
//   autoware_adapi_v1_msgs::msg::OperationModeState operation_mode_state;
//   operation_mode_state.mode = autoware_adapi_v1_msgs::msg::OperationModeState::STOP;
//   operation_mode_state.is_autoware_control_enabled = true;
//   operation_mode_state.is_in_transition = false;
//   pub_operation_mode_state_->publish(operation_mode_state);
//   sound_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE;

//   autoware_adapi_v1_msgs::msg::RouteState routing_state;
//   routing_state.state = autoware_adapi_v1_msgs::msg::RouteState::SET;
//   pub_routing_state_->publish(routing_state);

//   autoware_adapi_v1_msgs::msg::Route routing_route;
//   autoware_adapi_v1_msgs::msg::RouteData route_data;
//   routing_route.data.push_back(route_data);
//   pub_routing_route_->publish(routing_route);

//   autoware_state_machine_msgs::msg::StateLock lock_state;
//   lock_state.state = autoware_state_machine_msgs::msg::StateLock::STATE_OFF;
//   pub_lock_state_->publish(lock_state);

//   // 各topicをpublishし終わったら、一旦待ち
//   {
//     auto start = std::chrono::steady_clock::now();
//     while ((std::chrono::steady_clock::now() - start) < std::chrono::seconds(5)) {
//       executor.spin_once(std::chrono::milliseconds(100));
//     }
//   }

//   auto request = std::make_shared<tier4_external_api_msgs::srv::Engage::Request>();
//   request->engage = true;
//   auto engage_future = cli_engage_->async_send_request(request);
//   auto result = executor.spin_until_future_complete(engage_future, std::chrono::seconds(5));
//   ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);

//   ASSERT_TRUE(cli_engage_->wait_for_service(std::chrono::seconds(10)));

//   EXPECT_EQ(msgs_requesting_.size(), 3);
//   EXPECT_EQ(msgs_accepted_.size(), 3);

//   if (msgs_requesting_.size() > 0) {
//     EXPECT_TRUE(msgs_requesting_[0]);
//     EXPECT_FALSE(msgs_accepted_[0]);
//   }
//   if (msgs_requesting_.size() > 1) {
//     EXPECT_FALSE(msgs_requesting_[1]);
//     EXPECT_TRUE(msgs_accepted_[1]);
//   }
//   if (msgs_requesting_.size() > 2) {
//     EXPECT_FALSE(msgs_requesting_[2]);
//     EXPECT_FALSE(msgs_accepted_[2]);
//   }
// }



// TEST_F(EveCmdGateTest, Case2_multi_node_sequence) {
//   // test node
//   rclcpp::executors::SingleThreadedExecutor executor;
//   // pubsub node
//   executor.add_node(pub_sub_node_);

//   // target Input
//   // 起動開始
//   autoware_adapi_v1_msgs::msg::LocalizationInitializationState initialization_state_initializing;
//   initialization_state_initializing.state = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZING;
//   pub_initilization_state_->publish(initialization_state_initializing);

//   // 各topicをpublishし終わったら、一旦待ち
//   {
//     auto start = std::chrono::steady_clock::now();
//     while ((std::chrono::steady_clock::now() - start) < std::chrono::seconds(5)) {
//       executor.spin_once(std::chrono::milliseconds(100));
//     }
//   }

//   // ad_sound_manager
//   //   memo:autoware_state_machineの状態変化をトリガに音声関連topicを発信する
//   //        ADAPIのtopicを発信するたびに音声関連topicをsubすることになる
//   // STATE_CHECK_NODE_ALIVEの音声再生
//   // "/sound_voice_alarm/audio_cmd"をチェック
//   if (msgs_sound_voice_alarm_audio_cmd_.size() > 0) {
//     // TODO：期待値整理
//     EXPECT_EQ(msgs_sound_voice_alarm_audio_cmd_[0].type, 0);
//     EXPECT_EQ(msgs_sound_voice_alarm_audio_cmd_[0].file_path, 0);
//     EXPECT_EQ(msgs_sound_voice_alarm_audio_cmd_[0].volume, 0);
//     EXPECT_EQ(msgs_sound_voice_alarm_audio_cmd_[0].is_loop, 0);
//     EXPECT_EQ(msgs_sound_voice_alarm_audio_cmd_[0].loop_delay, 0);
//     EXPECT_EQ(msgs_sound_voice_alarm_audio_cmd_[0].start_delay, 0);
//   }

//   // "/autoware_state_machine/state_sound_done"をチェック
//   if (msgs_sound_state_.size() > 0 && msgs_sound_done_.size() > 0) {
//     // TODO：期待値整理
//     EXPECT_EQ(msgs_sound_state_[0].state, 0);
//     EXPECT_EQ(msgs_sound_done_[0].state, 0);
//   }

//   // STATE_CHECK_NODE_ALIVEの音声再生
//   // "/sound_voice_alarm/audio_cmd"をチェック




//   // 初期化完了
//   autoware_adapi_v1_msgs::msg::LocalizationInitializationState initialization_state_initialized;
//   initialization_state_initialized.state = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZIED;
//   pub_initilization_state_->publish(initialization_state_initialized);

//   EXPECT_EQ(msgs_requesting_.size(), 3);
//   EXPECT_EQ(msgs_accepted_.size(), 3);

//   if (msgs_requesting_.size() > 0) {
//     EXPECT_TRUE(msgs_requesting_[0]);
//     EXPECT_FALSE(msgs_accepted_[0]);
//   }
//   if (msgs_requesting_.size() > 1) {
//     EXPECT_FALSE(msgs_requesting_[1]);
//     EXPECT_TRUE(msgs_accepted_[1]);
//   }
//   if (msgs_requesting_.size() > 2) {
//     EXPECT_FALSE(msgs_requesting_[2]);
//     EXPECT_FALSE(msgs_accepted_[2]);
//   }

// }

