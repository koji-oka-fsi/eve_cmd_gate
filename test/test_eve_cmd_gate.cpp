
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <chrono>
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"

#include "autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp"
#include "autoware_state_machine_msgs/msg/state_lock.hpp"
#include "autoware_state_machine_msgs/msg/vehicle_button.hpp"
#include "shutdown_manager_msgs/msg/state_shutdown.hpp"
#include "dio_ros_driver/msg/dio_port.hpp"
#include "tier4_external_api_msgs/msg/operator.hpp"
#include <chrono>

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
  std::mutex mtx_resevation_lamp_;
  std::mutex mtx_shutdown_;
  std::mutex mtx_button_output_;
  std::mutex mtx_resevation_button_;

  // subscriberと期待値チェック同期
  std::condition_variable cv_resevation_lamp_;
  std::condition_variable cv_shutdown_;
  std::condition_variable cv_button_output_;
  std::condition_variable cv_resevation_button_;

  // テストノード
  std::shared_ptr<rclcpp::Node> reservation_button_mock_;
  std::shared_ptr<rclcpp::Node> button_output_selector_mock_;
  std::shared_ptr<rclcpp::Node> shutdown_mock_;
  std::shared_ptr<rclcpp::Node> eve_node_output_sub_;

  // Publisher (Target Node Input)
  rclcpp::Publisher<autoware_state_machine_msgs::msg::VehicleButton>::SharedPtr pub_button_;
  rclcpp::Publisher<autoware_state_machine_msgs::msg::VehicleButton>::SharedPtr pub_button_driver_;
  rclcpp::Publisher<shutdown_manager_msgs::msg::StateShutdown>::SharedPtr pub_shutdown_state_;
  rclcpp::Publisher<dio_ros_driver::msg::DIOPort>::SharedPtr pub_delivery_reservation_lamp_;
  rclcpp::Publisher<dio_ros_driver::msg::DIOArray>::SharedPtr din_port_array_publisher_;  

  // Subscriber (Target Node Output)
  rclcpp::Subscription<tier4_external_api_msgs::msg::Operator>::SharedPtr sub_get_operator_;
  rclcpp::Subscription<autoware_state_machine_msgs::msg::VehicleButton>::SharedPtr sub_button_;
  rclcpp::Subscription<autoware_state_machine_msgs::msg::VehicleButton>::SharedPtr sub_shutdown_button_;
  rclcpp::Subscription<shutdown_manager_msgs::msg::StateShutdown>::SharedPtr sub_shutdown_state_;
  rclcpp::Subscription<dio_ros_driver::msg::DIOArray>::SharedPtr dout_port_array_subscriber_;

  // lamp系のメッセージ
  std::deque<STLampDIO> msg_reservation_lamp_;

  //button系メッセージ
  autoware_state_machine_msgs::msg::VehicleButton::ConstSharedPtr msgs_button_driver_;
  autoware_state_machine_msgs::msg::VehicleButton::ConstSharedPtr button_state_;
  //shutdown系メッセージ
  shutdown_manager_msgs::msg::StateShutdown::ConstSharedPtr msg_shutdown_state_;

  void SetUp() override {
    msg_reservation_lamp_.clear();
    msgs_button_driver_.clear();
    button_state_.clear();
    msgs_sound_state_.state = autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED;
    msg_shutdown_state_.done = false;
    rclcpp::init(0, nullptr);
    reservation_button_mock_ = std::make_shared<rclcpp::Node>("test_reservation_button_mock_");
    button_output_selector_mock_ = std::make_shared<rclcpp::Node>("test_button_output_selector_mock_");
    shutdown_mock_ = std::make_shared<rclcpp::Node>("test_shutdown_mock_");
    eve_node_output_sub_ = std::make_shared<rclcpp::Node>("test_eve_node_output_sub_");

    // define reservation_button_mock_ start
    // Publisher
    pub_button_ = reservation_button_mock_->create_publisher<autoware_state_machine_msgs::msg::VehicleButton>(
    "/delivery_reservation_button_manager/output/delivery_reservation_button",
    rclcpp::QoS{1}.transient_local());
    // define reservation_button_mock_ end


    // define button_output_selector_mock_ start
    // Publisher
    pub_button_driver_ = button_output_selector_mock_->create_publisher<autoware_state_machine_msgs::msg::VehicleButton>(
      "/shutdown_button", rclcpp::QoS{3}.transient_local());
    // Subscriber
    sub_button_ = button_output_selector_mock_->create_subscription<autoware_state_machine_msgs::msg::VehicleButton>(
      "/delivery_reservation_button_manager/output/delivery_reservation_button", rclcpp::QoS{5}.transient_local(),
      [this](const autoware_state_machine_msgs::msg::VehicleButton::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_button_output_);
        msgs_button_driver_ = msg;
        cv_button_output_.notify_all();

        tier4_external_api_msgs::msg::Operator operator_mode;
        pub_button_driver_->publish(operator_mode);
      }
    );
    sub_get_operator_ = button_output_selector_mock_->create_subscription<Operator>(
    "/api/external/get/operator" rclcpp::QoS{5}.transient_local(),
    [this](const tier4_external_api_msgs::msg::Operator::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lock(mtx_button_output_);
      msgs_button_driver_ =msg->mode;
      cv_button_output_.notify_all();
    }
    );
    // define button_output_selector_mock_ end

    // define shutdown_mock_ start
    // piblisher
      pub_shutdown_state_ = shutdown_mock_->create_subscription<shutdown_manager_msgs::msg::StateShutdown>(
      "/shutdown_manager/state",rclcpp::QoS{5}.transient_local(),
    )
    // Subscriber
    sub_shutdown_button_ = shutdown_mock_->create_subscription<autoware_state_machine_msgs::msg::VehicleButton>(
      "/shutdown_button", rclcpp::QoS{5}.transient_local(),
      [this](const autoware_state_machine_msgs::msg::VehicleButton::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_shutdown_);
        button_state.data= msg->data;
        button_state.hold_down_time=msg->hold_down_time
        cv_shutdown_.notify_all();
      }
    );

    // define shutdown_mock_ end

    // define eve_node_output_sub_ start
    // Publisher
    pub_delivery_reservation_lamp_ = eve_node_output_sub_->create_publisher<dio_ros_driver::msg::DIOPort>(
      "delivery_reservation_lamp_out", rclcpp::QoS{3}.transient_local());
    // Subscriber
    sub_shutdown_state_ = eve_node_output_sub_->create_subscription<tier4_external_api_msgs::msg::ResponseStatus>(
      "/shutdown_manager/state", rclcpp::QoS{3}.transient_local(),
      [this](const shutdown_manager_msgs::msg::StateShutdown::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(cv_resevation_lamp_);
        msg_shutdown_state_ = msg->state;
        cv_resevation_lamp_.notify_all();
      }
    );
    // define eve_node_output_sub_ end

    // define xxx_mock start
    // Publisher
    // Subscriber
    // Cliengt
    // Service
    // define xxx_mock end

  }
  void TearDown() override {
    rclcpp::shutdown();
  }

  // output reservation_lamp キューをクリア
  void clear_reservation_lamp_queue() {
    {
      std::lock_guard<std::mutex> lock(cv_resevation_lamp_);
      msg_reservation_lamp_.clear();
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
    bool ok = cv_sound_done_.wait_for(lock, timeout, [this]{ return !msgs_sound_state_.state == autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED; });
    if (!ok) return false;

    return msgs_sound_state_.state == expected_sound_done_state;
  }

  // in_parking_state を待つ（timeout 以内、期待値チェックあり）
  bool wait_for_in_parking_state(int32_t expected_aw_state,
                         int32_t expected_vehicle_operation_mode,
                         std::chrono::milliseconds timeout = 2000ms) {
    std::unique_lock<std::mutex> lock(mtx_in_parking_state_);
    bool ok = cv_in_parking_state_.wait_for(lock, timeout, [this]{ return !msg_in_parking_state_; });
    if (!ok) return false;

    bool ret = false;
    if ((msg_in_parking_state_->aw_state == expected_aw_state)
      && (msg_in_parking_state_->vehicle_operation_mode == expected_vehicle_operation_mode)) {
      ret = true;
    }
    return ret;
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

TEST_F(EveCmdGateTest, Case_Initializing_SoundDone) {
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapi_mock_);
  executor.add_node(sound_voice_alarm_audio_driver_mock_);
  executor.add_node(cargo_loading_service_mock_);
  executor.add_node(eve_node_output_sub_);

  clear_status_lamp_queue();
  clear_warning_lamp_queue();
  clear_emergency_lamp_queue();

  // target Input
  // 起動開始
  autoware_adapi_v1_msgs::msg::LocalizationInitializationState initialization_state_initializing;
  initialization_state_initializing.state = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZING;
  pub_initilization_state_->publish(initialization_state_initializing);

  // warning_lamp, emergency_lamp を待って期待値一致
  ASSERT_TRUE(wait_for_in_parking_state(in_parking_msgs::msg::InParkingStatus::AW_UNAVAILABLE,
    in_parking_msgs::msg::InParkingStatus::VEHICLE_MANUAL, 2000ms));

  // warning_lamp, emergency_lamp を待って期待値一致
  ASSERT_TRUE(wait_for_warning_lamp(true, 2000ms));
  ASSERT_TRUE(wait_for_emergency_lamp(true, 2000ms));

  // status_lamp
  auto status_lamp_msgs = collect_status_lamp_msgs(8, 6000ms);
  ASSERT_GE(status_lamp_msgs.size(), 6u);

  EXPECT_TRUE(is_alternating(status_lamp_msgs));

  double period = estimate_period_sec(status_lamp_msgs);
  EXPECT_NEAR(period, PERIOD_SLOW_BLINK_SEC, TOL_SLOW_BLINK_SEC);

  // sound_done を待って期待値一致
  ASSERT_TRUE(wait_for_sound_done(autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE, 2000ms));

  // warning_lamp, emergency_lamp を待って期待値一致
  ASSERT_TRUE(wait_for_in_parking_state(in_parking_msgs::msg::InParkingStatus::AW_UNAVAILABLE,
    in_parking_msgs::msg::InParkingStatus::VEHICLE_MANUAL, 2000ms));

  // warning_lamp, emergency_lamp を待って期待値一致
  ASSERT_TRUE(wait_for_warning_lamp(true, 2000ms));
  ASSERT_TRUE(wait_for_emergency_lamp(false, 2000ms));

}

TEST_F(EveCmdGateTest, Case_Initialized) {
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapi_mock_);
  executor.add_node(sound_voice_alarm_audio_driver_mock_);
  executor.add_node(cargo_loading_service_mock_);
  executor.add_node(eve_node_output_sub_);

  clear_status_lamp_queue();
  clear_warning_lamp_queue();
  clear_emergency_lamp_queue();

  // target Input
  // 起動開始
  autoware_adapi_v1_msgs::msg::LocalizationInitializationState initialization_state_initializing;
  initialization_state_initializing.state = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZING;
  pub_initilization_state_->publish(initialization_state_initializing);

  // topicをpublishし終わったら、一旦待ち
  {
    auto start = std::chrono::steady_clock::now();
    while ((std::chrono::steady_clock::now() - start) < std::chrono::seconds(5)) {
      executor.spin_once(std::chrono::milliseconds(100));
    }
  }

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
  ASSERT_GE(status_lamp_msgs.size(), 6u);

  EXPECT_TRUE(is_alternating(status_lamp_msgs));

  double period = estimate_period_sec(status_lamp_msgs);
  EXPECT_NEAR(period, PERIOD_FAST_BLINK_SEC, TOL_FAST_BLINK_SEC);

  // sound_done を待って期待値一致
  EXPECT_TRUE(wait_for_sound_done(autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE, 2000ms));
}

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

