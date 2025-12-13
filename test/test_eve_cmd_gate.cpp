
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <chrono>
#include <mutex>                
#include <condition_variable>
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp"
#include "autoware_state_machine_msgs/msg/state_lock.hpp"
#include "autoware_state_machine_msgs/msg/vehicle_button.hpp"
#include "shutdown_manager_msgs/msg/state_shutdown.hpp"
#include "dio_ros_driver/msg/dio_port.hpp"
#include "dio_ros_driver/msg/dio_array.hpp"
#include "tier4_external_api_msgs/msg/operator.hpp"

using namespace std::chrono_literals;
// 期待周期と許容誤差
static const double PERIOD_TWO_BLINKS_UNTIL_EXPIRATION = 2.1;   // state=0 時の W 出力周期
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
  std::mutex mtx_reservation_lamp_;
  std::mutex mtx_shutdown_;
  std::mutex mtx_dio_ros_driver_;

  // subscriberと期待値チェック同期
  std::condition_variable cv_reservation_lamp_;
  std::condition_variable cv_shutdown_;
  std::condition_variable cv_dio_ros_driver_;

  // テストノード
  std::shared_ptr<rclcpp::Node> shutdown_mock_;
  std::shared_ptr<rclcpp::Node> eve_node_output_sub_;
  std::shared_ptr<rclcpp::Node> dio_ros_driver_sub_;

  // Publisher (Target Node Input)
  rclcpp::Publisher<shutdown_manager_msgs::msg::StateShutdown>::SharedPtr pub_shutdown_state_;
  rclcpp::Publisher<dio_ros_driver::msg::DIOPort>::SharedPtr pub_delivery_reservation_lamp_;

  // Subscriber (Target Node Output)
  rclcpp::Subscription<shutdown_manager_msgs::msg::StateShutdown>::SharedPtr sub_shutdown_state_;
  rclcpp::Subscription<dio_ros_driver::msg::DIOPort>::SharedPtr sub_delivery_reservation_lamp_;

  // lamp系のメッセージ
  std::deque<STLampDIO> msg_reservation_lamp_;

  //shutdown系メッセージ
  shutdown_manager_msgs::msg::StateShutdown msg_shutdown_state_;

  void SetUp() override {
    msg_reservation_lamp_.clear();
    rclcpp::init(0, nullptr);
    shutdown_mock_ = std::make_shared<rclcpp::Node>("test_shutdown_mock_");
    eve_node_output_sub_ = std::make_shared<rclcpp::Node>("test_eve_node_output_sub_");
    dio_ros_driver_sub_ = std::make_shared<rclcpp::Node>("test_dio_ros_driver_sub_");
   
    // define shutdown_mock_ start
    // publisher
      pub_shutdown_state_ = shutdown_mock_->create_publisher<shutdown_manager_msgs::msg::StateShutdown>(
      "/shutdown_manager/state",rclcpp::QoS{5}.transient_local()
    );

    // define shutdown_mock_ end

    // define eve_node_output_sub_ start
    // Publisher
    pub_delivery_reservation_lamp_ = eve_node_output_sub_->create_publisher<dio_ros_driver::msg::DIOPort>(
      "/dio/dout3", rclcpp::QoS{3}.transient_local());
    // Subscriber
    sub_shutdown_state_ = eve_node_output_sub_->create_subscription< shutdown_manager_msgs::msg::StateShutdown>(
      "/shutdown_manager/state", rclcpp::QoS{3}.transient_local(),
      [this](const shutdown_manager_msgs::msg::StateShutdown::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(mtx_reservation_lamp_);
        msg_shutdown_state_.state = msg->state;
        cv_reservation_lamp_.notify_all();
      }
    );
    //dio_ros_sub start
    sub_delivery_reservation_lamp_ = dio_ros_driver_sub_->create_subscription<dio_ros_driver::msg::DIOPort>(
      "/dio/dout3", rclcpp::QoS{3}.transient_local(),
      [this](const dio_ros_driver::msg::DIOPort::SharedPtr msg)
      {
      std::lock_guard<std::mutex> lock(mtx_dio_ros_driver_);
      msg_reservation_lamp_.push_back({msg->value, std::chrono::steady_clock::now()});
      cv_dio_ros_driver_.notify_all();
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
      std::lock_guard<std::mutex> lock(mtx_reservation_lamp_);
      msg_reservation_lamp_.clear();
    }
  }

  // delivery_lamp操作 を N 件収集（timeout 以内）
  std::vector<STLampDIO> collect_delivery_lamp_msgs(size_t n,
                                         std::chrono::milliseconds timeout) {
    std::vector<STLampDIO> out;
    out.reserve(n);
    auto deadline = std::chrono::steady_clock::now() + timeout;

    while (out.size() < n) {
      std::unique_lock<std::mutex> lock(mtx_dio_ros_driver_);
      if (msg_reservation_lamp_.empty()) {
        auto now = std::chrono::steady_clock::now();
        if (now >= deadline) break;
        cv_dio_ros_driver_.wait_until(lock, deadline, [this]{ return !msg_reservation_lamp_.empty(); });
        if (msg_reservation_lamp_.empty()) break;
      }
      out.push_back(msg_reservation_lamp_.front());
      msg_reservation_lamp_.pop_front();
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

  bool waiting_reservation_pub(bool expected,std::chrono::milliseconds timeout){
   auto deadline = std::chrono::steady_clock::now() + timeout;
    std::unique_lock<std::mutex> lock(mtx_dio_ros_driver_);

    while (std::chrono::steady_clock::now() < deadline) {
    // 最新値があればチェック
    if (!msg_reservation_lamp_.empty()) {
      bool last = msg_reservation_lamp_.back().value;
      if (last == expected) {
        return true;
      }
    }
      // 次のメッセージを待つ
      cv_dio_ros_driver_.wait_until(lock, deadline);
    }
      return false;
  }
};


TEST_F(EveCmdGateTest, Case_shutdown_button_was_pressed_for_the_first_time) {
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(shutdown_mock_);
  executor.add_node(eve_node_output_sub_);
  executor.add_node(dio_ros_driver_sub_);
  clear_reservation_lamp_queue();
  //shutdownstateがshutdouwn_managerから渡される
  shutdown_manager_msgs::msg::StateShutdown shutdown_msg ;
  shutdown_msg.state = shutdown_manager_msgs::msg::StateShutdown::STATE_INACTIVE_FOR_SHUTDOWN;
  pub_shutdown_state_ -> publish(shutdown_msg);
  
  // reservation_lamp 0.5秒間隔で点滅
  auto delivery_lamp_msgs = collect_delivery_lamp_msgs(6, 1000ms);
  ASSERT_GE(delivery_lamp_msgs.size(), 4u);           // 少なくとも4件必要
  EXPECT_TRUE(is_alternating(delivery_lamp_msgs));
  double period = estimate_period_sec(delivery_lamp_msgs);
  EXPECT_NEAR(period, PERIOD_FAST_BLINK_SEC, TOL_FAST_BLINK_SEC);
  //  reservation_publish
  EXPECT_TRUE(waiting_reservation_pub(true,2000ms));

}

TEST_F(EveCmdGateTest, Case_shutdown_button_was_pressed_for_the_second_time) {
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(shutdown_mock_);
  executor.add_node(eve_node_output_sub_);
  executor.add_node(dio_ros_driver_sub_);
  clear_reservation_lamp_queue();
  //shutdownstateがshutdouwn_managerから渡される
  shutdown_manager_msgs::msg::StateShutdown shutdown_msg ;
  shutdown_msg.state = shutdown_manager_msgs::msg::StateShutdown::STATE_SUCCESSFUL_SHUTDOWN_INITIATION;
  pub_shutdown_state_ -> publish(shutdown_msg);
  
  // status_lamp
  auto delivery_lamp_msgs = collect_delivery_lamp_msgs(12, 6300ms);
  ASSERT_GE(delivery_lamp_msgs.size(), 12u);

  EXPECT_TRUE(is_alternating(delivery_lamp_msgs));

  double period = estimate_period_sec(delivery_lamp_msgs);
  EXPECT_NEAR(period, PERIOD_TWO_BLINKS_UNTIL_EXPIRATION, TOL_FAST_BLINK_SEC);

  //  reservation_publish
  EXPECT_TRUE(waiting_reservation_pub(true,2000ms));
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

