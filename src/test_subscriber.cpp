#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/set_bool.hpp"


using namespace std::chrono_literals; // NOLINT

class DataPublisher : public rclcpp::Node
{
  public:
    DataPublisher()
    : Node("data_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::Int32>("/topic", 10);
      auto callback = [this]() {
        auto msg = std_msgs::msg::Int32();
        msg.data =count_;
        publisher_->publish(msg);
        count_++;
      };
      timer_ = this->create_wall_timer(500ms, callback);
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    size_t count_;
};

class TaskServer : public rclcpp::Node
{
  public:
    TaskServer()
    : Node("task_server")
    {
      // get params
      declare_parameter("use_dedicated_thread", use_dedicated_thread_);
      get_parameter("use_dedicated_thread", use_dedicated_thread_);
      // create service
      auto callback = [this](
        const std::shared_ptr<rmw_request_id_t>/*request_header*/,
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          (void)response;
          if (request->data) {
            activate();
          } else {
            deactivate();
          }
      };
      service_ = create_service<std_srvs::srv::SetBool>("/activate", callback);
      if (use_dedicated_thread_) {
          callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
          executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
          executor_->add_callback_group(callback_group_, get_node_base_interface());
          executor_thread_ = std::make_unique<std::thread>([this]() {executor_->spin();});
      }
    }
    ~TaskServer(){
      if (use_dedicated_thread_) {
        executor_->cancel();
        executor_thread_->join();
      }
    }
  private:
    void activate() {
      RCLCPP_INFO(this->get_logger(), "activate");
      auto callback = [this](const std_msgs::msg::Int32::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "recv data: %d.", msg->data);
      };
      auto sub_opt = rclcpp::SubscriptionOptions();
      sub_opt.callback_group = callback_group_;
      subscription_ = this->create_subscription<std_msgs::msg::Int32>("/topic", 10, callback, sub_opt);
    }
    void deactivate() {
      RCLCPP_INFO(this->get_logger(), "deactivate");
      subscription_.reset();
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_{nullptr};
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    bool active_{false};
    // dedicated thread for subscriber
    bool use_dedicated_thread_{false};
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::shared_ptr<std::thread> executor_thread_;
};


class TaskClient : public rclcpp::Node
{
  public:
    TaskClient()
    : Node("task_client")
    {
      client_ = create_client<std_srvs::srv::SetBool>("/activate");
    }
    void command(bool activate) {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = activate;

      while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
      }

      auto result = client_->async_send_request(request);
      // Wait for the result.
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "client success");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service add_two_ints");
      }
    }
  private:
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
};


template<typename NodeT>
std::shared_ptr<std::thread> create_spin_thread(NodeT & node)
{
  return std::make_shared<std::thread>(
    [node]() {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node->get_node_base_interface());
      executor.spin();
      executor.remove_node(node->get_node_base_interface());
    });
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // create node
  auto pub = std::make_shared<DataPublisher>();
  auto server = std::make_shared<TaskServer>();
  auto client = std::make_shared<TaskClient>();
  // spin threads
  std::vector<std::shared_ptr<std::thread>> threads;
  threads.push_back(create_spin_thread(pub));
  threads.push_back(create_spin_thread(server));
  std::this_thread::sleep_for(1000ms);
  // activate and deactivate
  while(rclcpp::ok()) {
    client->command(true);
    std::this_thread::sleep_for(3000ms);
    client->command(false);
    std::this_thread::sleep_for(3000ms);
  }
  for (auto t : threads) {
    t->join();
  }
  rclcpp::shutdown();
}