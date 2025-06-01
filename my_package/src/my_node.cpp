// my_node.cpp
#include "my_package/my_node.hpp"

MyNode::MyNode() : Node("my_node")
{
  // Callback groups
  cb_group_service_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_service_client_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_action_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  cb_group_action_client_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  cb_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_topic_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Service Server (deprecated warning suppressed by direct use of QoS)
  srv_ = this->create_service<std_srvs::srv::Trigger>(
    "trigger",
    std::bind(&MyNode::handle_service, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::QoS(10),
    cb_group_service_);

  // Service Client (deprecated warning suppressed by direct use of QoS)
  srv_client_ = this->create_client<std_srvs::srv::Trigger>(
    "trigger",
    rclcpp::QoS(10),
    cb_group_service_client_);

  // Action Server
  action_server_ = rclcpp_action::create_server<Fibonacci>(
    this,
    "fibonacci",
    std::bind(&MyNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MyNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&MyNode::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    cb_group_action_);

  // Action Client
  action_client_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci", cb_group_action_client_);

  // Publisher
  pub_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

  // Subscriber
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_topic_;
  sub_ = this->create_subscription<std_msgs::msg::String>(
    "chatter", 10,
    std::bind(&MyNode::topic_callback, this, std::placeholders::_1),
    sub_opts);

  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&MyNode::timer_callback, this),
    cb_group_timer_);
}

void MyNode::handle_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  res->success = true;
  res->message = "Service called successfully.";
}

void MyNode::call_service()
{
  if (!srv_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Service not available");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  srv_client_->async_send_request(request);
}

rclcpp_action::GoalResponse MyNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  const std::shared_ptr<const Fibonacci::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MyNode::handle_cancel(
  const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MyNode::handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  std::thread{std::bind(&MyNode::execute, this, goal_handle)}.detach();
}

void MyNode::execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  auto result = std::make_shared<Fibonacci::Result>();
  int a = 0, b = 1;
  result->sequence.push_back(a);
  result->sequence.push_back(b);
  for (int i = 2; i < goal_handle->get_goal()->order; ++i) {
    int next = a + b;
    result->sequence.push_back(next);
    a = b;
    b = next;
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Goal succeeded");
}

void MyNode::send_goal()
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Action server not available");
    return;
  }
  auto goal_msg = Fibonacci::Goal();
  goal_msg.order = 5;

  action_client_->async_send_goal(goal_msg);
}

void MyNode::topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
}

void MyNode::timer_callback()
{
  auto msg = std_msgs::msg::String();
  msg.data = "Hello from timer!";
  pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Published: '%s'", msg.data.c_str());

  call_service();
  send_goal();
} 
