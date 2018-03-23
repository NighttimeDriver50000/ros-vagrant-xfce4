#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/console.h>
#include <ros/ros.h>

mavros_msgs::State current_state;
// Called every time a new State message is processed.
void state_callback(const mavros_msgs::State::ConstPtr &msg) {
  current_state = *msg;
}

int main(int argc, char **argv) {
  // Start ROS.
  ros::init(argc, argv, "setpoint_test_node");
  ros::NodeHandle nh;

  // Set up subscriber, publisher, and service clients.
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
      "mavros/state", 10, &state_callback);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient
      <mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
      "mavros/set_mode");

  // Rate at which to send messages, in Hertz.
  ros::Rate rate(20.0);

  // Wait until connected.
  while (ros::ok() && !current_state.connected) {
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("FCU connected.");

  // Create target pose message.
  geometry_msgs::PoseStamped target;
  target.pose.position.x = 200;
  target.pose.position.y = 0;
  target.pose.position.z = 0;

  // Send it 100 times before switching mode to ensure the FCU is already
  // receiving it.
  for (int i = 0; ros::ok() && i < 100; ++i) {
    local_pos_pub.publish(target);
    ros::spinOnce();
    rate.sleep();
  }

  // Create service request messages.
  mavros_msgs::SetMode set_mode_guided;
  set_mode_guided.request.custom_mode = "GUIDED";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();
  // Minimum amount of time between requests, in seconds.
  ros::Duration min_request_period(5.0);

  while (ros::ok()) {
    if (current_state.mode != "GUIDED" && ros::Time::now() - last_request >
        min_request_period) {
      // If not in guided mode, switch modes.
      if (set_mode_client.call(set_mode_guided) &&
          set_mode_guided.response.mode_sent) {
        ROS_INFO("Guided mode enabled.");
      }
      last_request = ros::Time::now();
    } else if (!current_state.armed && ros::Time::now() - last_request >
        min_request_period) {
      // If not armed, arm.
      if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed.");
      }
      last_request = ros::Time::now();
    }

    // Continue publishing the target pose message.
    local_pos_pub.publish(target);

    ros::spinOnce();
    rate.sleep();
  }
}
