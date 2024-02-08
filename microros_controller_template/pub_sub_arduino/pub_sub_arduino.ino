#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>

#define LED_PIN 13

//////////// variables ////////////////

int mode = 0;

//////////// ros //////////////////////
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray sensors_msg;
std_msgs__msg__Float32MultiArray cmd_msg;
rclc_executor_t executor_pub;
rclc_executor_t executor_sub;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define PUBSIZE 5

static float pub_data[PUBSIZE];


void setup() {

//////////////////////////// ROS INIT ///////////////////////////////////
  set_microros_transports();

  sensors_msg.data.capacity = PUBSIZE;
  sensors_msg.data.size = PUBSIZE;
  sensors_msg.data.data = pub_data;


  cmd_msg.data.capacity = 100; 
  cmd_msg.data.size = 0;
  cmd_msg.data.data = (float*) malloc(cmd_msg.data.capacity * sizeof(float));

  cmd_msg.layout.dim.capacity = 100;
  cmd_msg.layout.dim.size = 0;
  cmd_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(cmd_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
  
  for(size_t i = 0; i < cmd_msg.layout.dim.capacity; i++){
      cmd_msg.layout.dim.data[i].label.capacity = 20;
      cmd_msg.layout.dim.data[i].label.size = 0;
      cmd_msg.layout.dim.data[i].label.data = (char*) malloc(cmd_msg.layout.dim.data[i].label.capacity * sizeof(char));
  }
  
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/sensors"));

  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    publisher_callback));

      // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/cmd"));

  // create executor
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
  
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &cmd_msg, &subscription_callback, ON_NEW_DATA));  
}

void loop() {

  RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(10)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(10)));
  delay(10);

}


/////////////////////////////////// ROS2 /////////////////////////////////////////////////

////////////////// ros errors /////////////////////
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

////////////////// ros pub //////////////////////
void publisher_callback(rcl_timer_t * timer, int64_t last_call_time){  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    
    sensors_msg.data.data[0] = mode;
    
    RCSOFTCHECK(rcl_publish(&publisher, &sensors_msg, NULL));
  }
}

////////////// ros sub ///////////////////////////
void subscription_callback(const void * msgin){  
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  digitalWrite(LED_PIN, (int(msg->data.data[2]) == 0) ? LOW : HIGH);  \
  
  mode = msg->data.data[0];
}
