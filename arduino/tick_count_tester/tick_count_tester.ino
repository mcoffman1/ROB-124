
//include ros libs
#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;
 
// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 4
#define ENC_IN_RIGHT_B 5
 
// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;
 
//========== ROS publishers =====================
std_msgs::Int16 right_wheel_ticks;
ros::Publisher rightpub("right_ticks", &right_wheel_ticks);
 

std_msgs::Int16 left_wheel_ticks;
ros::Publisher leftpub("left_ticks", &left_wheel_ticks);


// Increment the number of ticks
void right_wheel_tick() 
{
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);

  if (val == HIGH)
  {
    right_wheel_ticks.data = (right_wheel_ticks.data == encoder_maximum) ? encoder_minimum : right_wheel_ticks.data + 1;
  }
  else
  {
    right_wheel_ticks.data = (right_wheel_ticks.data == encoder_minimum) ? encoder_maximum : right_wheel_ticks.data - 1;
  }

}
 
// Increment the number of ticks
void left_wheel_tick() 
{
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if (val == HIGH)
  {
    left_wheel_ticks.data = (left_wheel_ticks.data == encoder_maximum) ? encoder_minimum : left_wheel_ticks.data + 1;
  }
  else
  {
    left_wheel_ticks.data = (left_wheel_ticks.data == encoder_minimum) ? encoder_maximum : left_wheel_ticks.data - 1;
  }
}
 
void setup() 
{
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT_PULLUP);
 
  // Every time the pin goes low, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, FALLING);

  // ros setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightpub);
  nh.advertise(leftpub);
}
 
void loop() 
{
  // Record the time
  currentMillis = millis();
 
  // If 100ms have passed, print the number of ticks
  if (currentMillis - previousMillis >= interval) 
  {   
    previousMillis = currentMillis;
     
    rightpub.publish( &right_wheel_ticks );
    leftpub.publish( &left_wheel_ticks );
    nh.spinOnce();
  }
}


