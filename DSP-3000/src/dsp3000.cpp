#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "cereal_port/CerealPort.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <sstream>

#const TIMEOUT 1000

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dsp3000");

  ros::NodeHandle n;

  // Grab the port name passed by the launch file.  If the launch file was not used, or the desired
  // port is not present, default to /dev/ttyUSB0.
  string port_name;
  n.param<std::string>("port", port_name, "/dev/ttyUSB0");

  // Define the publisher topic name
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("dsp3000", 1000);

  //ros::Rate loop_rate(100);

  cereal::CerealPort device;

  char reply[64];
  float rotate;
  int valid;

  // Open a port as defined in the launch file.  The DSP-3000 baud rate is 38400.
  try{ device.open(port_name.c_str(), 38400); }
  catch(cereal::Exception& e)
  {
      ROS_FATAL("Failed to open the serial port!!!");
      ROS_BREAK();
  }
  ROS_INFO("The serial port is opened.");

  // Configure the DSP-3000.
  // Start by zeroing the sensor.  Write three times, to ensure it is received (according to datasheet)
  ROS_INFO("Zeroing the DSP-3000.");
  try{ device.write("ZZZ", 3); }
  catch(cereal::TimeoutException& e)
    {
      ROS_ERROR("Unable to communicate with DSP-3000 device.");
    }

  // Set to "Rate" output.  R=Rate, A=Incremental Angle, P=Integrated Angle
  ROS_INFO("Configuring for Rate output.");
  try{ device.write("RRR", 3); }
  catch(cereal::TimeoutException& e)
    {
      ROS_ERROR("Unable to communicate with DSP-3000 device.");
    }

  while (ros::ok())
  {
    
    // Get the reply, the last value is the timeout in ms
    try{ device.readLine(reply, TIMEOUT); }
    catch(cereal::TimeoutException& e)
      {
        ROS_ERROR("Timeout!");
      }

    string str(reply);
    string buf; // A buffer string
    stringstream ss(str); // Insert the string into a stream
    vector<string> tokens; // Create a vector to hold the words

    while (ss >> buf)
      tokens.push_back(buf);

    // Extract the data we want from the string vector.
    rotate = atof(tokens[0].c_str());
    valid = atoi(tokens[1].c_str());

    // Used for debugging.  The DSP-3000 outputs a "valid" flag as long as the
    //data being output is OK.
    ROS_DEBUG("DSP-3000 Output: %f", rotate);
    if (valid==1)
         ROS_DEBUG("Data is valid");
    

    //Declare the sensor message
    std_msgs::Float32 dsp_out;
    dsp_out.data = rotate;

    //Publish the joint state message
    chatter_pub.publish(dsp_out);

    ros::spinOnce();

    loop_rate.sleep();

  }

  return 0;

}
