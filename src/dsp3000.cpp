/**
 *
 *  \file	dsp3000.cpp
 *  \brief      KVH DSP-3000 Fiber optic gyro controller.
 *              Parses data from DSP-3000 and publishes to dsp3000.
 *		Serial interface handled by cereal_port
 *  \author     Jeff Schmidt <jschmidt@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#include "cereal_port/CerealPort.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <math.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>

const int TIMEOUT = 1000;
const float PI = 3.14159265359f;

using namespace std;

bool configure_dsp3000(cereal::CerealPort *const device)
{
  bool output = true;

  // Start by zeroing the sensor.  Write three times, to ensure it is received
  // (according to datasheet)
  ROS_INFO("Zeroing the DSP-3000.");
  try
  {
    device->write("ZZZ", 3);
  }
  catch (cereal::TimeoutException &e)
  {
    ROS_ERROR("Unable to communicate with DSP-3000 device.");
    output = false;
  }
  // ros::Duration(0.1).sleep();

  if (output)
  {
    // Set to "Rate" output.  R=Rate, A=Incremental Angle, P=Integrated Angle
    ROS_INFO("Configuring for Rate output.");
    try
    {
      device->write("RRR", 3);
    }
    catch (cereal::TimeoutException &e)
    {
      ROS_ERROR("Unable to communicate with DSP-3000 device.");
    }
  }
  // ros::Duration(0.1).sleep();

  return output;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dsp3000");

  ros::NodeHandle n;

  // Grab the port name passed by the launch file.  If the launch file was not
  // used, or the desired
  // port is not present, default to /dev/ttyUSB0.
  string port_name;
  ros::param::param<std::string>("~port", port_name, "/dev/ttyUSB0");

  // Define the publisher topic name
  ros::Publisher dsp3000_pub = n.advertise<std_msgs::Float32>("dsp3000", 10);

  cereal::CerealPort device;

  // Open a port as defined in the launch file.  The DSP-3000 baud rate is
  // 38400.
  try
  {
    device.open(port_name.c_str(), 38400);
  }
  catch (cereal::Exception &e)
  {
    ROS_FATAL("Failed to open the serial port!!!");
    ROS_BREAK();
  }
  ROS_INFO("The serial port named \"%s\" is opened.", port_name.c_str());

  configure_dsp3000(&device);
  device.flush();
  static const int TEMP_BUFFER_SIZE = 128;
  char temp_buffer[TEMP_BUFFER_SIZE];
  while (ros::ok())
  {
    // Get the reply, the last value is the timeout in ms
    int temp_buffer_length = 0;
    try
    {
      temp_buffer_length =
          device.readLine(temp_buffer, TEMP_BUFFER_SIZE, TIMEOUT);
    }
    catch (cereal::TimeoutException &e)
    {
      ROS_ERROR("Unable to communicate with DSP-3000 device.");
    }

    static const char *ENDING_SEQUENCE = "\r\n";
    static const int ENDING_SEQUENCE_LENGTH = strlen(ENDING_SEQUENCE);
    string str(temp_buffer, temp_buffer_length - ENDING_SEQUENCE_LENGTH);
    stringstream ss(str);
    vector<string> tokens;
    string ss_buf;
    while (ss >> ss_buf)
      tokens.push_back(ss_buf);

    if (!(tokens.size() & 1))
    {
      // Extra loop to publish extra readings
      for (int offset = 0; offset < tokens.size() / 2; offset += 2)
      {
        const float rotate = atof(tokens[offset].c_str());
        const bool data_is_valid =
            (1 == atoi(tokens[offset + 1].c_str()) ? true : false);

        ROS_DEBUG("Raw DSP-3000 Output: %f", rotate);
        if (data_is_valid)
          ROS_DEBUG("Data is valid");

        // Declare the sensor message
        std_msgs::Float32 dsp_out;
        dsp_out.data = (rotate * PI) / 180.0f;

        // Publish the joint state message
        dsp3000_pub.publish(dsp_out);
      }
    }
    else
    {
      ROS_WARN("Bad data. Received data \"%s\" of length %i", ss.str().c_str(),
               static_cast<int>(tokens.size()));
    }

    ros::spinOnce();
  }

  return 0;
}
