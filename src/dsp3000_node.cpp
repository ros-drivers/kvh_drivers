/**
 *
 *  \file	dsp3000_node.cpp
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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>
#include "kvh/dsp3000_mode.h"
#include "kvh/dsp3000_parser.h"
#include "kvh/serial_port.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"

static constexpr int TIMEOUT = 1000;
static constexpr float PI = 3.14159265359f;

using std::string;
using std::stringstream;
using std::vector;
using std::tuple;
using std::get;

/// @return serial string, mode name
tuple<string, string> get_mode_data(KvhDsp3000Mode mode);

bool configure_dsp3000(SerialPort *device, KvhDsp3000Mode mode);

string get_mode_topic_name(KvhDsp3000Mode const mode);

tuple<string, string> get_mode_data(KvhDsp3000Mode const mode)
{
  tuple<string, string> output;
  switch (mode)
  {
    case KVH_DSP3000_RATE:
      get<0>(output) = "RRR";
      get<1>(output) = "rate";
      break;
    case KVH_DSP3000_INCREMENTAL_ANGLE:
      get<0>(output) = "AAA";
      get<1>(output) = "incremental angle";
      break;
    case KVH_DSP3000_INTEGRATED_ANGLE:
      get<0>(output) = "PPP";
      get<1>(output) = "integrated angle";
      break;
    default:
      assert(!"mode not understood");
  }
  return output;
}

string get_mode_topic_name(KvhDsp3000Mode const mode)
{
  string output;
  switch (mode)
  {
    case KVH_DSP3000_RATE:
      output = "rate";
      break;
    case KVH_DSP3000_INCREMENTAL_ANGLE:
      output = "incremental_angle";
      break;
    case KVH_DSP3000_INTEGRATED_ANGLE:
      output = "integrated_angle";
      break;
    default:
      assert(!"mode not understood");
  }
  return output;
}

bool configure_dsp3000(SerialPort *const device, KvhDsp3000Mode const mode)
{
  bool output = true;

  ROS_INFO("Zeroing the DSP-3000.");
  try
  {
    // Start by zeroing the sensor.  Write three times, to ensure it is received
    device->write("ZZZ", 3);
  }
  catch (SerialTimeoutException &e)
  {
    ROS_ERROR("Unable to communicate with DSP-3000 device.");
    output = false;
  }

  if (output)
  {
    tuple<string, string> mode_data(get_mode_data(mode));
    ROS_INFO("Configuring for %s output.", get<1>(mode_data).c_str());
    try
    {
      device->write(get<0>(mode_data).c_str(), static_cast<int>(get<0>(mode_data).size()));
    }
    catch (SerialTimeoutException &e)
    {
      ROS_ERROR("Unable to communicate with DSP-3000 device.");
    }
  }

  return output;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dsp3000");

  string port_name;
  ros::param::param<std::string>("~port", port_name, "/dev/ttyUSB0");
  int32_t mode;
  ros::param::param<int32_t>("~mode", mode, KVH_DSP3000_RATE);
  if (mode != KVH_DSP3000_RATE && mode != KVH_DSP3000_INCREMENTAL_ANGLE && mode != KVH_DSP3000_INTEGRATED_ANGLE)
  {
    ROS_ERROR("bad mode: %d", mode);
    return EXIT_FAILURE;
  }
  bool invert = false;
  ros::param::param<bool>("~invert", invert, invert);

  // Define the publisher topic name
  ros::NodeHandle n;
  ros::Publisher dsp3000_pub =
      n.advertise<std_msgs::Float32>("dsp3000_" + get_mode_topic_name(static_cast<KvhDsp3000Mode>(mode)), 100);

  SerialPort device;

  try
  {
    device.open(port_name.c_str(), 38400);
  }
  catch (SerialException &e)
  {
    ROS_FATAL("%s", e.what());
    return EXIT_FAILURE;
  }
  ROS_INFO("The serial port named \"%s\" is opened.", port_name.c_str());

  configure_dsp3000(&device, static_cast<KvhDsp3000Mode>(mode));
  char temp_buffer[128];
  bool user_notified_of_timeout = false;
  int previous_errno = 0;
  bool ignoring_buffer_overflow = true;
  int temp_buffer_length = 0;
  uint8_t temporary_buffer_ignore_limit = 0U;
  bool streaming_data = false;
  while (ros::ok())
  {
    // Get the reply, the last value is the timeout in ms
    try
    {
      // Subtract 1 from sizeof(temp_buffer) because we will manually null terminate later
      int const new_bytes =
          device.readLine(&temp_buffer[temp_buffer_length], sizeof(temp_buffer) - temp_buffer_length - 1, TIMEOUT);
      temp_buffer_length += new_bytes;
      ignoring_buffer_overflow = false;
    }
    catch (SerialTimeoutException &e)
    {
      if (!user_notified_of_timeout)
      {
        ROS_ERROR("Timed out while talking with DSP-3000 device.");
        user_notified_of_timeout = true;
      }
      continue;
    }
    catch (SerialBufferFilledException &e)
    {
      if (!ignoring_buffer_overflow)
      {
        ROS_ERROR("%s", e.what());
      }
      temp_buffer_length = sizeof(temp_buffer) - 1;
    }
    catch (SerialException &e)
    {
      int32_t constexpr INTERRUPTED_SYSTEM_CALL_ERRNO = 4;
      if (INTERRUPTED_SYSTEM_CALL_ERRNO != errno && previous_errno != errno)
      {
        ROS_ERROR("%s", e.what());
      }
      previous_errno = errno;
      continue;
    }

    if (user_notified_of_timeout || 0 != previous_errno)
    {
      ROS_INFO("Receiving data");
      previous_errno = 0;
      user_notified_of_timeout = false;
    }

    if (!streaming_data)
    {
      streaming_data = true;
      ROS_INFO("streaming data");
    }

    bool parser_is_working = true;
    int previous_temp_buffer_length = temp_buffer_length;
    while (temp_buffer_length > 0 && parser_is_working)
    {
      ParseDsp3000Data const parsed_data(parse_dsp3000(temp_buffer, temp_buffer_length));
      parser_is_working = previous_temp_buffer_length != parsed_data.new_buffer_length;
      temp_buffer_length = parsed_data.new_buffer_length;
      previous_temp_buffer_length = temp_buffer_length;

      if (parsed_data.did_parser_succeed)
      {
        if (!parsed_data.is_sensor_data_valid)
        {
          ROS_ERROR("Sensor data is invalid");
        }
        else
        {
          // Declare the sensor message
          std_msgs::Float32 dsp_out;
          float const rotation_measurement_rad = (parsed_data.value * PI) / 180.0f;
          dsp_out.data = (invert ? -rotation_measurement_rad : rotation_measurement_rad);

          // Publish the joint state message
          dsp3000_pub.publish(dsp_out);
        }
      }
      else if (!ignoring_buffer_overflow && temporary_buffer_ignore_limit >= 100)
      {
        temp_buffer[temp_buffer_length] = '\0';
        ROS_WARN("Bad data. Received data \"%s\" of length %i", temp_buffer, temp_buffer_length);
      }
      else
      {
        ++temporary_buffer_ignore_limit;
      }
    }

    ros::spinOnce();
  }

  return 0;
}
