/// Copyright Autonomous Solutions Inc. 2016

#ifndef KVH_DSP3000_PARSER_H
#define KVH_DSP3000_PARSER_H

#include "kvh/dsp3000_mode.h"

struct ParseDsp3000Data
{
  inline ParseDsp3000Data(bool const in_did_parser_succeed, int const in_new_buffer_length, float const in_value,
                          bool const in_is_sensor_data_valid)
    : did_parser_succeed(in_did_parser_succeed)
    , new_buffer_length(in_new_buffer_length)
    , value(in_value)
    , is_sensor_data_valid(in_is_sensor_data_valid)
  {
  }

  bool did_parser_succeed;
  int new_buffer_length;
  float value;
  bool is_sensor_data_valid;
};

ParseDsp3000Data parse_dsp3000(char *buffer, int buffer_size);

#endif  // KVH_DSP3000_PARSER_H
