// Copyright Autonomous Solutions Inc. 2016

#include "kvh/dsp3000_parser.h"
#include <cstdlib>

static bool is_important_symbol(char const c);
static int find_value_index(const char *buffer, const int buffer_size);
static int trim_buffer_up_to_rn(char *buffer, int buffer_size, int starting_index = 0);
static int find_mode_index(const char *buffer, const int buffer_size, int value_index);

bool is_important_symbol(char const c)
{
  return c != ' ' && c != '\r' && c != '\n';
}

int find_value_index(const char *buffer, const int buffer_size)
{
  int value_index = -1;
  bool space_found = false;
  bool found_important_symbol = false;
  for (int i = 0; i < buffer_size && !found_important_symbol; ++i)
  {
    if (' ' == buffer[i])
    {
      space_found = true;
    }
    else if (buffer[i] != '\r' && buffer[i] != '\n')
    {
      found_important_symbol = true;
      if (space_found)
      {
        value_index = i;
      }
    }
  }
  return value_index;
}

int trim_buffer_up_to_rn(char *const buffer, int const buffer_size, int const starting_index)
{
  int rn_index = -1;
  for (int i = starting_index; i < buffer_size && -1 == rn_index; ++i)
  {
    if ('\r' == buffer[i] && i + 1 < buffer_size && '\n' == buffer[i + 1])
    {
      rn_index = i + 1;
    }
  }

  int new_buffer_size;
  if (rn_index != -1)
  {
    new_buffer_size = buffer_size - rn_index - 1;
    for (int i = 0; i < new_buffer_size; ++i)
    {
      buffer[i] = buffer[i + rn_index + 1];
    }
  }
  else
  {
    new_buffer_size = buffer_size;
  }

  return new_buffer_size;
}

int find_mode_index(char const *const buffer, int const buffer_size, int const value_index)
{
  int mode_index = value_index;
  while (mode_index < buffer_size && is_important_symbol(buffer[mode_index]))
  {
    ++mode_index;
  }
  while (mode_index < buffer_size && buffer[mode_index] == ' ')
  {
    ++mode_index;
  }
  return mode_index;
}

ParseDsp3000Data parse_dsp3000(char *const buffer, int const buffer_size)
{
  bool parser_success = false;
  float value = 0.0;
  bool is_data_valid = false;

  int const value_index = find_value_index(buffer, buffer_size);

  if (-1 != value_index)
  {
    value = static_cast<float>(atof(&buffer[value_index]));

    int mode_index = find_mode_index(buffer, buffer_size, value_index);

    if (mode_index < buffer_size)
    {
      is_data_valid = '1' == buffer[mode_index];
      parser_success = '1' == buffer[mode_index] || '0' == buffer[mode_index];
    }
  }
  int const new_buffer_size = trim_buffer_up_to_rn(buffer, buffer_size);

  return ParseDsp3000Data(parser_success, new_buffer_size, value, is_data_valid);
}
