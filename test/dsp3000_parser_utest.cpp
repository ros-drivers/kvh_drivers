/// Copyright Autonomous Solutions Inc. 2016

#include "kvh/dsp3000_parser.h"
#include "gtest/gtest.h"

TEST(KvhDsp3000Parser, normalInput)
{
  char input[] = "       0.015733   1\r\n";

  ParseDsp3000Data parsed_data(parse_dsp3000(input, sizeof(input) - 1));
  EXPECT_EQ(true, parsed_data.did_parser_succeed);
  EXPECT_EQ(0, parsed_data.new_buffer_length);
  EXPECT_FLOAT_EQ(0.015733, parsed_data.value);
  EXPECT_EQ(true, parsed_data.is_sensor_data_valid);
}

TEST(KvhDsp3000Parser, normalInputButNoSpace)
{
  char input[] = "0.015733   1\r\n";

  ParseDsp3000Data parsed_data(parse_dsp3000(input, sizeof(input) - 1));
  EXPECT_EQ(false, parsed_data.did_parser_succeed);
  EXPECT_EQ(0, parsed_data.new_buffer_length);
}

TEST(KvhDsp3000Parser, normalInputButNoSpaceAndEnd)
{
  char input[] = "0.015733   1";

  ParseDsp3000Data parsed_data(parse_dsp3000(input, sizeof(input) - 1));
  EXPECT_EQ(false, parsed_data.did_parser_succeed);
  EXPECT_EQ(sizeof(input) - 1, parsed_data.new_buffer_length);
}

TEST(KvhDsp3000Parser, onlyValue)
{
  int constexpr DATA_LEN = 17;
  static char const *const DATA = "       0.015733  ";
  char input[DATA_LEN + 1];

  memcpy(input, DATA, DATA_LEN + 1);
  ParseDsp3000Data parsed_data(parse_dsp3000(input, DATA_LEN));
  EXPECT_EQ(false, parsed_data.did_parser_succeed);
  EXPECT_EQ(DATA_LEN, parsed_data.new_buffer_length);
  ASSERT_LT(parsed_data.new_buffer_length, sizeof(input));
  ASSERT_GE(parsed_data.new_buffer_length, 0);
  input[parsed_data.new_buffer_length] = '\0';
  EXPECT_STREQ(DATA, input);
}

TEST(KvhDsp3000Parser, onlyValid)
{
  char input[] = "  1\r\n";

  ParseDsp3000Data parsed_data(parse_dsp3000(input, sizeof(input) - 1));
  EXPECT_EQ(false, parsed_data.did_parser_succeed);
  EXPECT_EQ(0, parsed_data.new_buffer_length);
}

TEST(KvhDsp3000Parser, onlyValidThenNormal)
{
  static char const *const EXPECTED_INTERMEDIATE_DATA = "       0.015733   1\r\n";
  char input[] = "  1\r\n       0.015733   1\r\n";

  ParseDsp3000Data const parsed_data_valid(parse_dsp3000(input, sizeof(input) - 1));
  EXPECT_EQ(false, parsed_data_valid.did_parser_succeed);
  EXPECT_EQ(strlen(EXPECTED_INTERMEDIATE_DATA), parsed_data_valid.new_buffer_length);
  ASSERT_LT(parsed_data_valid.new_buffer_length, sizeof(input));
  ASSERT_GE(parsed_data_valid.new_buffer_length, 0);
  input[parsed_data_valid.new_buffer_length] = '\0';
  ASSERT_STREQ(input, EXPECTED_INTERMEDIATE_DATA);
  ParseDsp3000Data const parsed_data_normal(parse_dsp3000(input, parsed_data_valid.new_buffer_length));
  EXPECT_EQ(true, parsed_data_normal.did_parser_succeed);
  EXPECT_EQ(0, parsed_data_normal.new_buffer_length);
  EXPECT_FLOAT_EQ(0.015733, parsed_data_normal.value);
  EXPECT_EQ(true, parsed_data_normal.is_sensor_data_valid);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
