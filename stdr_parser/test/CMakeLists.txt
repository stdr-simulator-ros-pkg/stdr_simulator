catkin_add_gtest(stdr_parser_tools_test
  unit/stdr_parser_tools_test.cpp)
target_link_libraries(stdr_parser_tools_test
  stdr_parser
  gtest_main)

catkin_add_gtest(stdr_xml_parser_test
  unit/stdr_xml_parser_test.cpp)
target_link_libraries(stdr_xml_parser_test
  stdr_parser
  gtest_main)

catkin_add_gtest(stdr_parser_validator_test
  unit/stdr_parser_validator_test.cpp)
target_link_libraries(stdr_parser_validator_test
  stdr_parser
  gtest_main)

catkin_add_gtest(stdr_yaml_parser_test
  unit/stdr_yaml_parser_test.cpp)
target_link_libraries(stdr_yaml_parser_test
  stdr_parser
  gtest_main)

