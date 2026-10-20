cmake_minimum_required(VERSION 3.8)
project(cine_config)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(cine_config_cpp src/constants.cpp)
target_include_directories(cine_config_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(cine_config_cpp PUBLIC ament_index_cpp::ament_index_cpp)
set_target_properties(cine_config_cpp PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS cine_config_cpp
  EXPORT export_cine_config
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_cine_config test
    WORKING_DIRECTORY /tmp
  )
endif()

ament_export_targets(export_cine_config HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_index_cpp)
ament_package()
