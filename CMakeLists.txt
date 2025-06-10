cmake_minimum_required(VERSION 3.8)
project(turtlebot4_cpp_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Download some of our dependencies
file(DOWNLOAD https://github.com/BrainTech/openbci/raw/refs/heads/master/obci/drivers/eeg/cpp_amplifiers/gtec/api/libgusbampapiso64.so.1.11.00 libgusbampapiso64.so.1.11.00)

file(DOWNLOAD https://github.com/sense-base/boost_1_58_0/releases/download/0.0.7/libboost_date_time.so.1.58.0 libboost_date_time.so.1.58.0)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(turtlebot4_first_cpp_node src/turtlebot4_first_cpp_node.cpp)
ament_target_dependencies(turtlebot4_first_cpp_node rclcpp)

# this is testing if we can link to libusb. Maybe not necessary.
link_directories(AFTER /usr/lib/x86_64-linux-gnu/)
target_link_libraries(turtlebot4_first_cpp_node libusb-1.0)


target_include_directories(turtlebot4_first_cpp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(turtlebot4_first_cpp_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS turtlebot4_first_cpp_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
