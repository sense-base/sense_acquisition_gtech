cmake_minimum_required(VERSION 3.8)
project(sense_eeg VERSION 0.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(sense_eeg main.cpp)
# Download some of our dependencies
# we'll need to update this to the newer API, but need sort out how to handle downloads first

file(DOWNLOAD https://raw.githubusercontent.com/sense-base/PRIVATE_gUSBamp-Linux-Driver-C-API/refs/heads/main/gUSBamp_Demo_1_16_01/main.cpp?token=$ENV{GTEC_TOKEN} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp STATUS DOWNLOAD_STATUS)
# Separate the returned status code, and error message.
list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
# Check if download was successful.
if(${STATUS_CODE} EQUAL 0)
  message(STATUS "main.cpp download completed successfully!")
else()
  # Exit CMake if the download failed, printing the error message.
  message(FATAL_ERROR "Error occurred during download of main.cpp: ${ERROR_MESSAGE}")
endif()

file(DOWNLOAD https://raw.githubusercontent.com/sense-base/PRIVATE_gUSBamp-Linux-Driver-C-API/refs/heads/main/gUSBampAPI_1_16_01/gAPI.h?token=$ENV{GTEC_TOKEN_H} ${CMAKE_CURRENT_SOURCE_DIR}/gUSBampAPI/gAPI.h STATUS DOWNLOAD_STATUS)
	
# Separate the returned status code, and error message.
list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
# Check if download was successful.
if(${STATUS_CODE} EQUAL 0)
	message(STATUS "gAPI.h download completed successfully!")
else()
  # Exit CMake if the download failed, printing the error message.
  message(FATAL_ERROR "Error occurred during download of gAPI.h: ${ERROR_MESSAGE}")
endif()

file(DOWNLOAD https://raw.githubusercontent.com/sense-base/PRIVATE_gUSBamp-Linux-Driver-C-API/refs/heads/main/gUSBampAPI_1_16_01/libgUSBampAPIso.so.1.16.01?token=$ENV{GTEC_TOKEN_SO} ${CMAKE_CURRENT_SOURCE_DIR}/gUSBampAPI/libgUSBampAPIso.so.1.16.01 STATUS DOWNLOAD_STATUS)

# Separate the returned status code, and error message.
list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
# Check if download was successful.
if(${STATUS_CODE} EQUAL 0)
	message(STATUS "libgUSBampAPIso.so.1.16.01 download completed successfully!")
else()
  # Exit CMake if the download failed, printing the error message.
  message(FATAL_ERROR "Error occurred during download of libgUSBampAPIso.so.1.16.01: ${ERROR_MESSAGE}")
endif()

file(CREATE_LINK ${CMAKE_CURRENT_SOURCE_DIR}/gUSBampAPI/libgUSBampAPIso.so.1.16.01 ${CMAKE_CURRENT_SOURCE_DIR}/gUSBampAPI/libgUSBampAPIso.so.1.16i SYMBOLIC)

file(CREATE_LINK ${CMAKE_CURRENT_SOURCE_DIR}/gUSBampAPI/libgUSBampAPIso.so.1.16.01 ${CMAKE_CURRENT_SOURCE_DIR}/gUSBampAPI/libgUSBampAPIso.so SYMBOLIC)

# put the boost dependencies for libgusbampapiso64 in the same directory
file(DOWNLOAD https://github.com/sense-base/boost_1_58_0/releases/download/0.0.7/libboost_date_time.so.1.58.0 ${CMAKE_CURRENT_SOURCE_DIR}/gUSBampAPI/libboost_date_time.so.1.58.0)
file(DOWNLOAD https://github.com/sense-base/boost_1_58_0/releases/download/0.0.7/libboost_regex.so.1.58.0 ${CMAKE_CURRENT_SOURCE_DIR}/gUSBampAPI/libboost_regex.so.1.58.0)
file(DOWNLOAD https://github.com/sense-base/boost_1_58_0/releases/download/0.0.7/libboost_signals.so.1.58.0 ${CMAKE_CURRENT_SOURCE_DIR}/gUSBampAPI/libboost_signals.so.1.58.0)
file(DOWNLOAD https://github.com/sense-base/boost_1_58_0/releases/download/0.0.7/libboost_system.so.1.58.0 ${CMAKE_CURRENT_SOURCE_DIR}/gUSBampAPI/libboost_system.so.1.58.0)
file(DOWNLOAD https://github.com/sense-base/boost_1_58_0/releases/download/0.0.7/libboost_thread.so.1.58.0 ${CMAKE_CURRENT_SOURCE_DIR}/gUSBampAPI/libboost_thread.so.1.58.0)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)

ament_target_dependencies(sense_eeg rclcpp)

# this is testing if we can link to libusb. Maybe not necessary.
#link_directories(AFTER /usr/lib/x86_64-linux-gnu/)
#target_link_libraries(turtlebot4_first_cpp_node libusb-1.0)

# Can we link to the gtec API
# I've not got this working properly. Hack with an absolute path works locally for me.
link_directories(AFTER ${CMAKE_CURRENT_SOURCE_DIR}/gUSBampAPI)
target_link_libraries(sense_eeg ${CMAKE_CURRENT_SOURCE_DIR}/gUSBampAPI/libgUSBampAPIso.so.1.16.01)

#target_link_libraries(sense_eeg libgusbampapiso64.so.1.11.00)

target_include_directories(sense_eeg PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/gUSBampAPI>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sense_eeg PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS sense_eeg
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
