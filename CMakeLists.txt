cmake_minimum_required(VERSION 3.8)
project(gtec_eeg_publisher VERSION 0.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Download some of our dependencies

set(GTEC_API_LOCATION https://raw.githubusercontent.com/sense-base/PRIVATE_gUSBamp-Linux-Driver-C-API/refs/heads/main)

file(DOWNLOAD
  ${GTEC_API_LOCATION}/gUSBampAPI_1_16_01/gAPI.h
  /usr/include/gAPI.h
  STATUS DOWNLOAD_STATUS USERPWD $ENV{GTEC_TOKEN})

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

file(DOWNLOAD
  ${GTEC_API_LOCATION}/gUSBampAPI_1_16_01/libgUSBampAPIso.so.1.16.01
  /usr/lib/libgUSBampAPIso.so.1.16.01
  STATUS DOWNLOAD_STATUS USERPWD $ENV{GTEC_TOKEN})

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

file(CREATE_LINK /usr/lib/libgUSBampAPIso.so.1.16.01
  /usr/lib/libgUSBampAPIso.so.1.16 SYMBOLIC)

file(CREATE_LINK /usr/lib/libgUSBampAPIso.so.1.16.01
  /usr/lib/libgUSBampAPIso.so SYMBOLIC)

# install the filter data files.
file(MAKE_DIRECTORY /etc/gtec/filter_files)
file(DOWNLOAD
  ${GTEC_API_LOCATION}/gUSBampAPI_1_16_01/DSPfilter.bin 
  /etc/gtec/filter_files/DSPfilter.bin 
  STATUS DOWNLOAD_STATUS USERPWD $ENV{GTEC_TOKEN})

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

file(DOWNLOAD
	${GTEC_API_LOCATION}/gUSBampAPI_1_16_01/DSPNotchfilter.bin 
	/etc/gtec/filter_files/DSPNotchfilter.bin 
  STATUS DOWNLOAD_STATUS USERPWD $ENV{GTEC_TOKEN})

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

# put the boost dependencies for libgusbampapiso64 in the same directory
set(BOOST_LOCATION https://github.com/sense-base/boost_1_58_0/releases/download/0.0.7)
file(DOWNLOAD ${BOOST_LOCATION}/libboost_date_time.so.1.58.0 /usr/lib/libboost_date_time.so.1.58.0)
file(DOWNLOAD ${BOOST_LOCATION}/libboost_regex.so.1.58.0 /usr/lib/libboost_regex.so.1.58.0)
file(DOWNLOAD ${BOOST_LOCATION}/libboost_signals.so.1.58.0 /usr/lib/libboost_signals.so.1.58.0)
file(DOWNLOAD ${BOOST_LOCATION}/libboost_system.so.1.58.0 /usr/lib/libboost_system.so.1.58.0)
file(DOWNLOAD ${BOOST_LOCATION}/libboost_thread.so.1.58.0 /usr/lib/libboost_thread.so.1.58.0)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eeg_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(gtec_eeg_publisher src/sense_eeg.cpp)
ament_target_dependencies(gtec_eeg_publisher rclcpp std_msgs eeg_msgs)


# Link to the gtec API
target_link_libraries(gtec_eeg_publisher libgUSBampAPIso.so.1.16.01)

target_include_directories(gtec_eeg_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(gtec_eeg_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


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

install(TARGETS gtec_eeg_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
