cmake_minimum_required(VERSION 3.8)
project(gtec_eeg_publisher VERSION 0.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Download dependencies. For g.tec drivers we use status codes to check for download success,
# as these are likely to go wrong if there is an error with GTEC_TOKEN.
# We use the EXPECTED_HASH option to check if the file is already present to avoid unnecessary repeat downloads.

set(GTEC_API_LOCATION https://raw.githubusercontent.com/sense-base/PRIVATE_gUSBamp-Linux-Driver-C-API/refs/heads/main)

file(DOWNLOAD
  ${GTEC_API_LOCATION}/gUSBampAPI_1_16_01/gAPI.h
  /usr/include/gAPI.h
  STATUS DOWNLOAD_STATUS USERPWD $ENV{GTEC_TOKEN}
  EXPECTED_HASH MD5=f1993aea6d7c79e669f84dfd1acb2237)

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
  STATUS DOWNLOAD_STATUS USERPWD $ENV{GTEC_TOKEN}
  EXPECTED_HASH MD5=11de08b8b860e227f4597202206327ff)

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
  STATUS DOWNLOAD_STATUS USERPWD $ENV{GTEC_TOKEN}
  EXPECTED_HASH MD5=0eb52456f930fd442b31b0f5059a16b2)

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
  STATUS DOWNLOAD_STATUS USERPWD $ENV{GTEC_TOKEN}
  EXPECTED_HASH MD5=a72be3241397f52bae4c7285d43145e8)

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

# Get the boost dependencies for libgusbampapiso64
set(BOOST_LOCATION https://github.com/sense-base/boost_1_58_0/releases/download/0.0.7)
file(DOWNLOAD ${BOOST_LOCATION}/libboost_date_time.so.1.58.0 /usr/lib/libboost_date_time.so.1.58.0
  EXPECTED_HASH MD5=fdd94573385551381f38b23ea491ec0e)
file(DOWNLOAD ${BOOST_LOCATION}/libboost_regex.so.1.58.0 /usr/lib/libboost_regex.so.1.58.0
  EXPECTED_HASH MD5=3ae740bdce7155732ddd445c8359eb0e)
file(DOWNLOAD ${BOOST_LOCATION}/libboost_signals.so.1.58.0 /usr/lib/libboost_signals.so.1.58.0
  EXPECTED_HASH MD5=45e94dcfc82b223f10ec06946e6a387c)
file(DOWNLOAD ${BOOST_LOCATION}/libboost_system.so.1.58.0 /usr/lib/libboost_system.so.1.58.0
  EXPECTED_HASH MD5=dff3891848d683c62cb1f06ca70718a2)
file(DOWNLOAD ${BOOST_LOCATION}/libboost_thread.so.1.58.0 /usr/lib/libboost_thread.so.1.58.0
  EXPECTED_HASH MD5=73739077c05260017a6abffb39e744d0)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eeg_msgs REQUIRED)

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
