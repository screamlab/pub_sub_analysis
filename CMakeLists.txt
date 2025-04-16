cmake_minimum_required(VERSION 3.8)
project(pub_sub_analysis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Install pre-commit hook
execute_process(
  COMMAND find ${CMAKE_CURRENT_SOURCE_DIR}/.git/hooks/ -type l -name "pre-commit" -delete || true
  COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/pre-commit.hook ${CMAKE_CURRENT_SOURCE_DIR}/.git/hooks/pre-commit
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  RESULT_VARIABLE HOOK_INSTALL_RESULT
  OUTPUT_QUIET ERROR_QUIET
)

if (HOOK_INSTALL_RESULT)
  message(WARNING "Failed to install pre-commit hook in ${CMAKE_CURRENT_SOURCE_DIR}.")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(sub src/sub.cpp)
add_executable(pub src/pub.cpp)
add_executable(tty src/tty.cpp)
target_include_directories(sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(sub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(tty PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  sub
  rclcpp
  trajectory_msgs)
ament_target_dependencies(
  pub
  rclcpp
  trajectory_msgs)


install(TARGETS
  sub
  pub
  tty
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
