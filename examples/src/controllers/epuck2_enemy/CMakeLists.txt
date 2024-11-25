# Set Debug build type (if you need debugging for this module specifically)
set(CMAKE_BUILD_TYPE Debug)

# Define the enemy robot controller library
add_library(epuck2_enemy MODULE robot_xaxis_controller.cpp robot_xaxis_controller.h)

# Link necessary ARGoS libraries
target_link_libraries(epuck2_enemy
  argos3core_simulator
  argos3plugin_simulator_epuck2
  argos3plugin_simulator_genericrobot
)

# Add compiler flags for debugging
target_compile_options(epuck2_enemy PRIVATE -g)

