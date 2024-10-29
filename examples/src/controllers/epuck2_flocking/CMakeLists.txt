set(CMAKE_BUILD_TYPE Debug)  # Set the build type to Debug

add_library(epuck2_flocking MODULE epuck2_flocking.h epuck2_flocking.cpp)
target_link_libraries(epuck2_flocking
  argos3core_simulator
  argos3plugin_simulator_epuck2
  argos3plugin_simulator_genericrobot)

# Add compiler flags for debugging
# target_compile_options(epuck2_flocking PRIVATE -g)

