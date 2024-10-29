add_library(flocking_loop_functions MODULE
  flocking_loop_functions.h flocking_loop_functions.cpp)

target_link_libraries(flocking_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})

