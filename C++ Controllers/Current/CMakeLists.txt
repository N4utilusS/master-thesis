if(ARGOS_BUILD_FOR_SIMULATOR)
  add_library(e-puck_frontal_barrier_oc_controller MODULE frontal_barrier_oc.h frontal_barrier_oc.cpp)
  target_link_libraries(e-puck_frontal_barrier_oc_controller argos3plugin_${ARGOS_BUILD_FOR}_epuck)

  add_library(e-puck_frontal_barrier_human_oc_controller MODULE frontal_barrier_human_oc.h frontal_barrier_human_oc.cpp)
  target_link_libraries(e-puck_frontal_barrier_human_oc_controller argos3plugin_${ARGOS_BUILD_FOR}_epuck)

  # Create the loop function library
  add_library(frontal_barrier_lp SHARED frontal_barrier_lp.h frontal_barrier_lp.cpp)
  target_link_libraries(frontal_barrier_lp
  ${ARGOS_LIBRARIES}
  ${LUA_LIBRARIES}
  argos3plugin_simulator_entities)

  #add_subdirectory(NEAT.1.2.1)
  #add_subdirectory(../loop_functions)
else()
  add_executable(e-puck_frontal_barrier_oc_controller
    frontal_barrier_oc.h
    frontal_barrier_oc.cpp
    /home/adebruyn/argos3-iridia-tracking-system/src/plugins/robots/e-puck/real_robot/real_epuck_its_main.cpp)
#    ../plugins/robots/e-puck/real_robot/real_epuck_main.cpp)
  target_link_libraries(e-puck_frontal_barrier_oc_controller argos3plugin_${ARGOS_BUILD_FOR}_epuck 
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
    argos3plugin_${ARGOS_BUILD_FOR}_genericvirtualsensorsandactuators )

  add_executable(e-puck_frontal_barrier_human_oc_controller
    frontal_barrier_human_oc.h
    frontal_barrier_human_oc.cpp
    /home/adebruyn/argos3-epuck/src/plugins/robots/e-puck/real_robot/real_epuck_main.cpp)
  target_link_libraries(e-puck_frontal_barrier_human_oc_controller argos3plugin_${ARGOS_BUILD_FOR}_epuck
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot )

  #add_subdirectory(NEAT.1.2.1)
endif(ARGOS_BUILD_FOR_SIMULATOR)
