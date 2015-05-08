if(ARGOS_BUILD_FOR_SIMULATOR)
  add_library(console MODULE console.h client.h console.cpp client.cpp)
  target_link_libraries(console argos3plugin_${ARGOS_BUILD_FOR}_epuck)
  
  add_library(watcher MODULE watcher.h watcher.cpp)
  target_link_libraries(watcher argos3plugin_${ARGOS_BUILD_FOR}_epuck)
  
  add_library(tam_experiment_epuck MODULE tam_experiment_epuck.h tam_experiment_epuck.cpp)
  target_link_libraries(tam_experiment_epuck argos3plugin_${ARGOS_BUILD_FOR}_epuck)
  
  add_library(rab_selective_test MODULE rab_selective_test.h rab_selective_test.cpp)
  target_link_libraries(rab_selective_test argos3plugin_${ARGOS_BUILD_FOR}_epuck)

  add_library(frontal_barrier MODULE frontal_barrier.h frontal_barrier.cpp)
  target_link_libraries(frontal_barrier argos3plugin_${ARGOS_BUILD_FOR}_epuck)
  
  add_library(frontal_barrier_static MODULE frontal_barrier_static.h frontal_barrier_static.cpp)
  target_link_libraries(frontal_barrier_static argos3plugin_${ARGOS_BUILD_FOR}_epuck)

  add_library(rab_epuck_calibration_emitter MODULE rab_calibration_emitter.h rab_calibration_emitter.cpp)
  target_link_libraries(rab_epuck_calibration_emitter argos3plugin_${ARGOS_BUILD_FOR}_epuck)

  add_library(rab_epuck_calibration_receiver MODULE rab_calibration_receiver.h rab_calibration_receiver.cpp)
  target_link_libraries(rab_epuck_calibration_receiver argos3plugin_${ARGOS_BUILD_FOR}_epuck)

  add_library(e-puck_frontal_barrier_oc_controller MODULE frontal_barrier_oc.h frontal_barrier_oc.cpp)
  target_link_libraries(e-puck_frontal_barrier_oc_controller argos3plugin_${ARGOS_BUILD_FOR}_epuck)

  add_library(e-puck_frontal_barrier_human_oc_controller MODULE frontal_barrier_human_oc.h frontal_barrier_human_oc.cpp)
  target_link_libraries(e-puck_frontal_barrier_human_oc_controller argos3plugin_${ARGOS_BUILD_FOR}_epuck)

  #add_subdirectory(NEAT.1.2.1)
  add_subdirectory(loop_functions)
else()
  add_executable(console
    console.h
    console.cpp
    client.h
    client.cpp
    ../plugins/robots/e-puck/real_robot/real_epuck_main.cpp)
  target_link_libraries(console argos3plugin_${ARGOS_BUILD_FOR}_epuck)

  add_executable(watcher
    watcher.h
    watcher.cpp
    ../plugins/robots/e-puck/real_robot/real_epuck_main.cpp)
  target_link_libraries(watcher argos3plugin_${ARGOS_BUILD_FOR}_epuck)

  add_executable(calibration
    calibrate_epuck.h
    calibrate_epuck.cpp
    ../plugins/robots/e-puck/real_robot/real_epuck_main.cpp)
  target_link_libraries(calibration argos3plugin_${ARGOS_BUILD_FOR}_epuck)

  add_executable(frontal_barrier
    frontal_barrier.h
    frontal_barrier.cpp
    ../plugins/robots/e-puck/real_robot/real_epuck_main.cpp)
  target_link_libraries(frontal_barrier argos3plugin_${ARGOS_BUILD_FOR}_epuck)

  add_executable(frontal_barrier_static
    frontal_barrier_static.h
    frontal_barrier_static.cpp
    ../plugins/robots/e-puck/real_robot/real_epuck_main.cpp)
  target_link_libraries(frontal_barrier_static argos3plugin_${ARGOS_BUILD_FOR}_epuck)

  add_executable(rab_epuck_calibration_emitter
    rab_calibration_emitter.h 
    rab_calibration_emitter.cpp
    ../plugins/robots/e-puck/real_robot/real_epuck_main.cpp
  )
  target_link_libraries(rab_epuck_calibration_emitter argos3plugin_${ARGOS_BUILD_FOR}_epuck)

  add_executable(rab_epuck_calibration_receiver
    rab_calibration_receiver.h 
    rab_calibration_receiver.cpp
    ../plugins/robots/e-puck/real_robot/real_epuck_main.cpp
  )
  target_link_libraries(rab_epuck_calibration_receiver argos3plugin_${ARGOS_BUILD_FOR}_epuck)

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
    /home/adebruyn/argos3-iridia-tracking-system/src/plugins/robots/e-puck/real_robot/real_epuck_its_main.cpp)
#    ../plugins/robots/e-puck/real_robot/real_epuck_main.cpp)
  target_link_libraries(e-puck_frontal_barrier_human_oc_controller argos3plugin_${ARGOS_BUILD_FOR}_epuck
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
    argos3plugin_${ARGOS_BUILD_FOR}_genericvirtualsensorsandactuators )

  #add_subdirectory(NEAT.1.2.1)
endif(ARGOS_BUILD_FOR_SIMULATOR)
