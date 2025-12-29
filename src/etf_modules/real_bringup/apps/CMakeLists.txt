add_executable(test_move_xarm6 test_move_xarm6.cpp)
target_compile_features(test_move_xarm6 PUBLIC c_std_99 cxx_std_17)
target_link_libraries(test_move_xarm6 PUBLIC real_bringup)

add_executable(test_demo1 test_demo1.cpp)
target_compile_features(test_demo1 PUBLIC c_std_99 cxx_std_17)
target_link_libraries(test_demo1 PUBLIC real_bringup)

add_executable(test_demo2 test_demo2.cpp)
target_compile_features(test_demo2 PUBLIC c_std_99 cxx_std_17)
target_link_libraries(test_demo2 PUBLIC real_bringup)

add_executable(test_demo3 test_demo3.cpp)
target_compile_features(test_demo3 PUBLIC c_std_99 cxx_std_17)
target_link_libraries(test_demo3 PUBLIC real_bringup)

add_executable(test_planning test_planning.cpp)
target_compile_features(test_planning PUBLIC c_std_99 cxx_std_17)
target_link_libraries(test_planning PUBLIC real_bringup)

add_executable(test_task_planning test_task_planning.cpp)
target_compile_features(test_task_planning PUBLIC c_std_99 cxx_std_17)
target_link_libraries(test_task_planning PUBLIC real_bringup)

add_executable(test_dynamic_planning test_dynamic_planning.cpp)
target_compile_features(test_dynamic_planning PUBLIC c_std_99 cxx_std_17)
target_link_libraries(test_dynamic_planning PUBLIC real_bringup)

add_executable(test_dynamic_task_planning test_dynamic_task_planning.cpp)
target_compile_features(test_dynamic_task_planning PUBLIC c_std_99 cxx_std_17)
target_link_libraries(test_dynamic_task_planning PUBLIC real_bringup)

install(TARGETS 
  test_move_xarm6
  test_demo1
  test_demo2
  test_demo3
  test_planning
  test_task_planning
  test_dynamic_planning
  test_dynamic_task_planning
  DESTINATION lib/${PROJECT_NAME})