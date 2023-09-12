add_executable(pointcloud_combiner pointcloud_combiner.cpp)
target_compile_features(pointcloud_combiner PUBLIC c_std_99 cxx_std_17)
target_link_libraries(pointcloud_combiner PUBLIC perception_etflab_library)

add_executable(object_segmentation object_segmentation.cpp)
target_compile_features(object_segmentation PUBLIC c_std_99 cxx_std_17)
target_link_libraries(object_segmentation PUBLIC perception_etflab_library)

install(TARGETS
  	pointcloud_combiner
  	object_segmentation
  	DESTINATION lib/${PROJECT_NAME}
)
