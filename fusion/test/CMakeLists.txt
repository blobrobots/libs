# add link directories (-L)
link_directories(${PROJECT_SOURCE_DIR}/lib)

add_executable(test_imu7_z3q_linux test_imu7_z3q_linux.cpp) # build executable
target_link_libraries(test_imu7_z3q_linux blob_fusion blob_6dof) # link libraries

