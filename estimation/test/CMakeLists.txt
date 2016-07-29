# add link directories (-L)
link_directories(${PROJECT_SOURCE_DIR}/lib)

add_executable(test_ukf_imu7z3q_linux test_ukf_imu7z3q_linux.cpp) # build executable
target_link_libraries(test_ukf_imu7z3q_linux blob_estimation blob_6dof) # link libraries

add_executable(test_cf_imu4z3q_linux test_cf_imu4z3q_linux.cpp) # build executable
target_link_libraries(test_cf_imu4z3q_linux blob_estimation blob_6dof) # link libraries
