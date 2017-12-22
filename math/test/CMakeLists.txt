# add link directories (-L)
link_directories(${PROJECT_SOURCE_DIR}/lib)

# build binaries
if(${PLATFORM} MATCHES "Arduino")
  generate_arduino_firmware(test_matrix_arduino
      SRCS test_matrix_arduino.cpp
      LIBS blob_math
      PORT /dev/ttyACM0
      BOARD leonardo
      AFLAGS -cavr911)
else(${PLATFORM} MATCHES "Arduino")
  add_executable(test_matrix_linux test_matrix_linux.cpp) # build executable
  target_link_libraries(test_matrix_linux blob_math) # link libraries
  add_executable(test_vector_linux test_vector_linux.cpp) # build executable
  target_link_libraries(test_vector_linux blob_math) # link libraries
endif(${PLATFORM} MATCHES "Arduino")


