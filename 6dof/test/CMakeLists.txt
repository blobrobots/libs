# add link directories (-L)
link_directories(${PROJECT_SOURCE_DIR}/lib)

# build binaries
if(${PLATFORM} MATCHES "Arduino")
  generate_arduino_firmware(test_bmat_ino
      SRCS test_bmat_arduino.cpp
      LIBS blob_6dof
      PORT /dev/ttyACM0
      BOARD leonardo
      AFLAGS -cavr911)
else(${PLATFORM} MATCHES "Arduino")
  add_executable(test_bmat_lnx test_bmat_linux.cpp) # build executable
  target_link_libraries(test_bmat_lnx blob_6dof) # link libraries
endif(${PLATFORM} MATCHES "Arduino")


