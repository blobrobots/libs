# add link directories (-L)
link_directories(${PROJECT_SOURCE_DIR}/lib)

# build binaries
if(${PLATFORM} MATCHES "Arduino")
  generate_arduino_firmware(test_brt_ino
      SRCS test_arduino.cpp
      LIBS blob_rt
      PORT /dev/ttyACM0
      BOARD leonardo
      AFLAGS -cavr911)
else(${PLATFORM} MATCHES "Arduino")
  add_executable(test_brt_lnx test_linux.cpp) # build executable
  target_link_libraries(test_brt_lnx blob_rt) # link libraries
endif(${PLATFORM} MATCHES "Arduino")


