

include_directories(${PROJECT_SOURCE_DIR})
include_directories("lcm-types")
include_directories("/usr/local/include/lcm/")   # lcm includes

set(CMAKE_C_FLAGS "-std=gnu99")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(LINKER_FLAGS "-llcm" "-lpthread" "-lm")

file(GLOB SOURCES "*.cpp"  "*.c"  "lcm-types/*.c")
add_executable(simple_test ${SOURCES})

target_link_libraries(simple_test soem lcm m)
if(WIN32)
   target_link_libraries(simple_test wpcap.lib Packet.lib Ws2_32.lib Winmm.lib)
elseif(UNIX)
   target_link_libraries(simple_test pthread rt)
endif()

if(NOT DEFINED HOST_INSTALL)
   install(TARGETS simple_test DESTINATION bin)
endif()
