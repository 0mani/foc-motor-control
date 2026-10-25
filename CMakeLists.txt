cmake_minimum_required(VERSION 3.13)
project(foc_motor_control C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(FOC_BUILD_TESTS "Build host unit tests" ON)
option(FOC_BUILD_SIM_LIB "Build the shared library used by the Python simulator" ON)

set(FOC_SOURCES
  src/foc_math.c
  src/foc_pi.c
  src/foc_ctrl.c
)

# Static library for firmware projects
add_library(foc STATIC ${FOC_SOURCES})
target_include_directories(foc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(foc PRIVATE -Wall -Wextra -Wshadow -Wdouble-promotion -Werror)
target_link_libraries(foc PUBLIC m)

if(FOC_BUILD_SIM_LIB)
  # Shared library with a flat C API for the Python closed-loop simulator
  add_library(foc_sim SHARED ${FOC_SOURCES} sim/foc_sim_api.c)
  target_include_directories(foc_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_options(foc_sim PRIVATE -Wall -Wextra -Werror)
  target_link_libraries(foc_sim PUBLIC m)
endif()

if(FOC_BUILD_TESTS)
  enable_testing()
  add_executable(test_foc tests/test_foc.c)
  target_link_libraries(test_foc PRIVATE foc)
  target_compile_options(test_foc PRIVATE -Wall -Wextra -Werror)
  add_test(NAME foc_unit_tests COMMAND test_foc)
endif()
