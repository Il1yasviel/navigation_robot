cmake_minimum_required(VERSION 3.16)
project(navigation_robot_native_tests C)

enable_testing()
add_executable(protocol_tests
    test_protocols.c
    ../../components/common/src/crc16.c
    ../../components/protocols/src/host_frame.c
    ../../components/protocols/src/m0601c111_motor.c
)
target_compile_features(protocol_tests PRIVATE c_std_11)
target_compile_options(protocol_tests PRIVATE -Wall -Wextra -Werror)
target_include_directories(protocol_tests PRIVATE
    ../../components/common/include
    ../../components/protocols/include
)
add_test(NAME protocol_tests COMMAND protocol_tests)
