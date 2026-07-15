cmake_minimum_required(VERSION 3.10)
project(mloam_offline_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

enable_testing()
add_library(mloam_offline src/artifacts.cpp src/lidar_mapper.cpp src/manifest.cpp
                          src/pcd_preprocessor.cpp src/runner.cpp src/scenario.cpp
                          src/sequence_reader.cpp)
target_include_directories(mloam_offline PUBLIC include ${PCL_INCLUDE_DIRS})
target_compile_definitions(mloam_offline PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(mloam_offline PUBLIC Eigen3::Eigen yaml-cpp ${PCL_LIBRARIES})

add_executable(test_sequence_reader test/test_sequence_reader.cpp)
target_include_directories(test_sequence_reader PRIVATE include)
target_link_libraries(test_sequence_reader PRIVATE mloam_offline GTest::GTest GTest::Main)
add_test(NAME sequence_reader COMMAND test_sequence_reader)

add_executable(test_manifest test/test_manifest.cpp)
target_include_directories(test_manifest PRIVATE include)
target_link_libraries(test_manifest PRIVATE mloam_offline GTest::GTest GTest::Main)
add_test(NAME manifest COMMAND test_manifest)

add_executable(test_pcd_preprocessor test/test_pcd_preprocessor.cpp)
target_include_directories(test_pcd_preprocessor PRIVATE include)
target_link_libraries(test_pcd_preprocessor PRIVATE mloam_offline GTest::GTest GTest::Main)
add_test(NAME pcd_preprocessor COMMAND test_pcd_preprocessor)

add_executable(test_scenario_mapper test/test_scenario_mapper.cpp)
target_include_directories(test_scenario_mapper PRIVATE include)
target_link_libraries(test_scenario_mapper PRIVATE mloam_offline GTest::GTest GTest::Main)
add_test(NAME scenario_mapper COMMAND test_scenario_mapper)

add_executable(test_artifacts test/test_artifacts.cpp)
target_include_directories(test_artifacts PRIVATE include)
target_link_libraries(test_artifacts PRIVATE mloam_offline GTest::GTest GTest::Main)
add_test(NAME artifacts COMMAND test_artifacts)

add_executable(test_offline_integration test/test_offline_integration.cpp)
target_include_directories(test_offline_integration PRIVATE include)
target_link_libraries(test_offline_integration PRIVATE mloam_offline GTest::GTest GTest::Main)
add_test(NAME offline_integration COMMAND test_offline_integration)
