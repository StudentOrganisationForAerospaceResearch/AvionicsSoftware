#
# Inital Setup
#

cmake_minimum_required(VERSION 3.14)
project(my_project)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

#
# Compilation
#

SET(GCC_COMPILE_FLAGS "-fpermissive")

set(
  RocketSources
  # ./Components/Communication/UARTTask.cpp
  ./Components/Core/Command.cpp
  # ./Components/Core/DMAController.cpp
  # ./Components/Core/Mutex.cpp
  # ./Components/Core/Queue.cpp
  # ./Components/Core/Task.cpp
  # ./Components/Core/Timer.cpp
  # ./Components/FlightControl/FlightTask.cpp
  # ./Components/FlightControl/RocketSM.cpp
  # ./Components/FlightControl/WatchdogTask.cpp
  ./Components/main_avionics.cpp
  # ./Components/Sensors/BarometerTask.cpp
  # ./Components/Sensors/IMUTask.cpp
  # ./Components/SoarDebug/DebugTask.cpp
  # ./Components/Utils.cpp
  # ./Core/Src/main.cpp
)

set(
  RocketHeaderDirs
  ./Components
  ./Components/Communication/Inc
  ./Components/Core/Inc
  ./Components/FlightControl/Inc
  ./Components/Sensors/Inc
  ./Components/SoarDebug/Inc
  # ./Core/Inc
)

set(
  FakesSources
  ./Tests/Fakes/os/FakeOs.cpp
  ./Tests/Fakes/stm/FakeHal.cpp
)

set(
  FakesHeaderDirs
  ./Tests/Fakes/os
  ./Tests/Fakes/stm
)

set(
  TestSources
  Tests/UnitTests/test1.cpp
  Tests/UnitTests/test2.cpp
)

# find -name '*.h' -not -path './build/*'
set(
  NonRocketHeaderDirs
  # ./Components/_Libraries/embedded-template-library/include/etl
  # ./Components/_Libraries/embedded-template-library/include/etl/atomic
  # ./Components/_Libraries/embedded-template-library/include/etl/deprecated
  # ./Components/_Libraries/embedded-template-library/include/etl/experimental
  # ./Components/_Libraries/embedded-template-library/include/etl/generators
  # ./Components/_Libraries/embedded-template-library/include/etl/mutex
  # ./Components/_Libraries/embedded-template-library/include/etl/private
  # ./Components/_Libraries/embedded-template-library/include/etl/profiles
  ./Core/Inc
  ./Drivers/CMSIS/Device/ST/STM32F4xx/Include
  ./Drivers/CMSIS/Include
  # ./Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
  ./Drivers/STM32F4xx_HAL_Driver/Inc
  ./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
  ./Middlewares/Third_Party/FreeRTOS/Source/include
  ./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
)

enable_testing()

add_compile_definitions(STM32F405xx)
add_compile_definitions(COMPUTER_ENVIRONMENT)

add_definitions(${GCC_COMPILE_FLAGS})

add_executable(
  tests
  ${FakesSources}
  ${RocketSources}
  ${TestSources}
)

# target_include_directories(tests SYSTEM PUBLIC ${NonRocketHeaderDirs})
target_include_directories(tests PUBLIC ${FakesHeaderDirs}) 
target_include_directories(tests PUBLIC ${RocketHeaderDirs})
# target_include_directories(tests PUBLIC ${FakesHeaderDirs} ${RocketHeaderDirs})

target_link_libraries(
  tests
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(tests)
