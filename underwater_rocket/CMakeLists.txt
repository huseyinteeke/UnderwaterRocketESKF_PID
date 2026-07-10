cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME underwater_rocket)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})




# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

file(GLOB_RECURSE USER_SOURCES 
    "Algorithm+Sensors/*.c" 
    "Algorithm+Sensors/*.cpp" 
    "Tasks/*.c" 
    "Tasks/*.cpp"
    "Thirdparty/*.c"
    "Thirdparty/*.s"
    "Thirdparty/*.S"
)


# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${USER_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Thirdparty/SEGGER/Config
    Thirdparty/SEGGER/OS
    Thirdparty/SEGGER/SEGGER
    Thirdparty/FreeRTOS
    Thirdparty/FreeRTOS/include
    Algorithm+Sensors/BNO055
    Algorithm+Sensors/MAESTRO
    Algorithm+Sensors/MS5837
    Algorithm+Sensors/Navigation
    Algorithm+Sensors/PID
    Tasks
    Thirdparty/FreeRTOS/portable/GCC/ARM_CM4F
    Thirdparty/SEGGER
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)


set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/STM32F407VGTX_FLASH.ld") 



# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    m
    ${CMAKE_SOURCE_DIR}/Thirdparty/libarm_cortexM4lf_math.a
    )



add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND arm-none-eabi-objcopy -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND arm-none-eabi-objcopy -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND arm-none-eabi-size $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Building .hex and .bin files, and printing size..."
)