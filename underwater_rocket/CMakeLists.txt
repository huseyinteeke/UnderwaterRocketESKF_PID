cmake_minimum_required(VERSION 3.22)

project(underwater_rocket C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(${CMAKE_PROJECT_NAME})

add_subdirectory(cmake/stm32cubemx)

# Linker Script (Bellek haritası) ve optimizasyon ayarları
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F407VGTX_FLASH.ld
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)


target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/CMSIS/DSP/Lib/GCC/libarm_cortexM4lf_math.a"
    m
)


target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM4             # Cortex-M4 
    __FPU_PRESENT=1          
)




add_compile_options(
    -mcpu=cortex-m4 
    -mthumb 
    -mthumb-interwork
    -mfloat-abi=hard 
    -mfpu=fpv4-sp-d16
    -ffunction-sections
    -fdata-sections
)

add_link_options(
    -mcpu=cortex-m4 
    -mthumb 
    -mthumb-interwork
    -mfloat-abi=hard 
    -mfpu=fpv4-sp-d16
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,--gc-sections
)


aux_source_directory(Algorithm+Sensors/BNO055 BNO_SOURCES)
aux_source_directory(Algorithm+Sensors/MAESTRO MAESTRO_SOURCES)
aux_source_directory(Algorithm+Sensors/MS5837 MS5837_SOURCES)
aux_source_directory(Algorithm+Sensors/PID PID_SOURCES)
aux_source_directory(Algorithm+Sensors/Navigation NAVIGATION_SOURCES)
aux_source_directory(Tasks TASKS_SOURCES)


aux_source_directory(Thirdparty/FreeRTOS RTOS_SOURCES)
aux_source_directory(Thirdparty/FreeRTOS/portable/GCC/ARM_CM4F RTOS_PORT_SOURCES)
aux_source_directory(Thirdparty/FreeRTOS/portable/MemMang RTOS_MEM_SOURCES)
aux_source_directory(Thirdparty/SEGGER/SEGGER SEGGER_SOURCES)

set(ALL_SOURCES 
    ${BNO_SOURCES} 
    ${MAESTRO_SOURCES} 
    ${MS5837_SOURCES} 
    ${PID_SOURCES} 
    ${NAVIGATION_SOURCES}
    ${TASKS_SOURCES}
    ${RTOS_SOURCES}
    ${RTOS_PORT_SOURCES}
    ${RTOS_MEM_SOURCES}
    ${SEGGER_SOURCES}
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${ALL_SOURCES})