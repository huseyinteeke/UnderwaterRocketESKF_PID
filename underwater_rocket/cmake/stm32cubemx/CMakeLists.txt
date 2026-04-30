# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms 
    USE_HAL_DRIVER 
    STM32F407xx
    $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Include
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32f4xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32f4xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32f4xx_hal_timebase_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../startup_stm32f407xx.s
)

# STM32 HAL/LL Drivers
set(STM32_Drivers_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/system_stm32f4xx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
)

# ==============================================================================
# KULLANICI EKLENTİLERİ (Sensörler, Tasklar ve FreeRTOS)
# ==============================================================================

file(GLOB_RECURSE USER_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Tasks/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Algorithm+Sensors/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Thirdparty/SEGGER/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Thirdparty/SEGGER/*.S" # Assembly dosyası için kritik (Büyük S)
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Thirdparty/SEGGER/*.s" # Küçük s ihtimaline karşı
)

# 2. FreeRTOS C Kaynak Dosyaları (Manuel eklendi)
set(FREERTOS_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Thirdparty/FreeRTOS/tasks.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Thirdparty/FreeRTOS/queue.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Thirdparty/FreeRTOS/list.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Thirdparty/FreeRTOS/timers.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Thirdparty/FreeRTOS/event_groups.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Thirdparty/FreeRTOS/portable/GCC/ARM_CM4F/port.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Thirdparty/FreeRTOS/portable/MemMang/heap_4.c"
)

# 3. Kendi Klasörlerimizdeki ve SEGGER'deki Include (.h) Yolları
file(GLOB_RECURSE ALL_USER_HEADERS 
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Tasks/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Algorithm+Sensors/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Thirdparty/SEGGER/*.h" # SEGGER içindeki OS, Config vb. yolları bulur
)

set(USER_INCLUDE_DIRS "") 
foreach(_header_file ${ALL_USER_HEADERS})
    get_filename_component(_dir ${_header_file} DIRECTORY)
    list(APPEND USER_INCLUDE_DIRS ${_dir})
endforeach()

# 4. FreeRTOS Include (.h) Yollarını Listeye Ekliyoruz
list(APPEND USER_INCLUDE_DIRS 
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Thirdparty/FreeRTOS"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Thirdparty/FreeRTOS/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Thirdparty/FreeRTOS/portable/GCC/ARM_CM4F"
)

list(REMOVE_DUPLICATES USER_INCLUDE_DIRS)

# ==============================================================================
# KÜTÜPHANE VE KÖPRÜ OLUŞTURMA
# ==============================================================================

# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)

# Tüm include yolları stm32cubemx köprüsüne ekleniyor
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs} ${USER_INCLUDE_DIRS})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Create STM32_Drivers static library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)

# ==============================================================================
# ANA PROJEYE (underwater_rocket) KAYNAKLARI AKTARMA
# ==============================================================================

# Tüm kaynak dosyalarını kök dosyadaki projeye ekliyoruz
target_sources(${CMAKE_PROJECT_NAME} PRIVATE 
    ${MX_Application_Src} 
    ${USER_SOURCES} 
    ${FREERTOS_SOURCES}
)

# Kütüphaneleri ve stm32cubemx arayüzünü ana projeye bağlıyoruz
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE 
    STM32_Drivers
    stm32cubemx
)