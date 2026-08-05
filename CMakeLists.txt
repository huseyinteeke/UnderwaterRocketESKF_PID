cmake_minimum_required(VERSION 3.22)
project(SARA_Firmware C CXX)

# VS Code / Clangd için compile_commands.json üretimini aktif et
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# ==================================
# Eğer dışarıdan tam yol verilmediyse, STM32CubeIDE yolundan otomatik bul
if(NOT CMAKE_C_COMPILER)
    set(CMAKE_C_COMPILER "C:/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/tools/bin/arm-none-eabi-gcc.exe")
endif()

if(NOT CMAKE_CXX_COMPILER)
    set(CMAKE_CXX_COMPILER "C:/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/tools/bin/arm-none-eabi-g++.exe")
endif()
# 1. DONANIM VE BUILD KONFİGÜRASYONU
# ==========================================
# Donanım hedefi seçimi (Varsayılan: pertinaks)
set(HW_TARGET "pertinaks" CACHE STRING "Target hardware: pertinaks, pcb, fota")

# Seçilen donanıma göre dizinleri ve Preprocessor makrolarını belirle
if(HW_TARGET STREQUAL "pertinaks")
    set(TARGET_DIR "hardware_targets/pertinanks")
    set(HW_DEFINE "HW_PERTINAKS")
elseif(HW_TARGET STREQUAL "pcb")
    set(TARGET_DIR "hardware_targets/pcb")
    set(HW_DEFINE "HW_PCB")
elseif(HW_TARGET STREQUAL "fota")
    set(TARGET_DIR "hardware_targets/FOTA")
    set(HW_DEFINE "HW_FOTA")
else()
    message(FATAL_ERROR "Geçersiz HW_TARGET seçildi! 'pertinaks', 'pcb' veya 'fota' kullanın.")
endif()

message(STATUS "==========================================")
message(STATUS "Aktif Donanım Hedefi : ${HW_TARGET}")
message(STATUS "Derleme Tipi (Build) : ${CMAKE_BUILD_TYPE}")
message(STATUS "Preprocessor Makro   : ${HW_DEFINE}=1")
message(STATUS "Hedef Klasör       : ${TARGET_DIR}")
message(STATUS "==========================================")

# ==========================================
# 2. İLGİLİ DONANIMIN KENDİ CMAKE DOSYASINI ÇALIŞTIR
# ==========================================
# add_subdirectory çağrıldığı anda içerideki alt CMake dosyası 
# 'underwater_rocket' (firmware) hedefimi oluşturacak ve derleme makrosunu oraya işleyeceğiz.
# ==========================================
# 2. İLGİLİ DONANIMIN KENDİ CMAKE DOSYASINI ÇALIŞTIR
# ==========================================
add_subdirectory(${TARGET_DIR})

if(TARGET underwater_rocket)
    target_compile_definitions(underwater_rocket PRIVATE ${HW_DEFINE}=1)
elseif(TARGET firmware)
    target_compile_definitions(firmware PRIVATE ${HW_DEFINE}=1)
else()
    message(WARNING "Hedef bulunamadı, makro elle eklenemedi!")
endif()

find_program(CMAKE_OBJCOPY objcopy)
find_program(CMAKE_SIZE size)

set(BINARY_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/binary")

if(NOT CMAKE_TOOLCHAIN_FILE AND NOT CMAKE_C_COMPILER_LOADED)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/arm-none-eabi-gcc.toolchain.cmake"
        CACHE STRING "Default toolchain file" FORCE)
endif()