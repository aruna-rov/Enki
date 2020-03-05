#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.7)

# specify cross compilers and tools
SET(CMAKE_C_COMPILER_WORKS 1)
SET(CMAKE_C_COMPILER arm-none-eabi-gcc)
SET(CMAKE_CXX_COMPILER_WORKS 1)
SET(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)

SET(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld)

#Uncomment for hardware floating point
SET(FPU_FLAGS "-mfloat-abi=hard -mfpu=fpv4-sp-d16")
add_definitions(-DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1)

#Uncomment for software floating point
#SET(FPU_FLAGS "-mfloat-abi=soft")

SET(COMMON_FLAGS
    "-mcpu=cortex-m3 ${FPU_FLAGS} -mthumb -mthumb-interwork -ffunction-sections -fdata-sections \
    -g -fno-common -fmessage-length=0 -specs=nosys.specs -specs=nano.specs \
    -finline-limit -flto -fshort-enums -Os -fdebug-prefix-map=/=")

SET(CMAKE_CXX_FLAGS_INIT "${COMMON_FLAGS} -std=c++14")
SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections,--print-memory-usage -T ${LINKER_SCRIPT}")

PROJECT(Enki C CXX ASM)
set(CMAKE_CXX_STANDARD 14)

add_definitions(-DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1)
add_definitions(-DUSE_HAL_DRIVER -DSTM32F103xB)

file(GLOB_RECURSE SOURCES
        "startup/*.*"
        "Drivers/*.*"
        "Src/*.*"
        )

include_directories(
        Inc
        Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
        Drivers/STM32F1xx_HAL_Driver/Inc
        Drivers/CMSIS/Device/ST/STM32F1xx/Include
        Drivers/CMSIS/Include
    )

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map")

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")


add_library(FreeRTOS-POSIX STATIC
        lib/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_clock.c
        lib/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_mqueue.c
        lib/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_pthread_barrier.c
        lib/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_pthread.c
        lib/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_pthread_cond.c
        lib/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_pthread_mutex.c
        lib/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_sched.c
        lib/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_semaphore.c
        lib/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_timer.c
        lib/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_unistd.c
        lib/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_utils.c
        )

target_include_directories(FreeRTOS-POSIX PUBLIC
        lib/FreeRTOS-POSIX/include
        lib/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include
        lib/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include/portable
        lib/FreeRTOS-POSIX/include/private
        Inc/FreeRTOS-POSIX)

file(GLOB_RECURSE FREE-RTOS_SOURCES "Middlewares/Third_Party/FreeRTOS/*.*")

add_library(FreeRTOS STATIC
        ${FREE-RTOS_SOURCES}
        )

target_include_directories(FreeRTOS PUBLIC
        Inc
        Middlewares/Third_Party/FreeRTOS/Source/include
        Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
        )

target_link_libraries(FreeRTOS-POSIX FreeRTOS)
add_dependencies(FreeRTOS-POSIX FreeRTOS)

set(aruna_DIR lib/Aruna)
find_package(aruna)

add_dependencies(Enki.elf aruna)
target_link_libraries(Enki.elf aruna)

add_dependencies(Enki.elf FreeRTOS)
target_link_libraries(Enki.elf FreeRTOS)

add_dependencies(aruna FreeRTOS-POSIX)
target_link_libraries(aruna FreeRTOS-POSIX)

if (EXISTS "/usr/local/bin/bloaty")
    if (EXISTS "${PROJECT_BINARY_DIR}/${PROJECT_NAME}.old.elf")
        add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
                COMMAND bloaty ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.elf -- ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.old.elf
                COMMAND cp ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.elf ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.old.elf
                )
    else ()
        add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
                COMMAND cp ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.elf ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.old.elf
                )
    endif ()
endif ()