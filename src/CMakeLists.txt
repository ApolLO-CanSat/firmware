file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/*.c)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR})

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 1)

target_link_libraries(${PROJECT_NAME}
  pico_stdlib
  hardware_i2c
  FreeRTOS-Kernel-Heap4
)

if (PICO_BOARD STREQUAL "orpheus_pico")
  target_link_libraries(${PROJECT_NAME} pico_bootsel_via_double_reset)
  target_compile_definitions(${PROJECT_NAME} PRIVATE
   	PICO_BOOTSEL_VIA_DOUBLE_RESET_ACTIVITY_LED=23
  	)
endif()

if (PICO_CYW43_SUPPORTED)
  target_link_libraries(${PROJECT_NAME} pico_cyw43_arch_none)
endif()

pico_add_extra_outputs(${PROJECT_NAME})
