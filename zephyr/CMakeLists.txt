if(CONFIG_ZENBEDDED)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../zenbedded_transport
                   ${CMAKE_CURRENT_BINARY_DIR}/zenbedded_transport)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../zenbedded_rcl
                   ${CMAKE_CURRENT_BINARY_DIR}/zenbedded_rcl)
endif()