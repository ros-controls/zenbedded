cmake_minimum_required(VERSION 3.20.0)

zephyr_library()
zephyr_include_directories(include)

zephyr_library_sources(src/zenoh_transport.c)
zephyr_library_compile_definitions(Z_CONFIG_SOCKET_TIMEOUT=1)

zephyr_compile_definitions(
  Z_FEATURE_UNSTABLE_API=1
  Z_TRANSPORT_LEASE=60000
)

if(CONFIG_ZENBEDDED_TIER_1)
  zephyr_library_sources(src/serialization.c)
endif()

if(CONFIG_ZENBEDDED_TIER_2)
  set(GEN_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_tier2_payload.py")

  if(DEFINED CONFIG_ZENBEDDED_TIER2_SCHEMA_PATH AND NOT "${CONFIG_ZENBEDDED_TIER2_SCHEMA_PATH}" STREQUAL "")
    set(SCHEMA_FILE "${APPLICATION_SOURCE_DIR}/${CONFIG_ZENBEDDED_TIER2_SCHEMA_PATH}")
  elseif(EXISTS "${APPLICATION_SOURCE_DIR}/config/interface_schema.yaml")
    set(SCHEMA_FILE "${APPLICATION_SOURCE_DIR}/config/interface_schema.yaml")
  elseif(EXISTS "${APPLICATION_SOURCE_DIR}/interface_schema.yaml")
    set(SCHEMA_FILE "${APPLICATION_SOURCE_DIR}/interface_schema.yaml")
  else()
    set(SCHEMA_FILE "")
  endif()

  if(SCHEMA_FILE AND EXISTS "${SCHEMA_FILE}")
    set(GEN_OUTPUT_DIR "${ZEPHYR_BINARY_DIR}/include/zenbedded_transport/generated")
    set(GEN_HEADER "${GEN_OUTPUT_DIR}/interface_data.h")

    add_custom_command(
      OUTPUT "${GEN_HEADER}"
      COMMAND ${CMAKE_COMMAND} -E make_directory "${GEN_OUTPUT_DIR}"
      COMMAND ${PYTHON_EXECUTABLE} "${GEN_SCRIPT}"
              --yaml "${SCHEMA_FILE}"
              --output "${GEN_HEADER}"
      DEPENDS "${SCHEMA_FILE}" "${GEN_SCRIPT}"
      COMMENT "[zenbedded] Generating Tier 2 payload struct from ${SCHEMA_FILE}"
    )

    add_custom_target(zenbedded_tier2_header_gen ALL DEPENDS "${GEN_HEADER}")
    add_dependencies(zephyr_interface zenbedded_tier2_header_gen)
    zephyr_include_directories("${ZEPHYR_BINARY_DIR}/include")
  else()
    message(FATAL_ERROR "[zenbedded] CONFIG_ZENBEDDED_TIER_2=y but schema YAML not found! "
                          "Place 'interface_schema.yaml' in app root/config or set CONFIG_ZENBEDDED_TIER2_SCHEMA_PATH.")
  endif()
endif()
