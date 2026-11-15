if(CONFIG_ZENBEDDED_RCL)
    zephyr_library_named(zenbedded_rcl)
    zephyr_library_sources(
        src/zenbedded_client.cpp
    )
    zephyr_include_directories(include)
endif() # CONFIG_ZENBEDDED_RCL
