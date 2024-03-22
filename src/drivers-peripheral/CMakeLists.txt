add_library(mcp4728-i2c-driver-intf
    INTERFACE
    )

target_include_directories(mcp4728-i2c-driver-intf
    INTERFACE
    .
    )

target_link_libraries(mcp4728-i2c-driver-intf
    INTERFACE
    hal-intf
    )

add_library(mcp4728-i2c-driver
    OBJECT
    MCP4728-i2c-driver.cpp
    )

target_link_libraries(mcp4728-i2c-driver
    PUBLIC
    mcp4728-i2c-driver-intf
    PRIVATE
    core-conf
    hal-intf
    )