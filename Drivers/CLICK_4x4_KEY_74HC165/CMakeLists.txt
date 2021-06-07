CMAKE_MINIMUM_REQUIRED(VERSION 3.8)
PROJECT(74HC165_driver C)

################################################################################
# Source groups
################################################################################
set(Source
    "74HC165.c"
)
source_group("Source" FILES ${Source})

set(ALL_FILES
    ${Source}
)

################################################################################
# Target
################################################################################
add_library(${PROJECT_NAME} STATIC ${ALL_FILES})
target_link_libraries (${PROJECT_NAME} applibs pthread gcc_s c azureiot)

# azsphere_target_hardware_definition(${PROJECT_NAME} TARGET_DIRECTORY "../HardwareDefinitions/avnet_mt3620_sk" TARGET_DEFINITION "azure_sphere_learning_path.json")