#!/usr/bin/cmake -P

# first argument - preset name
# second argument - Asterisk version number

IF(NOT DEFINED CMAKE_ARGV3)
    SET(PRESET_NAME "default")
ELSE()
    SET(PRESET_NAME ${CMAKE_ARGV3})
ENDIF()

IF(NOT DEFINED CMAKE_ARGV4)
    SET(ASTERISK_VERSION_NUM 180000) # default value
ELSE()
    SET(ASTERISK_VERSION_NUM ${CMAKE_ARGV4})
ENDIF()

CMAKE_PATH(GET CMAKE_SCRIPT_MODE_FILE PARENT_PATH SCRIPT_DIR)

EXECUTE_PROCESS(
    COMMAND ${CMAKE_COMMAND} -DASTERISK_VERSION_NUM=${ASTERISK_VERSION_NUM} --preset ${PRESET_NAME}
    WORKING_DIRECTORY ${SCRIPT_DIR}
    COMMAND_ERROR_IS_FATAL ANY
)
