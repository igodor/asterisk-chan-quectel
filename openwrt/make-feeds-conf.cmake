#
# make-feeds-conf.cmake
#

IF(NOT CMAKE_INSTALL_COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT openwrt)
ENDIF()

IF(EXISTS $ENV{DESTDIR})
    SET(OPENWRT_INSTALL_DIR $ENV{DESTDIR})
    CMAKE_PATH(APPEND OPENWRT_INSTALL_DIR ${CMAKE_INSTALL_PREFIX} ${CMAKE_INSTALL_COMPONENT})
ELSE()
    SET(OPENWRT_INSTALL_DIR ${CMAKE_INSTALL_PREFIX})
    CMAKE_PATH(APPEND OPENWRT_INSTALL_DIR ${CMAKE_INSTALL_COMPONENT})
ENDIF()
CMAKE_PATH(APPEND OPENWRT_INSTALL_DIR feed OUTPUT_VARIABLE FEED_DIR)

FILE(CONFIGURE OUTPUT ${OPENWRT_INSTALL_DIR}/feeds-strskx.conf CONTENT "src-link strskx \@FEED_DIR\@" @ONLY)
