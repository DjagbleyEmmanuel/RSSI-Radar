cmake_minimum_required(VERSION 3.19)
project(rssiradar VERSION 1.4.3 LANGUAGES CXX
        DESCRIPTION "Passive WiFi/Bluetooth radiometric motion sensing")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g")

find_package(Qt6 REQUIRED COMPONENTS Widgets Core Gui)
find_package(PkgConfig REQUIRED)
pkg_check_modules(BLUETOOTH REQUIRED IMPORTED_TARGET bluez)

add_executable(rssiradar
    src/main.cpp
    src/Dsp.cpp
    src/Tracker.cpp
    src/LifeSign.cpp
    src/Direction.cpp
    src/Signature.cpp
    src/Anomaly.cpp
    src/Config.cpp
    src/Engine.cpp
    src/sensors/WifiSensor.cpp
    src/sensors/CsiSensor.cpp
    src/sensors/BluetoothSensor.cpp
    src/sensors/System.cpp
    ui/Widgets.cpp
    ui/MainWindow.cpp
)

target_include_directories(rssiradar PRIVATE include src ui)
target_link_libraries(rssiradar PRIVATE
    Qt6::Widgets
    Qt6::Core
    Qt6::Gui
    PkgConfig::BLUETOOTH
)

# Raw sockets and netlink need _GNU_SOURCE.
target_compile_definitions(rssiradar PRIVATE _GNU_SOURCE)

target_compile_options(rssiradar PRIVATE -Wall -Wextra -Wno-unused-parameter)

# ---------------------------------------------------------------- packaging

include(GNUInstallDirs)

install(TARGETS rssiradar RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES README.md
        DESTINATION ${CMAKE_INSTALL_DOCDIR}
        RENAME README.md.gz)
install(FILES packaging/rssiradar.desktop
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/applications)
install(FILES packaging/rssiradar.svg
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/icons/hicolor/scalable/apps)
install(PROGRAMS packaging/postinst packaging/postrm
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/DEBIAN)

set(CPACK_PACKAGE_NAME "rssiradar")
set(CPACK_PACKAGE_VENDOR "RSSI Radar")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
    "Passive WiFi/Bluetooth radiometric motion sensing")
set(CPACK_PACKAGE_HOMEPAGE_URL "")
set(CPACK_PACKAGE_CONTACT "RSSI Radar maintainer")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "RSSI Radar maintainer")
set(CPACK_PACKAGE_HOMEPAGE_URL "")

# Depends: Qt6 runtime, bluez, iw, network-manager.
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_PACKAGE_RECOMMENDS "iw | iw | network-manager")
set(CPACK_DEBIAN_PACKAGE_SECTION "science")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_FILE_NAME
    "rssiradar_${PROJECT_VERSION}_${CMAKE_SYSTEM_PROCESSOR}.deb")
# postinst/postrm are shipped as files rather than as CPack script hooks,
# because they must be byte-identical to what lands in DEBIAN/.
set(CPACK_PACKAGE_EXECUTABLES "rssiradar;RSSI Radar passive radiometric motion sensing")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION
    "RSSI Radar performs passive WiFi and Bluetooth motion sensing using real
raw sockets (AF_PACKET with radiotap, AF_BLUETOOTH HCI). It contains no
simulated-data path: every reported value derives from frames a real radio
received, and unavailable hardware is reported explicitly rather than faked.

Includes an 8-preset operating library, EKF/particle-filter state estimation,
Welch-based envelope-beat Doppler velocity, and adaptive CFAR plus entropy and
chi-square change detection.")

# ------------------------------------------------------------------- CPack

set(CPACK_GENERATOR "DEB;TGZ")
set(CPACK_DEB_COMPONENT_INSTALL OFF)
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE ${CMAKE_SYSTEM_PROCESSOR})
set(CPACK_VERBATIM_VARIABLES ON)