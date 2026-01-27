cmake_minimum_required(VERSION 3.16)
project(librevna-headless-cli LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(LIBREVNA_HEADLESS_ENABLE_REAL_DRIVER "Build the headless CLI with the real LibreVNA USB host implementation" ON)

set(LIBREVNA_HEADLESS_USE_REAL_DRIVER ${LIBREVNA_HEADLESS_ENABLE_REAL_DRIVER})

if(LIBREVNA_HEADLESS_USE_REAL_DRIVER)
    find_path(LIBUSB_INCLUDE_DIR
        NAMES libusb-1.0/libusb.h libusb.h)
    find_library(LIBUSB_LIBRARY
        NAMES usb-1.0 libusb-1.0 libusb)
    if(NOT LIBUSB_INCLUDE_DIR OR NOT LIBUSB_LIBRARY)
        message(WARNING "libusb-1.0 not found; falling back to stub host core. Set LIBUSB_INCLUDE_DIR and LIBUSB_LIBRARY to enable the real driver.")
        set(LIBREVNA_HEADLESS_USE_REAL_DRIVER OFF)
    endif()
endif()

add_library(librevna_host_core STATIC)

target_include_directories(librevna_host_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_sources(librevna_host_core
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/device_discovery.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/calibration.cpp
)

if(LIBREVNA_HEADLESS_USE_REAL_DRIVER)
    target_sources(librevna_host_core
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src/host_core_real.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/librevna_protocol/Protocol.cpp
    )

    target_include_directories(librevna_host_core
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/librevna_protocol
            ${LIBUSB_INCLUDE_DIR}
    )

    target_link_libraries(librevna_host_core
        PUBLIC
            ${LIBUSB_LIBRARY}
    )

    target_compile_definitions(librevna_host_core PUBLIC LIBREVNA_HEADLESS_HAS_REAL_DRIVER=1)
else()
    target_sources(librevna_host_core
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src/host_core_stub.cpp
    )

    target_compile_definitions(librevna_host_core PUBLIC LIBREVNA_HEADLESS_HAS_REAL_DRIVER=0)
endif()

if(NOT TARGET nlohmann_json::nlohmann_json)
    add_library(nlohmann_json::nlohmann_json INTERFACE IMPORTED)
    set_target_properties(nlohmann_json::nlohmann_json PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
    )
endif()

add_executable(librevna-cli
    src/main.cpp
    src/cli_runner.cpp
    src/cli_options.cpp
    src/json_writer.cpp
    src/sweep_result.cpp
)

target_include_directories(librevna-cli
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

find_package(Threads REQUIRED)

target_link_libraries(librevna-cli
    PRIVATE
        librevna_host_core
        Threads::Threads
)

install(TARGETS librevna-cli RUNTIME DESTINATION bin)

set(QT_IPC_COMPONENTS Core Network)
find_package(Qt6 COMPONENTS ${QT_IPC_COMPONENTS} QUIET)
if(Qt6_FOUND)
    set(QT_IPC_LIBS Qt6::Core Qt6::Network)
elseif(NOT Qt6_FOUND)
    find_package(Qt5 COMPONENTS ${QT_IPC_COMPONENTS} QUIET)
    if(Qt5_FOUND)
        set(QT_IPC_LIBS Qt5::Core Qt5::Network)
    endif()
endif()

if(QT_IPC_LIBS)
    add_executable(librevna-ipc
        src/ipc_main.cpp
        src/json_writer.cpp
        src/sweep_result.cpp
    )

    target_include_directories(librevna-ipc
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    target_link_libraries(librevna-ipc
        PRIVATE
            librevna_host_core
            ${QT_IPC_LIBS}
    )

    install(TARGETS librevna-ipc RUNTIME DESTINATION bin)
else()
    message(STATUS "Qt Core/Network not found; skipping librevna-ipc target")
endif()

set(QT_REQUIRED_COMPONENTS Widgets Charts)
find_package(Qt6 COMPONENTS ${QT_REQUIRED_COMPONENTS} QUIET)

if(Qt6_FOUND)
    set(QT_LIBS Qt6::Widgets Qt6::Charts)
elseif(NOT Qt6_FOUND)
    find_package(Qt5 COMPONENTS ${QT_REQUIRED_COMPONENTS} QUIET)
    if(Qt5_FOUND)
        set(QT_LIBS Qt5::Widgets Qt5::Charts)
    endif()
endif()

if(QT_LIBS)
    set(GUI_SOURCES
        src/gui/main.cpp
        src/gui/ui/theme/ThemeLoader.cpp
        src/gui/ui/widgets/PlotView.cpp
        src/gui/ui/widgets/ColorPopover.cpp
        src/gui/ui/widgets/SeriesCheck.cpp
        src/gui/ui/views/Sidebar.cpp
        src/gui/ui/views/BottomPanel.cpp
        src/gui/ui/views/MainView.cpp
        src/gui/resources.qrc
    )

    add_executable(vna-spoofer-gui ${GUI_SOURCES})

    set_target_properties(vna-spoofer-gui PROPERTIES
        AUTOMOC ON
        AUTOUIC ON
        AUTORCC ON
    )

    if(WIN32)
        set_target_properties(vna-spoofer-gui PROPERTIES WIN32_EXECUTABLE ON)
    endif()

    target_include_directories(vna-spoofer-gui
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src/gui
    )

    target_link_libraries(vna-spoofer-gui
        PRIVATE
            ${QT_LIBS}
            librevna_host_core
    )

    install(TARGETS vna-spoofer-gui RUNTIME DESTINATION bin)
else()
    message(STATUS "Qt not found; skipping vna-spoofer-gui target")
endif()
