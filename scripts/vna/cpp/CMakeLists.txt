cmake_minimum_required(VERSION 3.20)
project(lmx_librevna_tools LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

option(LIBREVNA_BUILD_GUI "Build the original LibreVNA GUI executable" OFF)
option(LIBREVNA_BUILD_CLI "Build the headless LibreVNA CLI" ON)
option(LIBREVNA_BUILD_IPC "Build the LibreVNA IPC server" ON)

set(LIBREVNA_GUI_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/LibreVNA-GUI_Source/LibreVNA-GUI")
set(LIBREVNA_PROTOCOL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/librevna_protocol")

file(GLOB_RECURSE LIBREVNA_GUI_CPP CONFIGURE_DEPENDS "${LIBREVNA_GUI_ROOT}/*.cpp")
file(GLOB_RECURSE LIBREVNA_GUI_UI CONFIGURE_DEPENDS "${LIBREVNA_GUI_ROOT}/*.ui")
file(GLOB_RECURSE LIBREVNA_GUI_QRC CONFIGURE_DEPENDS "${LIBREVNA_GUI_ROOT}/*.qrc")
set(LIBREVNA_MINIZ "${LIBREVNA_GUI_ROOT}/Util/QMicroz/miniz.c")

list(FILTER LIBREVNA_GUI_CPP EXCLUDE REGEX ".*/main\\.cpp$")

find_package(Qt6 REQUIRED COMPONENTS Core Widgets Network Gui Svg OpenGL)

add_library(librevna_core STATIC
    ${LIBREVNA_GUI_CPP}
    ${LIBREVNA_GUI_UI}
    ${LIBREVNA_GUI_QRC}
    ${LIBREVNA_MINIZ}
    ${LIBREVNA_PROTOCOL_ROOT}/Protocol.cpp
)


set(LIBREVNA_UIC_DIRS "${LIBREVNA_GUI_ROOT}")
foreach(ui_file IN LISTS LIBREVNA_GUI_UI)
    get_filename_component(ui_dir "${ui_file}" DIRECTORY)
    list(APPEND LIBREVNA_UIC_DIRS "${ui_dir}")
endforeach()
list(REMOVE_DUPLICATES LIBREVNA_UIC_DIRS)

set_target_properties(librevna_core PROPERTIES
    AUTOUIC_SEARCH_PATHS "${LIBREVNA_UIC_DIRS}"
)

target_include_directories(librevna_core PUBLIC
    ${LIBREVNA_GUI_ROOT}
    ${LIBREVNA_GUI_ROOT}/Tools
    ${LIBREVNA_GUI_ROOT}/Tools/Eigen
    ${LIBREVNA_GUI_ROOT}/Util
    ${LIBREVNA_GUI_ROOT}/Traces
    ${LIBREVNA_GUI_ROOT}/Device
    ${LIBREVNA_GUI_ROOT}/Calibration
    ${LIBREVNA_GUI_ROOT}/CustomWidgets
    ${LIBREVNA_GUI_ROOT}/VNA_embedded/Application/Communication
    ${LIBREVNA_PROTOCOL_ROOT}
)

target_link_libraries(librevna_core PUBLIC
    Qt6::Core
    Qt6::Widgets
    Qt6::Network
    Qt6::Gui
    Qt6::Svg
    Qt6::OpenGL
)

target_compile_definitions(librevna_core PUBLIC QMICROZ_LIBRARY)
target_compile_definitions(librevna_core PUBLIC
    FW_MAJOR=1
    FW_MINOR=6
    FW_PATCH=4
    FW_SUFFIX=\"\"
    GITHASH=\"unknown\"
)

find_path(LIBUSB_INCLUDE_DIR NAMES libusb.h PATH_SUFFIXES libusb-1.0)
find_library(LIBUSB_LIBRARY NAMES libusb-1.0)

if(LIBUSB_INCLUDE_DIR AND LIBUSB_LIBRARY)
    target_include_directories(librevna_core PRIVATE ${LIBUSB_INCLUDE_DIR})
    target_link_libraries(librevna_core PRIVATE ${LIBUSB_LIBRARY})
    target_compile_definitions(librevna_core PUBLIC LIBREVNA_HEADLESS_HAS_REAL_DRIVER=1)
else()
    target_compile_definitions(librevna_core PUBLIC LIBREVNA_HEADLESS_HAS_REAL_DRIVER=0)
endif()

if(LIBREVNA_BUILD_CLI)
    add_executable(librevna-cli
        src/app/librevna_cli_main.cpp
        src/headless/librevna_headless.cpp
    )
    target_include_directories(librevna-cli PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/headless)
    target_link_libraries(librevna-cli PRIVATE librevna_core Qt6::Core Qt6::Widgets)
endif()

if(LIBREVNA_BUILD_IPC)
    add_executable(librevna-ipc
        src/app/librevna_ipc_main.cpp
        src/headless/librevna_headless.cpp
    )
    target_include_directories(librevna-ipc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/headless)
    target_link_libraries(librevna-ipc PRIVATE librevna_core Qt6::Core Qt6::Network Qt6::Widgets)
endif()
