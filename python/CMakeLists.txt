cmake_minimum_required(VERSION 3.14)
project(pydxfrw)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Debug symbols for debugging
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_C_FLAGS_DEBUG "-g -O0")

# Find Python and pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG REQUIRED)

# libdxfrw source files
set(LIBDXFRW_SOURCES
    ../src/drw_classes.cpp
    ../src/drw_entities.cpp
    ../src/drw_header.cpp
    ../src/drw_objects.cpp
    ../src/libdwgr.cpp
    ../src/libdxfrw.cpp
    ../src/intern/drw_dbg.cpp
    ../src/intern/drw_textcodec.cpp
    ../src/intern/dwgbuffer.cpp
    ../src/intern/dwgreader.cpp
    ../src/intern/dwgreader15.cpp
    ../src/intern/dwgreader18.cpp
    ../src/intern/dwgreader21.cpp
    ../src/intern/dwgreader24.cpp
    ../src/intern/dwgreader27.cpp
    ../src/intern/dwgreader32.cpp
    ../src/intern/dwgutil.cpp
    ../src/intern/dxfreader.cpp
    ../src/intern/dxfwriter.cpp
    ../src/intern/rscodec.cpp
)

# Create pybind11 module
pybind11_add_module(pydxfrw
    pydxfrw.cpp
    ${LIBDXFRW_SOURCES}
)

# Include directories
target_include_directories(pydxfrw PRIVATE
    ../src
    ../src/intern
)

# Installation
install(TARGETS pydxfrw DESTINATION .)
