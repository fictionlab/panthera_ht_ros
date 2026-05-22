cmake_minimum_required(VERSION 3.5)
project(serial_cmake)

# Include version information
include(cmake/version.cmake)

# Include installation logic
include(GNUInstallDirs)

# Load package configuration helpers
include(CMakePackageConfigHelpers)

# Source files
set(SERIAL_SRC
    src/serial.cc
    src/impl/unix.cc
    src/impl/list_ports/list_ports_linux.cc
)

# Build library
add_library(${PROJECT_NAME} SHARED ${SERIAL_SRC})

# Header directories
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Link system libraries
target_link_libraries(${PROJECT_NAME}
    PUBLIC rt pthread
)

# Install: header files
install(DIRECTORY include/serial
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install: library files
install(TARGETS ${PROJECT_NAME}
    EXPORT serial_cmakeTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Export Targets
install(EXPORT serial_cmakeTargets
    FILE serial_cmakeTargets.cmake
    NAMESPACE serial::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/serial_cmake
)

# Generate Config file
configure_package_config_file(
    cmake/serial_cmakeConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/serial_cmakeConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/serial_cmake
)

# Install Config
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/serial_cmakeConfig.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/serial_cmake
)
