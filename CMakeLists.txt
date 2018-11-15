# Global config ---------------------------------------------------------------
# cmake 3.11 is required in order to use the neat external project syntax
cmake_minimum_required (VERSION 3.11)
project (ca821x-openthread)

# Get required packages -------------------------------------------------------
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Set configuration variables -------------------------------------------------
set(CASCODA_OPENTHREAD_MAKE make CACHE STRING "The 'make' program to be used to build openthread")
set(CASCODA_OPENTHREAD_MAKE_ARGS -j10 CACHE STRING "The arguments to be passed to the 'make' program when building openthread")
set(CASCODA_OPENTHREAD_CONFIGURE_OPTS
       --enable-cli
       --enable-application-coap
       --enable-joiner
       --enable-commissioner
       --enable-tmf-proxy
       --enable-mac-filter
       --enable-cert-log
       --enable-dhcp6-client
       --enable-dhcp6-server
       --enable-border-router
       --with-mac=external
       --enable-ftd
       --enable-mtd
       --enable-cli
       --disable-docs
     CACHE STRING
       "The arguments to be passed to configure when configuring openthread"
)
set(CASCODA_OPENTHREAD_GIT_TAG "origin/ext-mac" CACHE STRING "The tag to clone from the openthread git repo")
set(CASCODA_OPENTHREAD_GIT_REPO "https://github.com/Cascoda/openthread.git" CACHE STRING "The address of the repo to be cloned")

# Config file generation ------------------------------------------------------
if(${CASCODA_CA_VER} == 8210)
	MESSAGE( WARNING "CA-${CASCODA_CA_VER} is not fully supported for thread, please upgrade")
	set(CASCODA_DEVICE_TABLE_SIZE 10)
elseif(${CASCODA_CA_VER} == 8211)
	set(CASCODA_DEVICE_TABLE_SIZE 32)
else()
	MESSAGE( ERROR "CA-${CASCODA_CA_VER} is not supported with this version")
endif()

configure_file(
	"${PROJECT_SOURCE_DIR}/platform/include/ca821x-posix-thread/ca821x-openthread-config.h.in"
	"${PROJECT_BINARY_DIR}/platform/include/ca821x-posix-thread/ca821x-openthread-config.h"
	)

set(OPENTHREAD_INCLUDE_ARG "-DOPENTHREAD_PROJECT_CORE_CONFIG_FILE=\"ca821x-openthread-config.h\" -I${PROJECT_BINARY_DIR}/platform/include/ca821x-posix-thread/"

# Sub-project configuration ---------------------------------------------------
include(FetchContent)
include(ExternalProject)

FetchContent_Declare(
  ca821x-posix
  GIT_REPOSITORY https://github.com/Cascoda/ca821x-posix.git
  GIT_TAG        origin/master
)

FetchContent_GetProperties(ca821x-posix)
if(NOT ca821x-posix_POPULATED)
  FetchContent_Populate(ca821x-posix)
  add_subdirectory(${ca821x-posix_SOURCE_DIR} ${ca821x-posix_BINARY_DIR})
endif()

ExternalProject_Add(
  openthread
  GIT_REPOSITORY ${CASCODA_OPENTHREAD_GIT_REPO}
  GIT_TAG        ${CASCODA_OPENTHREAD_GIT_TAG}
  GIT_SHALLOW    1
  CONFIGURE_COMMAND cd <SOURCE_DIR> && ./bootstrap && cd <SOURCE_DIR>
	    COMMAND <SOURCE_DIR>/configure ${CASCODA_OPENTHREAD_CONFIGURE_OPTS}
	    "CFLAGS=${OPENTHREAD_INCLUDE_ARG}"
	    "CPPFLAGS=${OPENTHREAD_INCLUDE_ARG}"
	    "CXXFLAGS=${OPENTHREAD_INCLUDE_ARG}"
  BUILD_COMMAND ${CASCODA_OPENTHREAD_MAKE} ${CASCODA_OPENTHREAD_MAKE_ARGS}
  INSTALL_COMMAND   ""
  TEST_COMMAND      ""
)

ExternalProject_Get_Property(openthread SOURCE_DIR)
ExternalProject_Get_Property(openthread BINARY_DIR)
set(openthread_SOURCE_DIR ${SOURCE_DIR})
set(openthread_BINARY_DIR ${BINARY_DIR})

add_library(openthread-ftd STATIC IMPORTED)
add_library(openthread-mtd STATIC IMPORTED)
add_library(openthread-cli-ftd STATIC IMPORTED)
add_library(openthread-cli-mtd STATIC IMPORTED)
add_library(openthread-ncp-ftd STATIC IMPORTED)
add_library(openthread-ncp-mtd STATIC IMPORTED)
add_library(openthread-mbedtls STATIC IMPORTED)
set_property(TARGET openthread-ftd
             PROPERTY IMPORTED_LOCATION ${openthread_BINARY_DIR}/src/core/${CMAKE_STATIC_LIBRARY_PREFIX}openthread-ftd${CMAKE_STATIC_LIBRARY_SUFFIX})
set_property(TARGET openthread-mtd
             PROPERTY IMPORTED_LOCATION ${openthread_BINARY_DIR}/src/core/${CMAKE_STATIC_LIBRARY_PREFIX}openthread-mtd${CMAKE_STATIC_LIBRARY_SUFFIX})
set_property(TARGET openthread-cli-ftd
             PROPERTY IMPORTED_LOCATION ${openthread_BINARY_DIR}/src/cli/${CMAKE_STATIC_LIBRARY_PREFIX}openthread-cli-ftd${CMAKE_STATIC_LIBRARY_SUFFIX})
set_property(TARGET openthread-cli-mtd
             PROPERTY IMPORTED_LOCATION ${openthread_BINARY_DIR}/src/cli/${CMAKE_STATIC_LIBRARY_PREFIX}openthread-cli-mtd${CMAKE_STATIC_LIBRARY_SUFFIX})
set_property(TARGET openthread-ncp-ftd
             PROPERTY IMPORTED_LOCATION ${openthread_BINARY_DIR}/src/ncp/${CMAKE_STATIC_LIBRARY_PREFIX}openthread-ncp-ftd${CMAKE_STATIC_LIBRARY_SUFFIX})
set_property(TARGET openthread-ncp-mtd
             PROPERTY IMPORTED_LOCATION ${openthread_BINARY_DIR}/src/ncp/${CMAKE_STATIC_LIBRARY_PREFIX}openthread-ncp-mtd${CMAKE_STATIC_LIBRARY_SUFFIX})
set_property(TARGET openthread-mbedtls
             PROPERTY IMPORTED_LOCATION ${openthread_BINARY_DIR}/third_party/mbedtls/${CMAKE_STATIC_LIBRARY_PREFIX}mbedcrypto${CMAKE_STATIC_LIBRARY_SUFFIX})

target_link_libraries(openthread-ftd INTERFACE openthread-mbedtls)
target_link_libraries(openthread-mtd INTERFACE openthread-mbedtls)
target_link_libraries(openthread-cli-ftd INTERFACE openthread-ftd)
target_link_libraries(openthread-cli-mtd INTERFACE openthread-mtd)
target_link_libraries(openthread-ncp-ftd INTERFACE openthread-ftd)
target_link_libraries(openthread-ncp-mtd INTERFACE openthread-mtd)

# Main library config ---------------------------------------------------------
add_library(ca821x-openthread-posix-plat
	${PROJECT_SOURCE_DIR}/platform/alarm.c
	${PROJECT_SOURCE_DIR}/platform/flash.c
	${PROJECT_SOURCE_DIR}/platform/logging.c
	${PROJECT_SOURCE_DIR}/platform/misc.c
	${PROJECT_SOURCE_DIR}/platform/platform.c
	${PROJECT_SOURCE_DIR}/platform/radio.c
	${PROJECT_SOURCE_DIR}/platform/radio-stubs.c
	${PROJECT_SOURCE_DIR}/platform/random.c
	${PROJECT_SOURCE_DIR}/platform/selfpipe.c
	${PROJECT_SOURCE_DIR}/platform/serial.c
	${PROJECT_SOURCE_DIR}/platform/settings.c
	${PROJECT_SOURCE_DIR}/platform/spi-stubs.c
	)

target_link_libraries(ca821x-openthread-posix-plat ca821x-posix m)

target_link_libraries(openthread-ftd INTERFACE ca821x-openthread-posix-plat)
target_link_libraries(openthread-mtd INTERFACE ca821x-openthread-posix-plat)

target_include_directories( ca821x-openthread-posix-plat
	PRIVATE
		${PROJECT_SOURCE_DIR}/platform
		${openthread_SOURCE_DIR}/src/core
		${openthread_BINARY_DIR}/include
	PUBLIC
		${openthread_SOURCE_DIR}/include
		${PROJECT_SOURCE_DIR}/platform/include
	)

# Test app config -------------------------------------------------------------
add_executable(cliapp
	${PROJECT_SOURCE_DIR}/example/main.c
	)

add_executable(cliapp-mtd
	${PROJECT_SOURCE_DIR}/example/main.c
	)

add_executable(mt-example
	${PROJECT_SOURCE_DIR}/example/mainMultithread.c
	)

target_link_libraries(cliapp openthread-cli-ftd)
target_link_libraries(cliapp-mtd openthread-cli-mtd)
target_link_libraries(mt-example openthread-cli-ftd Threads::Threads)

# Run tests -------------------------------------------------------------------
include(CTest)
# TODO: Add tests
