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
      "--enable-cli-app=all\
       --enable-application-coap\
       --enable-joiner\
       --enable-commissioner\
       --enable-tmf-proxy\
       --enable-mac-filter\
       --enable-cert-log\
       --enable-dhcp6-client\
       --enable-dhcp6-server\
       --enable-border-router\
       --with-mac=external\
       --enable-ftd\
       --enable-mtd\
       --enable-cli\
       --disable-docs"
     CACHE STRING
       "The arguments to be passed to configure when configuring openthread"
)
set(CASCODA_OPENTHREAD_GIT_TAG "origin/ext-mac" CACHE STRING "The tag to clone from the openthread git repo")
set(CASCODA_OPENTHREAD_GIT_REPO "https://github.com/Cascoda/openthread.git" CACHE STRING "The address of the repo to be cloned")

set(CASCODA_CA821X_POSIX_GIT_TAG "origin/master" CACHE STRING "The tag to clone from the ca821x-posix git repo")
set(CASCODA_CA821X_POSIX_GIT_REPO "https://github.com/Cascoda/ca821x-posix.git" CACHE STRING "The address of the repo to be cloned")

# Config file generation ------------------------------------------------------

# Sub-project configuration ---------------------------------------------------
include(FetchContent)

FetchContent_Declare(
  ca821x-posix
  GIT_REPOSITORY CASCODA_CA821X_POSIX_GIT_REPO
  GIT_TAG        CASCODA_CA821X_POSIX_GIT_TAG
)

FetchContent_GetProperties(ca821x-posix)
if(NOT ca821x-posix_POPULATED)
  FetchContent_Populate(ca821x-posix)
  add_subdirectory(${ca821x-posix_SOURCE_DIR} ${ca821x-posix_BINARY_DIR})
endif()

FetchContent_Declare(
  openthread
  GIT_REPOSITORY CASCODA_OPENTHREAD_GIT_REPO
  GIT_TAG        CASCODA_OPENTHREAD_GIT_TAG
  GIT_SHALLOW    1
  CONFIGURE_COMMAND make distclean -C $(openthread_BINARY_DIR); $(openthread_SOURCE_DIR)/bootstrap
            COMMAND cd $(openthread_BINARY_DIR) && $(openthread_SOURCE_DIR)/configure $(CASCODA_OPENTHREAD_CONFIGURE_OPTS)
  BUILD_COMMAND ${CASCODA_OPENTHREAD_MAKE} ${CASCODA_OPENTHREAD_MAKE_ARGS}
  INSTALL_COMMAND   ""
  TEST_COMMAND      ""
)

FetchContent_GetProperties(openthread)
if(NOT openthread_POPULATED)
  FetchContent_Populate(openthread)
endif()

find_library(OPENTHREAD_FTD_LIB openthread-ftd HINTS $(openthread_BINARY_DIR)/src/core)
find_library(OPENTHREAD_MTD_LIB openthread-mtd HINTS $(openthread_BINARY_DIR)/src/core)
find_library(OPENTHREAD_CLI_FTD_LIB openthread-cli-ftd HINTS $(openthread_BINARY_DIR)/src/cli)
find_library(OPENTHREAD_CLI_MTD_LIB openthread-cli-mtd HINTS $(openthread_BINARY_DIR)/src/cli)
find_library(OPENTHREAD_NCP_FTD_LIB openthread-ncp-ftd HINTS $(openthread_BINARY_DIR)/src/ncp)
find_library(OPENTHREAD_NCP_MTD_LIB openthread-ncp-mtd HINTS $(openthread_BINARY_DIR)/src/ncp)

# Main library config ---------------------------------------------------------
add_library(ca821x-openthread-posix-plat
	${PROJECT_SOURCE_DIR}/platform/alarm.c
	${PROJECT_SOURCE_DIR}/platform/flash.c
	${PROJECT_SOURCE_DIR}/platform/logging.c
	${PROJECT_SOURCE_DIR}/platform/misc.c
	${PROJECT_SOURCE_DIR}/platform/platform.c
	${PROJECT_SOURCE_DIR}/platform/posix-platform.c
	${PROJECT_SOURCE_DIR}/platform/radio.c
	${PROJECT_SOURCE_DIR}/platform/radio-stubs.c
	${PROJECT_SOURCE_DIR}/platform/random.c
	${PROJECT_SOURCE_DIR}/platform/selfpipe.c
	${PROJECT_SOURCE_DIR}/platform/serial.c
	${PROJECT_SOURCE_DIR}/platform/settings.c
	${PROJECT_SOURCE_DIR}/platform/spi-stubs.c
	)

target_link_libraries(ca821x-openthread-posix-plat ca821x-posix m)

target_include_directories( ca821x-openthread-posix-plat
	PRIVATE
		${PROJECT_SOURCE_DIR}/platform
	PUBLIC
		${openthread_SOURCE_DIR}/include
	)

# Test app config -------------------------------------------------------------
add_executable(cliapp
	${PROJECT_SOURCE_DIR}/example/main.c
	)

add_executable(mt_example
	${PROJECT_SOURCE_DIR}/example/mainMultithread.c
	)

target_link_libraries(cliapp ca821x-openthread-posix-plat OPENTHREAD_CLI_FTD_LIB OPENTHREAD_FTD_LIB)
target_link_libraries(mt_example ca821x-openthread-posix-plat OPENTHREAD_FTD_LIB Threads::Threads)

# Run tests -------------------------------------------------------------------
include(CTest)
# TODO: Add tests
