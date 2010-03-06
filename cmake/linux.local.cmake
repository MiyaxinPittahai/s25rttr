################################################################################
### $Id: linux.local.cmake 6121 2010-03-06 12:27:48Z FloSoft $
################################################################################

# this one is important
EXECUTE_PROCESS(COMMAND "uname"
	OUTPUT_VARIABLE CMAKE_SYSTEM_NAME OUTPUT_STRIP_TRAILING_WHITESPACE)
EXECUTE_PROCESS(COMMAND "uname" "-m"
	OUTPUT_VARIABLE CMAKE_SYSTEM_PROCESSOR OUTPUT_STRIP_TRAILING_WHITESPACE)

# specify the compiler
SET(CMAKE_C_COMPILER   gcc)
SET(CMAKE_CXX_COMPILER g++)

# set compiler flags for "native"
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mtune=native")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mtune=native")

INCLUDE(cmake/linux.common.cmake)
