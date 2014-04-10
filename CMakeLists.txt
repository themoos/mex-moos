cmake_minimum_required(VERSION 2.8)
project(mex-moos)
include(ExternalProject)

set(EPN PRIVATE_MOOS)


set(${EPN}_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/${EPN}")
set(${EPN}_INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/${EPN}")
set(${EPN}_BUILD_DIR ${${EPN}_INSTALL_DIR}/build)
#set(CMAKE_PREFIX_PATH ${${EPN}_BUILD_DIR} ${CMAKE_PREFIX_PATH}) 


set(CMakeScripts_MODULE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_MODULE_PATH ${CMakeScripts_MODULE_DIR})


set(${EPN}_CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${${EPN}_INSTALL_DIR};-DENABLE_EXPORT=OFF;-DUSE_LIBSTDCPP=ON)

message("${EPN}_PREFIX= '${${EPN}_PREFIX}'")
message("${EPN}_INSTALL_DIR='${${EPN}_INSTALL_DIR}'")
message("${EPN}_CMAKE_ARGS='${${EPN}_CMAKE_ARGS}'")
message("GIT_EXECUTABLE='${GIT_EXECUTABLE}'")

ExternalProject_Add(${EPN}
  PREFIX ${${EPN}_PREFIX}
  #GIT_REPOSITORY https://github.com/themoos/core-moos.git
  GIT_REPOSITORY https://github.com/themoos/core-moos.git
  GIT_TAG devel
  BINARY_DIR ${${EPN}_BUILD_DIR}
  INSTALL_DIR ${${EPN}_INSTALL_DIR}
  CMAKE_ARGS ${${EPN}_CMAKE_ARGS}
  INSTALL_COMMAND ""
)


find_package(MOOS 10  CONFIG HINTS ${${EPN}_BUILD_DIR} NO_DEFAULT_PATH )
if(NOT MOOS_FOUND)
	message("MOOS NOT FOUND BUT THATS OK - run CMake and try again....")
	return()
endif()

#message("MOOS_INCLUDE_DIRS='${MOOS_INCLUDE_DIRS}'")
#message("MOOS_LIBRARIES='${MOOS_LIBRARIES}'")
#message("EPN_BUILD_DIR = '${${EPN}_BUILD_DIR}'")



find_package( Matlab QUIET )
if(NOT MATLAB_FOUND)
  message(STATUS "Did not find MATLAB, disabling ${MRG_LIBRARY_FULL_NAME} mex "
    "wrappers, set MATLAB_ROOT for MATLAB support.")
  return()
endif()

set(MRG_LIBRARY_FULL_NAME matlabmoos) 
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# Declare the mex target name and associated source file.
set(${MRG_LIBRARY_FULL_NAME}_MEX_TARGET ${MRG_LIBRARY_FULL_NAME}_mex)

#set(${MRG_LIBRARY_FULL_NAME}_MEX_SOURCE
#  ${${MRG_LIBRARY_FULL_NAME}_MEX_TARGET}.cpp)
 
set(${MRG_LIBRARY_FULL_NAME}_MEX_SOURCE
  mex-moos.cpp mexHelpers.cpp )
  
message(STATUS " {MRG_LIBRARY_FULL_NAME}_MEX_TARGET ${${MRG_LIBRARY_FULL_NAME}_MEX_TARGET}")
  
  
  
# Explicitly set the output location for the mex file to be build/lib, and set
# the extension to match the mex extension for the current platform.
set(${MRG_LIBRARY_FULL_NAME}_MEX_FILE
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${${MRG_LIBRARY_FULL_NAME}_MEX_TARGET}.${MATLAB_MEX_EXTENSION})

message(STATUS " {MRG_LIBRARY_FULL_NAME}_MEX_FILE ${${MRG_LIBRARY_FULL_NAME}_MEX_FILE}")


# Add custom command to build the mex file.
#
# Build the -I<DIR> include arguments for mex.  We do _not_ include the MRG
# library dependency includes here, as they are explicitly not in the public
# API and thus not required by mex.
set(MEX_API_PUBLIC_INCLUDE_DIRS
  ${MOOS_INCLUDE_DIRS}
  ${${MRG_LIBRARY_FULL_NAME}_EXTERNAL_DEPEND_INCLUDE_DIRS}
  ${${MRG_LIBRARY_FULL_NAME}_SELF_INCLUDE_DIRS})
list(REMOVE_DUPLICATES MEX_API_PUBLIC_INCLUDE_DIRS)
foreach (INC_DIR ${MEX_API_PUBLIC_INCLUDE_DIRS})
  list(APPEND MEX_BUILD_INCLUDE_DIRS_CMDS "-I${INC_DIR}")
endforeach()

# We only need to _explicitly_ link against libraries that are used _directly_
# in the MEX file.  As such we do not need to explicitly link against glog or
# Ceres as they come in through the linking of -l${MRG_LIBRARY_FULL_NAME}.
#
# Note that we explicitly disable ensures in Sophus library when compiling mex
# as the default ensure uses printf which is a macro for mexPrintf in MEX files
# which does not have the same arguments.
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  # On Linux, the mex compiler does not properly use rpath linking (up to
  # at least 2013b), thus although the mex file will link, it will not save
  # full paths to the required libraries, thus otool -L / ldd with show
  # 'libmx -> not found' or similar.  Thus we have to explicitly set the
  # -rpath list to include BOTH the MATLAB library directory, AND the directory
  # containing our libraries (passed to -L as well).  We have to do this for
  # MATLAB as well as MATLAB **INCORRECTLY** sets the rpath (WTF?).
  #
  # Note that due to an incomprehensible reason, -rpath fails if you have
  # spaces between it and the previous / next argument, see [1].
  #
  # [1]: http://www-01.ibm.com/support/docview.wss?uid=swg21329145
  set(FORCE_AND_FIX_MEX_TO_USE_RPATH_LINKING_ARGS
    "LDFLAGS='$$LDFLAGS,-rpath=${MATLAB_MEX_LIBRARIES_DIR},-rpath=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}'")
endif()
# IMPORTANT: DO **NOT** add VERBATIM to the end of this add_custom_command(),
#            it screws up the odd MEX_LD_FLAGS escaping (adds wrapping "" and
#            does crazy things with the $LDFLAGS statement).
add_custom_command( OUTPUT ${${MRG_LIBRARY_FULL_NAME}_MEX_FILE}
  COMMAND ${MATLAB_MEX_EXECUTABLE}
  ARGS -DSOPHUS_DISABLE_ENSURES
  ${MEX_BUILD_INCLUDE_DIRS_CMDS}
  ${FORCE_AND_FIX_MEX_TO_USE_RPATH_LINKING_ARGS}
  #-L${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
  -L${${EPN}_BUILD_DIR}/lib
  -l${MOOS_LIBRARIES}
  -output ${${MRG_LIBRARY_FULL_NAME}_MEX_FILE}
  ${${MRG_LIBRARY_FULL_NAME}_MEX_SOURCE}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  DEPENDS ${${MRG_LIBRARY_FULL_NAME}_MEX_SOURCE}
  COMMENT "Building MEX interface for ${MRG_LIBRARY_FULL_NAME}.")

# Add custom target which is always built, that depends on the mex file to drive
# it's creation.
add_custom_target( ${${MRG_LIBRARY_FULL_NAME}_MEX_TARGET}
  ALL DEPENDS ${${MRG_LIBRARY_FULL_NAME}_MEX_FILE}
  SOURCES ${${MRG_LIBRARY_FULL_NAME}_MEX_SOURCE}
  VERBATIM )

message(STATUS "==> MRG: IMPORTANT: To use ${MRG_LIBRARY_FULL_NAME} MEX "
  "library, add the following to MATLAB's path:")
message(STATUS "==> MRG:   -- ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "==> MRG:   -- ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")





