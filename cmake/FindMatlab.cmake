# Find MATLAB - Necessary for linking _against_ MATLAB from C/C++ or using MEX.
#
# Defines:
#  MATLAB_ROOT:              Path to MATLAB root directory.
#  MATLAB_INCLUDE_DIR:       Include path for mex.h, engine.h
#  MATLAB_LIBRARIES:         All required libraries: libmex, etc.
#  MATLAB_MEX_LIBRARY:       Path to libmex.
#  MATLAB_MX_LIBRARY:        Path to libmx.
#  MATLAB_ENG_LIBRARY:       Path to libeng.
#  MATLAB_MEX_EXECUTABLE:    Path to mex executable.
#  MATLAB_MEXEXT_EXECUTABLE: Path to mexext executable.
#  MATLAB_MEX_EXTENSION:     Mex file extension (e.g. mexmaci64 on OSX).
#  MATLAB_MEX_LIBRARIES_DIR: Directory containing MATLAB specific shared
#                            libraries, required for MEX linking.
#
# Alex Stewart, Oxford Mobile Robotics Group (MRG) 2013

if ( MATLAB_ROOT AND NOT MATLAB_ROOT STREQUAL MATLAB_ROOT_INTERNAL )
  # User has reset matlab root since last call, re-find it.
  set(MATLAB_FOUND FALSE)
endif()

if( NOT MATLAB_FOUND )
  if( NOT MATLAB_ROOT )
    # Expand the symlink 'matlab' to /path/to/matlab (symlink).
    execute_process(
      COMMAND which matlab
      RESULT_VARIABLE LOCATE_MATLAB_RESULT
      OUTPUT_VARIABLE LOCATE_MATLAB_SYMLINK
      ERROR_VARIABLE LOCATE_MATLAB_SYMLINK
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_STRIP_TRAILING_WHITESPACE )

    if( LOCATE_MATLAB_SYMLINK STREQUAL "" )
      message(STATUS "Failed to find matlab with \"which matlab\" command.")
    else()
      # Convert the symlink /path/to/matlab to the full absolute path.
      # In OS X readlink is used as: readlink <filename>
      # but in Linux it requires a -f flag: readlink -f <filename>
      if( CMAKE_SYSTEM_NAME MATCHES "Linux" )
        set(READLINK_ARGS "-f")
      endif()

      execute_process(
        COMMAND readlink ${READLINK_ARGS} ${LOCATE_MATLAB_SYMLINK}
        RESULT_VARIABLE MATLAB_RESULT
        OUTPUT_VARIABLE MATLAB_ROOT
        ERROR_VARIABLE MATLAB_ROOT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_STRIP_TRAILING_WHITESPACE )
    endif()

    if (EXISTS ${MATLAB_ROOT})
      # Strip the trailing /bin/matlab from /abs/path/to/matlab/bin/matlab,
      # note that we call get_filename_component() twice, once to strip each
      # suffix.
      get_filename_component(MATLAB_ROOT ${MATLAB_ROOT} PATH)
      get_filename_component(MATLAB_ROOT ${MATLAB_ROOT} PATH)
      message(STATUS "Found MATLAB at ${MATLAB_ROOT}")
    else()
      # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by FindPackage()
      # use the camelcase library name, not uppercase.
      if (Matlab_FIND_QUIETLY)
        message(STATUS "Failed to find MATLAB by invoking matlab symlink.")
      elseif (Matlab_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find MATLAB by invoking matlab symlink, "
          "have you setup a /usr/local/bin/matlab symlink?  You can fix this "
          "by either setting one up, or specifying MATLAB_ROOT manually in "
          "the CMake GUI.")
      else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("Failed to find MATLAB by invoking matlab symlink, "
          "have you setup a /usr/local/bin/matlab symlink?  You can fix this "
          "by either setting one up, or specifying MATLAB_ROOT manually in "
          "the CMake GUI.")
      endif()
    endif()
  endif()

  # Save output to the (user visible) cache.
  set(MATLAB_ROOT ${MATLAB_ROOT} CACHE PATH
    "Root directory (e.g. /Applications/MATLAB_R2011b.app) of MATLAB version to use")
  # Save private internal copy to catch changes.
  set(MATLAB_ROOT_INTERNAL "${MATLAB_ROOT}")

  # Find matlab libraries.
  find_path( MATLAB_INCLUDE_DIR mex.h
    PATHS ${MATLAB_ROOT}/extern/include
    NO_DEFAULT_PATH )
  find_library(MATLAB_MEX_LIBRARY NAMES mex
    PATHS
    ${MATLAB_ROOT}/bin/maci64
    ${MATLAB_ROOT}/bin/glnx86
    ${MATLAB_ROOT}/bin/glnxa64
    NO_DEFAULT_PATH )
  find_library(MATLAB_MX_LIBRARY NAMES mx
    PATHS
    ${MATLAB_ROOT}/bin/maci64
    ${MATLAB_ROOT}/bin/glnx86
    ${MATLAB_ROOT}/bin/glnxa64
    NO_DEFAULT_PATH )
  find_library(MATLAB_ENG_LIBRARY NAMES eng
    PATHS
    ${MATLAB_ROOT}/bin/maci64
    ${MATLAB_ROOT}/bin/glnx86
    ${MATLAB_ROOT}/bin/glnxa64
    NO_DEFAULT_PATH )
  find_program(MATLAB_MEX_EXECUTABLE NAMES mex
    PATHS
    ${MATLAB_ROOT}/bin/
    NO_DEFAULT_PATH )
  find_program(MATLAB_MEXEXT_EXECUTABLE NAMES mexext
    PATHS
    ${MATLAB_ROOT}/bin/
    NO_DEFAULT_PATH )

  # If we found the matlab mex library, we can look for the matlab mex libraries
  # directory.
  unset(MATLAB_MEX_LIBRARIES_DIR)
  if (MATLAB_MEX_LIBRARY)
    # This is a bit ugly, but the format for the directory name is specific
    # to different OSs.
    if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin") # OS X.
      set(MATLAB_MEX_LIBRARIES_DIR "${MATLAB_ROOT}/bin/maci64")
    elseif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
      set(MATLAB_MEX_LIBRARIES_DIR "${MATLAB_ROOT}/bin/glnxa64")
    else()
      message(FATAL_ERROR "The current OS name is: ${CMAKE_SYSTEM_NAME}, "
        "for which we do not have a handler to determine the correct "
        "location relative to MATLAB_ROOT: ${MATLAB_ROOT} for "
        "MATLAB_MEX_LIBRARIES_DIR.")
    endif()
    # Check that the specified directory really exists.
    if (NOT (EXISTS ${MATLAB_MEX_LIBRARIES_DIR} AND
          IS_DIRECTORY ${MATLAB_MEX_LIBRARIES_DIR}))
      message(FATAL_ERROR "Specified MATLAB_MEX_LIBRARIES_DIR: "
        "${MATLAB_MEX_LIBRARIES_DIR}, determined from system name and "
        "MATLAB_ROOT = ${MATLAB_ROOT}, does not exist, or is not a directory.")
    endif()

    get_filename_component(MEX_EXECUTABLE_DIR ${MATLAB_MEX_EXECUTABLE} PATH)
    get_filename_component(MEXEXT_EXECUTABLE_DIR ${MATLAB_MEXEXT_EXECUTABLE} PATH)

    if (NOT MEX_EXECUTABLE_DIR STREQUAL MEXEXT_EXECUTABLE_DIR)
      # Given that we only use mexext to pull the platform specific extension, it
      # is likely that we can continue even if they were found in different
      # locations (as the OS and thus mex file extension is fixed).
      message(WARNING "The 'mex' and 'mexext' programs have been found in "
        "different locations, MEX_EXECUTABLE_DIR: ${MEX_EXECUTABLE_DIR}, "
        "MEXEXT_EXECUTABLE_DIR: ${MEXEXT_EXECUTABLE_DIR}. It's likely that one "
        "of them is not part of the MATLAB installation, please set MATLAB_ROOT "
        "to desired MATLAB install.")
    endif()
  endif()
endif()

set( MATLAB_LIBRARIES
  ${MATLAB_MEX_LIBRARY}
  ${MATLAB_MX_LIBRARY}
  ${MATLAB_ENG_LIBRARY} )

# Get the mex file extension from mexext
if (MATLAB_MEXEXT_EXECUTABLE)
  execute_process(COMMAND ${MATLAB_MEXEXT_EXECUTABLE}
    OUTPUT_VARIABLE MATLAB_MEX_EXTENSION
    OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( Matlab DEFAULT_MSG
  MATLAB_ROOT
  MATLAB_MEX_EXECUTABLE
  MATLAB_MEXEXT_EXECUTABLE
  MATLAB_MEX_EXTENSION
  MATLAB_MEX_LIBRARIES_DIR
  MATLAB_INCLUDE_DIR
  MATLAB_LIBRARIES )

mark_as_advanced( MATLAB_LIBRARIES
  MATLAB_MEX_LIBRARY
  MATLAB_MX_LIBRARY
  MATLAB_ENG_LIBRARY
  MATLAB_INCLUDE_DIR
  MATLAB_INCLUDE_DIR
  MATLAB_ROOT_INTERNAL
  MATLAB_MEX_EXECUTABLE
  MATLAB_MEXEXT_EXECUTABLE
  MATLAB_MEX_EXTENSION )
