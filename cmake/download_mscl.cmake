function(download_mscl)
  cmake_parse_arguments(
    download_mscl
    ""
    "VERSION"
    ""
    ${ARGN}
  )

  # If the user specified the location for MSCL we will not download the artifact
  set(MSCL_DIR "" CACHE STRING "Path to MSCL that already exists on the system. If not specified, MSCL will be downloaded")
  if(MSCL_DIR STREQUAL "")
    # Allow the MSCL version to be overridden
    set(MSCL_VERSION "${download_mscl_VERSION}" CACHE STRING "Version of MSCL to download and build with")

    # Normally we would just check cmake system processor, but the buildfarm builds arm on an arm64 machine, so we would download the wrong library
    execute_process(
      COMMAND ${CMAKE_COMMAND} -E env /bin/bash -c "${CMAKE_CXX_COMPILER} -dumpmachine"
      OUTPUT_VARIABLE TARGET_ARCHITECTURE
    )

    # Convert the CMake architecture to the format that MSCL uses
    if("${TARGET_ARCHITECTURE}" MATCHES ".*x86_64.*")
      set(MSCL_ARCH "amd64")
    elseif("${TARGET_ARCHITECTURE}" MATCHES ".*aarch64.*")
      set(MSCL_ARCH "arm64")
    elseif("${TARGET_ARCHITECTURE}" MATCHES ".*arm.*")
      set(MSCL_ARCH "armhf")
    else()
      message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    endif()

    # Install MSCL from github
    set(MSCL_GITHUB_ORG "LORD-MicroStrain")
    set(MSCL_GITHUB_REPO "MSCL")
    set(MSCL_GITHUB_TAG "v${MSCL_VERSION}")
    set(MSCL_GITHUB_ARTIFACT "c++-mscl_${MSCL_VERSION}_${MSCL_ARCH}.deb")
    set(MSCL_DEB_ARTIFACT "${PROJECT_BINARY_DIR}/${MSCL_GITHUB_ARTIFACT}")

    # If the artifact already exists, no need to download it again
    if(NOT EXISTS "${MSCL_DEB_ARTIFACT}")
      # Get the release ID for the version we are trying to download
      message(STATUS "Looking for MSCL tag ${MSCL_GITHUB_TAG} on github")
      set(MSCL_LIST_REPO_URL "https://api.github.com/repos/${MSCL_GITHUB_ORG}/${MSCL_GITHUB_REPO}/releases/tags/${MSCL_GITHUB_TAG}")
      execute_process(
        COMMAND ${CMAKE_COMMAND} -E env /bin/bash -c "curl -fsSl '${MSCL_LIST_REPO_URL}' | jq '.assets[] | select(.name==\"${MSCL_GITHUB_ARTIFACT}\") | .url' | sed 's/\"// g'"
        OUTPUT_VARIABLE MSCL_DOWNLOAD_LINK
      )

      # Download the deb file
      message(STATUS "Downloading ${MSCL_GITHUB_ARTIFACT} to ${MSCL_DEB_ARTIFACT}")
      execute_process(
        COMMAND ${CMAKE_COMMAND} -E env /bin/bash -c "curl -fsSLJo ${MSCL_DEB_ARTIFACT} -H 'Accept: application/octet-stream' ${MSCL_DOWNLOAD_LINK}"
      )
    endif()

    # Extract the deb file into the workspace
    get_filename_component(MSCL_DEB_EXTRACTED_DIR_NAME "${MSCL_GITHUB_ARTIFACT}" NAME_WE)
    set(MSCL_DEB_EXTRACTED_DIR "${PROJECT_BINARY_DIR}/${MSCL_DEB_EXTRACTED_DIR_NAME}")
    execute_process(
      COMMAND ${CMAKE_COMMAND} -E env /bin/bash -c "dpkg-deb -x '${MSCL_DEB_ARTIFACT}' '${MSCL_DEB_EXTRACTED_DIR}'"
    )
    set(MSCL_DIR "${MSCL_DEB_EXTRACTED_DIR}/usr/share/c++-mscl")
  else()
    message(STATUS "Skipping download of MSCL and looking in ${MSCL_DIR} instead")
  endif()

  # Set some variables in the parent scope
  set(MSCL_DIR "${MSCL_DIR}" PARENT_SCOPE)

  # Locate MSCL and the Boost install it was distributed with
  find_library(MSCL_LIB_PATH NAMES libmscl.so PATHS "${MSCL_DIR}" DOC "MSCL Library" NO_DEFAULT_PATH)
  find_library(BOOST_LIB_PATH NAMES libboost_chrono.so PATHS "${MSCL_DIR}/Boost/lib" NO_DEFAULT_PATH)
  set(MSCL_INC_PATH "${MSCL_DIR}/source" PARENT_SCOPE)
  set(BOOST_INC_PATH "${MSCL_DIR}/Boost/include" PARENT_SCOPE)
endfunction()