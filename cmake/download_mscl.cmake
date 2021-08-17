function(download_mscl)
  cmake_parse_arguments(
    download_mscl
    ""
    "VERSION"
    ""
    ${ARGN}
  )
  # Allow the MSCL version to be overridden
  set(MSCL_VERSION "${download_mscl_VERSION}" CACHE STRING "Version of MSCL to download and build with")

  # Convert the CMake architecture to the format that MSCL uses
  if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64")
    set(MSCL_ARCH "amd64")
  elseif("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
    set(MSCL_ARCH "arm64")
  elseif("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "armhf")
    set(MSCL_ARCH "armhf")
  else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  endif()

  # Install MSCL from github
  set(MSCL_GITHUB_ORG "LORD-MicroStrain")
  set(MSCL_GITHUB_REPO "MSCL")
  set(MSCL_GITHUB_TAG "v${MSCL_VERSION}")
  set(MSCL_GITHUB_ARTIFACT "c++-mscl_${MSCL_VERSION}_${MSCL_ARCH}.deb")

  # Get the release ID for the version we are trying to download
  set(MSCL_LIST_REPO_URL "https://api.github.com/repos/${MSCL_GITHUB_ORG}/${MSCL_GITHUB_REPO}/releases/tags/${MSCL_GITHUB_TAG}")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E env /bin/bash -c "curl -fsSl '${MSCL_LIST_REPO_URL}' | jq '.assets[] | select(.name==\"${MSCL_GITHUB_ARTIFACT}\") | .url' | sed 's/\"// g'"
    OUTPUT_VARIABLE MSCL_DOWNLOAD_LINK
  )

  # Download the deb file
  set(MSCL_DEB_ARTIFACT "${PROJECT_BINARY_DIR}/${MSCL_GITHUB_ARTIFACT}")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E env /bin/bash -c "curl -fsSLJo ${MSCL_DEB_ARTIFACT} -H 'Accept: application/octet-stream' ${MSCL_DOWNLOAD_LINK}"
  )

  # Extract the deb file into the workspace
  get_filename_component(MSCL_DEB_EXTRACTED_DIR_NAME "${MSCL_GITHUB_ARTIFACT}" NAME_WE)
  set(MSCL_DEB_EXTRACTED_DIR "${PROJECT_BINARY_DIR}/${MSCL_DEB_EXTRACTED_DIR_NAME}")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E env /bin/bash -c "dpkg-deb -x '${MSCL_DEB_ARTIFACT}' '${MSCL_DEB_EXTRACTED_DIR}'"
  )
  set(MSCL_DIR "${MSCL_DEB_EXTRACTED_DIR}/usr/share/c++-mscl")
  set(MSCL_DIR "${MSCL_DIR}" PARENT_SCOPE)

  # Locate MSCL and the Boost install it was distributed with
  find_library(MSCL_LIB_PATH NAMES libmscl.so PATHS "${MSCL_DIR}" DOC "MSCL Library" NO_DEFAULT_PATH)
  find_library(BOOST_LIB_PATH NAMES libboost_chrono.so PATHS "${MSCL_DIR}/Boost/lib" NO_DEFAULT_PATH)
  set(MSCL_INC_PATH "${MSCL_DIR}/source" PARENT_SCOPE)
  set(BOOST_INC_PATH "${MSCL_DIR}/Boost/include" PARENT_SCOPE)
endfunction()