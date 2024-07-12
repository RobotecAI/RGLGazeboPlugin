
########################################################################
# RobotecGPULidar
########################################################################

# Use this variable to make sure the downloaded RGL will be up-to-date
set(RGL_FORCE_DOWNLOAD OFF CACHE BOOL
    "Removes existing RGL binaries and downloads a new one")

set(RGL_TAG "v0.17.0")

set(RGL_PROJECT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/RobotecGPULidar")
set(RGL_LIBRARY_DIR "${RGL_PROJECT_DIR}/lib")
set(RGL_SO_FILENAME "libRobotecGPULidar.so")
set(RGL_SO_PATH "${RGL_LIBRARY_DIR}/${RGL_SO_FILENAME}")
set(RGL_INCLUDE_DIR "${RGL_PROJECT_DIR}/include")
set(RGL_API_HEADER_PATH "${RGL_INCLUDE_DIR}/rgl/api/core.h")

set(RGL_SO_ZIP_FILENAME "RGL-core-linux-x64.zip")
set(RGL_SO_ZIP_URL "https://github.com/RobotecAI/RobotecGPULidar/releases/download/${RGL_TAG}/${RGL_SO_ZIP_FILENAME}")
set(RGL_API_HEADER_URL "https://raw.githubusercontent.com/RobotecAI/RobotecGPULidar/${RGL_TAG}/include/rgl/api/core.h")

if (RGL_FORCE_DOWNLOAD)
    file(REMOVE_RECURSE ${RGL_PROJECT_DIR})
endif()

# Download RGL library if not present
if(NOT EXISTS ${RGL_SO_PATH})
    set(RGL_SO_ZIP_PATH "${RGL_LIBRARY_DIR}/${RGL_SO_ZIP_FILENAME}")
    file(DOWNLOAD ${RGL_SO_ZIP_URL} ${RGL_SO_ZIP_PATH})
    file(ARCHIVE_EXTRACT INPUT ${RGL_SO_ZIP_PATH}
        DESTINATION ${RGL_LIBRARY_DIR}
        PATTERNS ${RGL_SO_FILENAME}
        VERBOSE
    )
    file(REMOVE ${RGL_SO_ZIP_PATH})
endif()

# Download RGL API header if not present
if(NOT EXISTS ${RGL_API_HEADER_PATH})
    file(DOWNLOAD ${RGL_API_HEADER_URL} ${RGL_API_HEADER_PATH})
endif()

add_library(RobotecGPULidar SHARED IMPORTED GLOBAL)
set_target_properties(RobotecGPULidar PROPERTIES
    IMPORTED_LOCATION ${RGL_SO_PATH}
    INTERFACE_INCLUDE_DIRECTORIES ${RGL_INCLUDE_DIR}
)
