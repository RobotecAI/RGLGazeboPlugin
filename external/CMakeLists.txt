
########################################################################
# RobotecGPULidar
########################################################################

set(RGL_PROJECT_DIR "${CMAKE_CURRENT_BINARY_DIR}/RobotecGPULidar")
set(RGL_LIBRARY_DIR "${RGL_PROJECT_DIR}/lib")
set(RGL_SO_FILENAME "libRobotecGPULidar.so")
set(RGL_SO_PATH "${RGL_LIBRARY_DIR}/${RGL_SO_FILENAME}")
set(RGL_INCLUDE_DIR "${RGL_PROJECT_DIR}/include")
set(RGL_API_HEADERS_DIR "${RGL_INCLUDE_DIR}/rgl/api")

set(RGL_CUSTOM_LIBRARY_PATH "" CACHE FILEPATH
    "Path to the custom RobotecGPULidar library build. An alternative to release downloading.")
set(RGL_CUSTOM_API_HEADER_PATH "" CACHE PATH
    "Path to the include directory with API headers compatible with the custom library build
     provided with `RGL_CUSTOM_LIBRARY_FILEPATH`.
     In most cases it will be path to `include` directory of RobotecGPULidar project")

if((RGL_CUSTOM_LIBRARY_PATH AND NOT RGL_CUSTOM_API_HEADER_PATH) OR
   (NOT RGL_CUSTOM_LIBRARY_PATH AND RGL_CUSTOM_API_HEADER_PATH))
    message(FATAL_ERROR "It is required to set both `RGL_CUSTOM_LIBRARY_PATH` and `RGL_CUSTOM_API_HEADER_PATH`\
                         to finish custom RobotecGPULidar configuration")
endif()

if(RGL_CUSTOM_LIBRARY_PATH AND RGL_CUSTOM_API_HEADER_PATH) # Copy custom RobotecGPULidar from the provided location
    file(MAKE_DIRECTORY ${RGL_LIBRARY_DIR})
    file(COPY ${RGL_CUSTOM_LIBRARY_PATH} DESTINATION ${RGL_LIBRARY_DIR})
    file(COPY ${RGL_CUSTOM_API_HEADER_PATH} DESTINATION ${RGL_PROJECT_DIR})
else() # Download RobotecGPULidar from the release
    set(RGL_TAG "v0.20.0")
    set(RGL_API_CORE_HEADER_PATH "${RGL_API_HEADERS_DIR}/core.h")
    set(RGL_SO_ZIP_FILENAME "RGL-core-linux-x64.zip")
    set(RGL_SO_ZIP_URL "https://github.com/RobotecAI/RobotecGPULidar/releases/download/${RGL_TAG}/${RGL_SO_ZIP_FILENAME}")
    set(RGL_API_CORE_HEADER_URL "https://raw.githubusercontent.com/RobotecAI/RobotecGPULidar/${RGL_TAG}/include/rgl/api/core.h")

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
    if(NOT EXISTS ${RGL_API_CORE_HEADER_PATH})
        file(DOWNLOAD ${RGL_API_CORE_HEADER_URL} ${RGL_API_CORE_HEADER_PATH})
    endif()
endif()

add_library(RobotecGPULidar SHARED IMPORTED GLOBAL)
set_target_properties(RobotecGPULidar PROPERTIES
    IMPORTED_LOCATION ${RGL_SO_PATH}
    INTERFACE_INCLUDE_DIRECTORIES ${RGL_INCLUDE_DIR}
)
