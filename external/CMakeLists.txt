include(FetchContent)

FetchContent_Declare(
        RobotecGPULidar
        GIT_REPOSITORY git@github.com:RobotecAI/RobotecGPULidar.git
        GIT_TAG 7204bc8dec97c3af47cd35c88809fb2a220ecbd5
        GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(RobotecGPULidar)