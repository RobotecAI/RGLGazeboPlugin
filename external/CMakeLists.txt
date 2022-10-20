include(FetchContent)

FetchContent_Declare(
        RobotecGPULidar
        GIT_REPOSITORY git@github.com:RobotecAI/RobotecGPULidar.git
        GIT_TAG v0.10.2
)

FetchContent_MakeAvailable(RobotecGPULidar)