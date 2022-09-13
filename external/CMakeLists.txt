include(FetchContent)

FetchContent_Declare(
        RobotecGPULidar
        GIT_REPOSITORY git@github.com:RobotecAI/RobotecGPULidar.git
        GIT_TAG 35deb0de1ce502e683edf2cd0a5b383e19c2fc44
)

FetchContent_MakeAvailable(RobotecGPULidar)