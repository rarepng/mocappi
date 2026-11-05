# Define the minimum CMAKE version required:
cmake_minimum_required(VERSION 3.0...3.20 FATAL_ERROR)

if (DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
            CACHE STRING "")
endif ()
# set vcpkg target triplet to silence some warnings
if (DEFINED ENV{VCPKG_DEFAULT_TRIPLET} AND NOT DEFINED VCPKG_TARGET_TRIPLET)
    set(VCPKG_TARGET_TRIPLET "$ENV{VCPKG_DEFAULT_TRIPLET}" CACHE STRING "")
endif ()

# Define the project:
project(MocapFramework C CXX)

# CCache:
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Using ccache: ${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()

# Set C++ stardard:
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# cmake does this automatically
if(MSVC)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++20")
endif(MSVC)

# Enable automatic include of generated files (like paths.h):
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# CUDA Lambda device functions:
# set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --extended-lambda")

# Include IMGUI (which already includes GLFW):
add_subdirectory(lib/imgui-1.88/)

# Include GLAD for OpenGL 3.3 Core functionalities:
add_subdirectory(lib/glad)

find_package(OpenMP)

# Assimp:
find_package(assimp CONFIG REQUIRED)

# Ceres solver:
find_package(Ceres REQUIRED)

# PCL:
find_package(PCL REQUIRED)

# Json:
find_package(nlohmann_json 3.2.0 REQUIRED)

# LibSSH:
find_package(libssh CONFIG REQUIRED)

# to compile the helper tool conveniently
option(MOCAP_BUILD_TOOLS "Build helper tools under tools/" ON)
if(MOCAP_BUILD_TOOLS)
    add_subdirectory(utils/DummyMarkerSender) # can add more tools later
endif()

# Define all header files we want to compile:
set(HEADERS
    # Model:
    src/model/DataModel.h

    src/model/camera/Camera.h
    src/model/camera/CameraCalibration.h
    src/model/camera/CameraDeviceState.h
    src/model/camera/CameraFrame.h
    src/model/camera/CameraBuffer.h

    src/model/primitive/ImageMarker.h
    src/model/primitive/RigidBody.h
    src/model/primitive/RawMarker.h
    src/model/primitive/Marker.h

    src/model/skeleton/Bone.h
    src/model/skeleton/Skeleton.h
    src/model/skeleton/SkeletalMarker.h
    src/model/skeleton/SkeletalRigidBody.h

    # Processors:
    src/processors/Processor.h
    src/processors/ProcessorCollection.h

    src/processors/calibration/CeresCostFunctions.h
    src/processors/calibration/CalibrationSticks.h

    src/processors/calibration/CameraCalibrator.h
    src/processors/input/CameraDataProvider.h
    src/processors/input/TickProvider.h
    src/processors/marker/MarkerPersistor.h
    src/processors/marker/RawMarkerTriangulator.h
    src/processors/rigid/RigidBodyDetector.h
    src/processors/rigid/GroundTruthEvaluator.h
    src/processors/skeleton/SkeletalFitter.h

    # View:
    src/view/View.h
    src/view/ViewData.h
    src/view/renderer/SkeletonRenderer.h

    src/view/ui/widget/CameraImageUI.h
    src/view/ui/menu/CalibrationPaneUI.h
    src/view/ui/menu/RigidBodyPaneUI.h
    src/view/ui/menu/SkeletonPaneUI.h
    src/view/ui/menu/NetworkPaneUI.h

    # Switch
    src/view/ui/elements/SwitchUI.h

    # Util
    src/util/TemporalList.h
    src/util/Semaphore.h

    # Cam Networking
    src/util/network/SSHCommunicator.h
    src/util/network/RemoteCam.h
    src/util/network/CameraNetworkUtils.h

    # Settings
    src/util/settings/Settings.h

    # Animation
    src/util/animation/Animation.h

    # GL stuff
    src/util/gl/Shader.h
    src/util/gl/Texture2D.h
    src/util/gl/TextureFBO.h
    src/util/gl/StaticMesh.h
    src/util/gl/SkeletalMesh.h
    src/util/gl/GLRenderable.h
    src/util/gl/TextRenderer.h

    # GL Primitives
    src/util/gl/primitive/Triangle.h
    src/util/gl/primitive/Vertex.h
    src/util/gl/primitive/TexCoord.h

    # Skeleton Templates
    src/util/skeleton/SkeletonTemplates.h
    src/util/skeleton/SkeletonTemplates.cpp
    src/util/skeleton/SkeletonLoader.h

    # Math classes
    src/util/math/Vec4.h
    src/util/math/Mat4.h
    src/util/math/Quat.h
    src/util/math/Utils.h

    #
    src/util/GuiUtils.h

    src/util/math/RBPredictor.h

    src/pch.h
)

# Define all source files we want to compile:
set(SOURCES 
    src/main.cpp

    # Model:
    src/model/skeleton/Skeleton.cpp

    # Processors:
    src/processors/ProcessorCollection.cpp
    src/processors/calibration/CameraCalibrator.cpp
    src/processors/input/CameraDataProvider.cpp
    src/processors/marker/MarkerPersistor.cpp
    src/processors/marker/RawMarkerTriangulator.cpp
    src/processors/skeleton/SkeletalFitter.cpp

    # View:
    src/view/View.cpp
    src/view/ViewData.cpp

    # GL stuff
    src/util/gl/Shader.cpp
    src/util/gl/Texture2D.cpp
    src/util/gl/TextureFBO.cpp
    src/util/gl/StaticMesh.cpp
    src/util/gl/SkeletalMesh.cpp
    src/util/gl/TextRenderer.cpp

    # Settings
    src/util/settings/Settings.cpp

    # CameraNetworkUtils
    src/util/network/CameraNetworkUtils.cpp
)

# Define shader & resources which should be listed in IDE:
set(RESOURCES
    shader/unlitColor.frag
    shader/unlitColor.vert
)
# Allow to include files directly in this paths
# (without the need to specify folders):
include_directories(src src/util/math lib/stb_image lib/tiny_obj_loader lib/ukf/include)

# Define executables with source files and resources:
add_executable(MocapFramework ${SOURCES} ${HEADERS} ${RESOURCES} src/pch.cpp)

include(CMakeLocal.txt OPTIONAL)

target_compile_definitions(MocapFramework PRIVATE UKF_SINGLE_PRECISION)

# IMGUI specific compile definition:
#target_compile_definitions(MocapFramework PUBLIC -DCMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_compile_definitions(MocapFramework PUBLIC -DCMAKE_SOURCE_DIR=".")

# Define the libraries to link against:
target_link_libraries(MocapFramework PUBLIC imgui glad)

# CUDA
#target_include_directories(MocapFramework PRIVATE ${CUDAToolkit_INCLUDE_DIRS})
#target_link_libraries(MocapFramework PRIVATE ${CUDA_LIBRARIES})

# OpenCV
target_link_libraries(MocapFramework PRIVATE ${OpenCV_LIBS} )

# OpenMP
target_link_libraries(MocapFramework PUBLIC OpenMP::OpenMP_CXX)

# OpenMP
target_link_libraries(MocapFramework PRIVATE nlohmann_json::nlohmann_json)

# PCL
target_link_libraries(MocapFramework PRIVATE ${PCL_LIBRARIES})
target_include_directories(MocapFramework PRIVATE ${PCL_INCLUDE_DIRS})

# Assimp
target_link_libraries(MocapFramework PRIVATE assimp::assimp)

# SSH
target_link_libraries(MocapFramework PRIVATE ssh)

# Ceres
target_link_libraries(MocapFramework PUBLIC ${CERES_LIBRARIES})
target_include_directories(MocapFramework PUBLIC ${CERES_INCLUDE_DIRS})

option(DEVELOPMENT_MODE "Enable development features" OFF)

if(DEVELOPMENT_MODE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE DEVELOPMENT_MODE)
endif()

# Precompiled Header konfigurieren
target_precompile_headers(MocapFramework PRIVATE src/pch.h)

add_custom_command(TARGET MocapFramework POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/../cam"
        "$<TARGET_FILE_DIR:MocapFramework>/cam"
    COMMENT "Copying cam folder to binary directory..."
)

add_custom_command(TARGET MocapFramework POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/data"
        "$<TARGET_FILE_DIR:MocapFramework>/data"
    COMMENT "Copying cam folder to binary directory..."
)

add_custom_command(TARGET MocapFramework POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/lib/imgui-1.88/misc/fonts"
        "$<TARGET_FILE_DIR:MocapFramework>/lib/imgui-1.88/misc/fonts"
    COMMENT "Copying cam folder to binary directory..."
)

add_custom_command(TARGET MocapFramework POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/shader"
        "$<TARGET_FILE_DIR:MocapFramework>/shader"
    COMMENT "Copying cam folder to binary directory..."
)
