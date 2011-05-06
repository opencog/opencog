#The name of the project
PROJECT(DeSTIN_CUDA)

#The CMake Minimum version that is required. The FindCUDA script
#is distributed since version 2.8
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

add_subdirectory( pugixml )

#Searching CUDA
FIND_PACKAGE(CUDA)

#Make solution
CUDA_ADD_EXECUTABLE(destinCuda DestinCuda.cu DestinData.cu AdviceData.cu)

TARGET_LINK_LIBRARIES( destinCuda
    pugixml
)