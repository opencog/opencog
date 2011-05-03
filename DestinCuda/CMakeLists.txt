#The name of the project
PROJECT(DeSTIN_CUDA)

#The CMake Minimum version that is required. The FindCUDA script
#is distributed since version 2.8
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

#Searching CUDA
FIND_PACKAGE(CUDA)

CUDA_ADD_EXECUTABLE(destinCuda DestinCuda.cu AdviceData.h AdviceData.cu DestinData.h DestinData.cu)