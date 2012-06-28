#The name of the project
PROJECT(DeSTIN_CUDA_Alt)

#The CMake Minimum version that is required. The FindCUDA script
#is distributed since version 2.8
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

#Searching CUDA
FIND_PACKAGE(CUDA 4.0)

if(CUDA_FOUND)
    #Enable debug for CUDA and set Tesla build command
    #Change the 20 for the type of you card: (CUDA computable of 1.3 = 13)
    set(CUDA_NVCC_FLAGS -g -G -gencode arch=compute_30,code=sm_30)
    
    INCLUDE_DIRECTORIES( include )
    CUDA_ADD_EXECUTABLE(destin src/main.cu src/destin.cu src/node.cu )    
    CUDA_ADD_EXECUTABLE(kerneltest src/kerneltest.cu src/destin.cu src/node.cu )    
	
    SET_TARGET_PROPERTIES( destin PROPERTIES RUNTIME_OUTPUT_DIRECTORY bin )
    SET_TARGET_PROPERTIES( kerneltest PROPERTIES RUNTIME_OUTPUT_DIRECTORY bin )
	
else()
	MESSAGE("CUDA not found. Not building CUDA Destin.")
endif()

