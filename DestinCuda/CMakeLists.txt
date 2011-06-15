#The name of the project
PROJECT(DeSTIN_CUDA)

#The CMake Minimum version that is required. The FindCUDA script
#is distributed since version 2.8
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)


#Searching CUDA
FIND_PACKAGE(CUDA 4.0)

if(CUDA_FOUND)
    #Enable debug for CUDA and set Tesla build command
    set(CUDA_COMPILE_TIME_EXTRA_FLAGS -g -G -arch=sm_13)
    
    #Add Pugi XML
	add_subdirectory( pugixml )
	
	#Add cuRAND libary of CUDA
	find_library(CURAND_LIBRARY curand "/usr/local/cuda/lib64")
	
	#Make solution
	CUDA_ADD_EXECUTABLE(destinCuda DestinCuda.cu DestinData.cu AdviceData.cu DestinKernel.cu)
	TARGET_LINK_LIBRARIES( destinCuda
	    pugixml
	    ${CUDA_CURAND_LIBRARY}
		${Boost_FILESYSTEM_LIBRARY}
		${Boost_SYSTEM_LIBRARY}
	)
else()
	MESSAGE("CUDA not found. Not building CUDA DestinPort.")
endif()
