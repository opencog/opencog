#The name of the project
PROJECT(DeSTIN_CUDA)

#The CMake Minimum version that is required. The FindCUDA script
#is distributed since version 2.8
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)



#Searching CUDA
FIND_PACKAGE(CUDA 4.0)

if(CUDA_FOUND)
    #Enable debug for CUDA
    set(CUDA_COMPILE_TIME_EXTRA_FLAGS -g -G)
    
	add_subdirectory( pugixml )

	#Make solution
	CUDA_ADD_EXECUTABLE(destinCuda DestinCuda.cu DestinData.cu AdviceData.cu DestinKernel.cu)
	
	TARGET_LINK_LIBRARIES( destinCuda
	    pugixml
		${Boost_FILESYSTEM_LIBRARY}
		${Boost_SYSTEM_LIBRARY}
	)
else()
	MESSAGE("CUDA not found. Not building CUDA DestinPort.")
endif()
