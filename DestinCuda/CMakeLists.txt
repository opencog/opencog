#The name of the project
PROJECT(DeSTIN_CUDA)

#The CMake Minimum version that is required. The FindCUDA script
#is distributed since version 2.8
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)



#Searching CUDA
FIND_PACKAGE(CUDA)

if(CUDA_FOUND)
	add_subdirectory( pugixml )

	#Make solution
	CUDA_ADD_EXECUTABLE(destinCuda DestinCuda.cu DestinData.cu AdviceData.cu DestinLayer.cu)
	
	TARGET_LINK_LIBRARIES( destinCuda
	    pugixml
	)
else()
	MESSAGE("CUDA not found. Not building CUDA DestinPort.")
endif()
