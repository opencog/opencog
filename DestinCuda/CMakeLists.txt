#The name of the project
PROJECT(DeSTIN_CUDA)

#The CMake Minimum version that is required. The FindCUDA script
#is distributed since version 2.8
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)


#Searching CUDA
FIND_PACKAGE(CUDA 4.0)

if(CUDA_FOUND)
    #Enable debug for CUDA and set Tesla build command
    #Change the 20 for the type of you card: (CUDA computable of 1.3 = 13)
    set(CUDA_NVCC_FLAGS -g -G -gencode arch=compute_20,code=sm_20)
    
    #Add Pugi XML
	add_subdirectory( pugixml )
        
	#Add cuRAND libary of CUDA
        if(WIN32)
	        #find_library_local_first is nifty macro found in cmake's FindCUDA script 
	        find_library_local_first(lib_curand curand "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v4.0/lib/win32")	
	else()
       		 FIND_LIBRARY(lib_curand curand "/usr/local/cuda/lib64" )
	endif()

	#Make solution
	CUDA_ADD_EXECUTABLE(destinCuda DestinCuda.cu DestinData.cu AdviceData.cu DestinKernel.cu)
	TARGET_LINK_LIBRARIES( destinCuda
	    pugixml
        ${lib_curand}
	)
        if(WIN32)
        link_directories( $ENV{CUDA_LIB_PATH} )
        endif()
else()
	MESSAGE("CUDA not found. Not building CUDA DestinPort.")
endif()
