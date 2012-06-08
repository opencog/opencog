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
    set(CUDA_NVCC_FLAGS -g -G -gencode arch=compute_30,code=sm_30)
    
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
	set(cudadest_source DestinCuda.cu DestinData.cu AdviceData.cu DestinKernel.cu Transporter.h CurandGeneratorWrapper.h)



 	FIND_LIBRARY(lib_highgui highgui )

	if(lib_highgui)
		set(cv_libs /usr/lib/libcv.so /usr/lib/libcxcore.so ) 
		set(cudadest_source ${cudadest_source} VideoSource)
		ADD_EXECUTABLE(testvideo TestVideo VideoSource)
 		target_link_libraries(testvideo ${lib_highgui} ${cv_libs} )
	else()
		message("OpenCV highgui lib not found, not building video input capabilities or java wrappers.")
		set(lib_highgui "") #set it to nothing so it wont complain about it being set to NOTFOUND
	endif(lib_highgui)	

	CUDA_ADD_LIBRARY(cudadestin SHARED ${cudadest_source} ) 
	CUDA_ADD_EXECUTABLE(destinCuda DestinCuda.cu)
	

	TARGET_LINK_LIBRARIES( destinCuda
	    pugixml
            ${lib_curand}
	    cudadestin
	    ${lib_highgui}
	)

        if(WIN32)
        	link_directories( $ENV{CUDA_LIB_PATH} )
        endif()

	#This section creates java JNI bindings using SWIG 2.0
	FIND_PACKAGE (SWIG 2.0.4)
	if(SWIG_FOUND AND lib_highgui)
		INCLUDE(UseSWIG)

		FIND_PACKAGE(JNI)
		if(JNI_FOUND)
			include_directories( ${JNI_INCLUDE_DIRS} )
			SET_SOURCE_FILES_PROPERTIES(DestinKernel.i PROPERTIES CPLUSPLUS ON)
			SET_SOURCE_FILES_PROPERTIES(DestinKernel.i PROPERTIES SWIG_FLAGS "-package;javadestin"  )
			SET(CMAKE_SWIG_OUTDIR ./JavaDestin/gen_src/javadestin )			
			SWIG_ADD_MODULE(destinjava java  DestinKernel.i )	
			SWIG_LINK_LIBRARIES(destinjava cudadestin pugixml ${lib_curand}  ${lib_highgui} )
		else(JNI_FOUND)
			message("Couldn't find jni. Is a jdk installed? Not building java interfaces.")
		endif(JNI_FOUND)
	else()
		message("Could not find either SWIG > 2.0 or libhighgui. Not building java interfaces.")
	endif()		
	
else()
	MESSAGE("CUDA not found. Not building CUDA DestinPort.")
endif()
