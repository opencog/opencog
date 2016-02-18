IF(GHC_FOUND)

SET(GHC_VERSION "7.10.1")
SET(GHC_LIB_DIR "/opt/ghc/${GHC_VERSION}/lib/ghc-${GHC_VERSION}/" PARENT_SCOPE)

SET(BUILD_MODULE_INIT ${GHC_EXECUTABLE} -fPIC -c ${CMAKE_CURRENT_SOURCE_DIR}/moduleinit.c
                                        -o ${CMAKE_CURRENT_BINARY_DIR}/moduleinit.o 
                                        -i${CMAKE_CURRENT_SOURCE_DIR})
                      
SET(BUILD_LIB_CMD ${GHC_EXECUTABLE} -dynamic -shared -fPIC -i${CMAKE_CURRENT_SOURCE_DIR} 
                                    -i${CMAKE_CURRENT_BINARY_DIR}  ${CMAKE_CURRENT_SOURCE_DIR}/TVToolBoxCInterface.hs ${CMAKE_CURRENT_BINARY_DIR}/moduleinit.o
                                    -o libjsd.so -odir ${CMAKE_CURRENT_BINARY_DIR} -hidir ${CMAKE_CURRENT_BINARY_DIR} -optl-Wl,-rpath,${GHC_LIB_DIR} -lHSrts-ghc${GHC_VERSION})

ADD_CUSTOM_TARGET(jsdistancehaskellinterface
                  COMMAND ${BUILD_MODULE_INIT}
                  COMMAND ${BUILD_LIB_CMD}                   
                  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}                 
                  )

#Not sure regarding the portability of mv command.
ADD_CUSTOM_TARGET(mvstubheader
                  COMMAND mv ${CMAKE_CURRENT_SOURCE_DIR}/TVToolBoxCInterface_stub.h ${CMAKE_CURRENT_BINARY_DIR}
                  DEPENDS jsdistancehaskellinterface
                  )
                   
ENDIF(GHC_FOUND)

#TODO:
#      - set global accessible variables for GHC_VERSION, GHC_LIB_DIR and GHC_INCLUDE_DIR
#      - Make sure mv is portable.