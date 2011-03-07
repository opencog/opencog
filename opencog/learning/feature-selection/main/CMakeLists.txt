ADD_EXECUTABLE (feature-selection feature-selection.cc)
TARGET_LINK_LIBRARIES(feature-selection
  moses
  comboreduct
  util
  ${Boost_PROGRAM_OPTIONS_LIBRARY} 
)

# install feature-selection
INSTALL(TARGETS feature-selection RUNTIME DESTINATION bin)
