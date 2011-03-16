ADD_EXECUTABLE (feature-selection feature-selection.cc)
TARGET_LINK_LIBRARIES(feature-selection
  moses
  comboreduct
  util
  ${Boost_PROGRAM_OPTIONS_LIBRARY} 
)

ADD_EXECUTABLE (eval-features eval-features.cc)
TARGET_LINK_LIBRARIES(eval-features
  comboreduct
  util
  ${Boost_PROGRAM_OPTIONS_LIBRARY} 
)

# install feature-selection
INSTALL(TARGETS feature-selection eval-features RUNTIME DESTINATION bin)
