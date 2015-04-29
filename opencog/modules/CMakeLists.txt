

# Build the query cogserver module
ADD_LIBRARY(QueryModule SHARED
   QueryModule.cc
)
TARGET_LINK_LIBRARIES(QueryModule query server)
