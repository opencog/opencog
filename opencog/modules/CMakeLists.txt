

# ----------------------------------------
# Build the query (pattern-matching) cogserver module
ADD_LIBRARY(QueryModule SHARED
   QueryModule.cc
)
TARGET_LINK_LIBRARIES(QueryModule query server)

# ----------------------------------------
# The SQL persistance module

ADD_LIBRARY (PersistModule SHARED
   PersistModule.cc
)

TARGET_LINK_LIBRARIES(PersistModule
   persist
   persist-sql
)

INSTALL (TARGETS PersistModule
   LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)
