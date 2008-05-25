
# Enable the use of SQL storage, if either iodbc of unixodbc is found.
# Caution: this can also increase RAM usage significantly!
#
IF (IODBC_FOUND)
	ADD_DEFINITIONS(-DHAVE_SQL_STORAGE)
ENDIF (IODBC_FOUND)

ADD_SUBDIRECTORY (AtomSpace)
ADD_SUBDIRECTORY (CogServer)
ADD_SUBDIRECTORY (dynamics)
ADD_SUBDIRECTORY (examples)
ADD_SUBDIRECTORY (guile)
ADD_SUBDIRECTORY (NMXml)
ADD_SUBDIRECTORY (persist)
ADD_SUBDIRECTORY (query)
ADD_SUBDIRECTORY (nlp)
ADD_SUBDIRECTORY (Util)
