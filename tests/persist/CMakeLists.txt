
ADD_SUBDIRECTORY (file)

IF (ODBC_FOUND)
   ADD_SUBDIRECTORY (sql)
ENDIF (ODBC_FOUND)

# I can't get this stuff to work, and its obsolete anyway
# IF (HAVE_XML)
#    ADD_SUBDIRECTORY (xml)
# ENDIF (HAVE_XML)

