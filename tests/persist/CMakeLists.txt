ADD_SUBDIRECTORY (file)

IF (HAVE_PERSIST)
   ADD_SUBDIRECTORY (sql)
ENDIF (HAVE_PERSIST)

# I can't get this stuff to work, and its obsolete anyway
# IF (HAVE_XML)
#    ADD_SUBDIRECTORY (xml)
# ENDIF (HAVE_XML)

IF (HAVE_ZMQ)
    ADD_SUBDIRECTORY (zmq)
ENDIF (HAVE_ZMQ)

