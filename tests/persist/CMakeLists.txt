
# File-based saving/loading is obsolete; no one uses it.
# ADD_SUBDIRECTORY (file)

IF (HAVE_ZMQ)
    ADD_SUBDIRECTORY (zmq)
ENDIF (HAVE_ZMQ)

