# 
# The build order used here is loosely in terms of 
# dependencies: the later parts depend on, or may
# someday depend on the earlier parts.
#
ADD_SUBDIRECTORY (util)
IF (HAVE_ATOMSPACE)
    ADD_SUBDIRECTORY (atomspace)
    ADD_SUBDIRECTORY (benchmark)
ENDIF (HAVE_ATOMSPACE)
IF (HAVE_ATTENTION)
    ADD_SUBDIRECTORY (dynamics)
ENDIF (HAVE_ATTENTION)
IF (HAVE_SERVER)
    ADD_SUBDIRECTORY (server)
ENDIF(HAVE_SERVER)

# There is nothing in this directory?
#IF (NOT WIN32)
#ADD_SUBDIRECTORY(modules)
#ENDIF (NOT WIN32)

# Extension language support
IF (HAVE_GUILE)
	ADD_SUBDIRECTORY (guile)
	ADD_SUBDIRECTORY (scm)
	ADD_SUBDIRECTORY (shell)
ENDIF (HAVE_GUILE)

# Python bindings currently don't compile because of clash with
# Node class in python lib.
IF (HAVE_PYTHON_BINDING)
	ADD_SUBDIRECTORY (python)
ENDIF (HAVE_PYTHON_BINDING)

IF (HAVE_PERSIST)
   ADD_SUBDIRECTORY (persist)
ENDIF (HAVE_PERSIST)

# adaptors depends on server
IF (HAVE_SERVER)
   ADD_SUBDIRECTORY (adaptors)
ENDIF(HAVE_SERVER)

ADD_SUBDIRECTORY (query)
ADD_SUBDIRECTORY (nlp)
ADD_SUBDIRECTORY (reasoning)
ADD_SUBDIRECTORY (comboreduct)
ADD_SUBDIRECTORY (spatial)
ADD_SUBDIRECTORY (web)

# Joel: I have no idea why this was relying on the expat library...
#IF (EXPAT_FOUND)
ADD_SUBDIRECTORY (learning)
#ENDIF (EXPAT_FOUND)

IF (XERCESC_VERSION_OK AND LUABIND_FOUND AND EXPAT_FOUND)
	ADD_SUBDIRECTORY (embodiment)
ENDIF (XERCESC_VERSION_OK AND LUABIND_FOUND AND EXPAT_FOUND)

# Graphics and visualization subsystems
ADD_SUBDIRECTORY (dotty)
IF (CURL_FOUND AND XMLRPC_FOUND)
    ADD_SUBDIRECTORY (ubigraph)
ENDIF (CURL_FOUND AND XMLRPC_FOUND)

