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
IF (GUILE_FOUND)
	ADD_SUBDIRECTORY (guile)
	ADD_SUBDIRECTORY (scm)
	ADD_SUBDIRECTORY (shell)
ENDIF (GUILE_FOUND)

# Python bindings currently don't compile because of clash with
# Node class in python lib.
IF (NOT APPLE AND Boost_PYTHON_FOUND AND PYTHONLIBS_FOUND)
	ADD_SUBDIRECTORY (python)
ENDIF (NOT APPLE AND Boost_PYTHON_FOUND AND PYTHONLIBS_FOUND)

ADD_SUBDIRECTORY (persist)
ADD_SUBDIRECTORY (adaptors)
ADD_SUBDIRECTORY (query)
ADD_SUBDIRECTORY (nlp)
ADD_SUBDIRECTORY (reasoning)
ADD_SUBDIRECTORY (comboreduct)
ADD_SUBDIRECTORY (spatial)
ADD_SUBDIRECTORY (web)

IF (EXPAT_FOUND)
	ADD_SUBDIRECTORY (learning)
ENDIF (EXPAT_FOUND)

IF (XERCESC_VERSION_OK AND LUABIND_FOUND AND EXPAT_FOUND)
	ADD_SUBDIRECTORY (embodiment)
ENDIF (XERCESC_VERSION_OK AND LUABIND_FOUND AND EXPAT_FOUND)

# Graphics and visualization subsystems
ADD_SUBDIRECTORY (dotty)
IF (CURL_FOUND AND XMLRPC_FOUND)
    ADD_SUBDIRECTORY (ubigraph)
ENDIF (CURL_FOUND AND XMLRPC_FOUND)

