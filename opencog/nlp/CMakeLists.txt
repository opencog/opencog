
# Build C++ code in assorted NLP subdirectories

ADD_SUBDIRECTORY (chatbot)

# The code in the question dir is now deprecated, 
# and should not be built
# ADD_SUBDIRECTORY (question)

ADD_SUBDIRECTORY (types)
ADD_SUBDIRECTORY (wsd)

IF (HAVE_LINK_GRAMMAR)
    ADD_SUBDIRECTORY (sureal)
ENDIF (HAVE_LINK_GRAMMAR)
