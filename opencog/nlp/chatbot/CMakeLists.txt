
ADD_LIBRARY (chatbot SHARED
	IRC.cc
	go-irc.cc
	whirr-sockets.cc
)

INSTALL (TARGETS chatbot
   LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	IRC.h
	whirr-sockets.h
	DESTINATION "include/opencog/nlp/chatbot"
)
