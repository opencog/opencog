#
# Cogita is just a simple, staind-alone binary,
# it translates between IRC protocol and the cog-server
#
ADD_EXECUTABLE(cogita
	CogitaConfig.cc
	IRC.cc
	go-irc.cc
	whirr-sockets.cc
)

LINK_DIRECTORIES (
	${PROJECT_BINARY_DIR}/opencog/util
)

TARGET_LINK_LIBRARIES (cogita
	${COGUTIL_LIBRARY}
)
