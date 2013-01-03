
#
# Cogita is just a simple, staind-alone binary,
# it translates between IRC protocol and the cog-server
ADD_EXECUTABLE(cogita 
	CogitaConfig
	IRC
	go-irc
	whirr-sockets
)

LINK_DIRECTORIES (
	${PROJECT_BINARY_DIR}/opencog/util
)

ADD_DEPENDENCIES(cogita utils)

TARGET_LINK_LIBRARIES (cogita
	cogutil
)

