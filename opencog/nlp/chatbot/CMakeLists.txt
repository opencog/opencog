
#
# Cogita is just a simple, staind-alone binary,
# it translates between IRC protocol and the cog-server
ADD_EXECUTABLE(cogita 
	IRC
	go-irc
	whirr-sockets
)

