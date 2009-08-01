INSTALL (FILES
	opencog.conf
	opencog-chatbot.conf
	DESTINATION "${CONFDIR}"
)

# ??? Why do these need to be installed ??? 
# This list is incomplete, and I suspect it's not needed at all ... 
INSTALL (FILES
	FindCSockets.cmake
	FindCxxtest.cmake
	FindGuile.cmake
	FindGSL.cmake
	# IODBC no longer supported, due to failed UTF-8 support
	#	FindIODBC.cmake
	FindPthreads.cmake
	FindSTLPort.cmake
	FindUnixODBC.cmake
	FindLibUbiGraphClient.cmake
	FindLuabind.cmake
	FindXercesC.cmake
	OpenCogAtomTypes.cmake
	OpenCogConfig.cmake
	DESTINATION "${DATADIR}/cmake"
)
