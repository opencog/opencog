INSTALL (FILES
	opencog.conf
	DESTINATION "${CONFDIR}"
)

INSTALL (FILES
	FindBoost.cmake
	FindCSockets.cmake
	FindCxxtest.cmake
	FindGuile.cmake
	FindGSL.cmake
	# IODBC no longer supported, due to failed UTF-8 support
	#	FindIODBC.cmake
	FindPthreads.cmake
	FindSTLPort.cmake
	FindUnixODBC.cmake
	# Link-grammar not used in current code.
	#	FindLinkGrammar.cmake
	FindLibUbiGraphClient.cmake
	FindLua51.cmake
	FindLuabind.cmake
	FindXercesC.cmake
	OpenCogAtomTypes.cmake
	OpenCogConfig.cmake
	DESTINATION "${DATADIR}/cmake"
)
