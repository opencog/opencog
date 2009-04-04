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
	FindIODBC.cmake
	FindPthreads.cmake
	FindSTLPort.cmake
	FindUnixODBC.cmake
	FindLinkGrammar.cmake
	FindLibUbiGraphClient.cmake
	OpenCogAtomTypes.cmake
	OpenCogConfig.cmake
	DESTINATION "${DATADIR}/cmake"
)
