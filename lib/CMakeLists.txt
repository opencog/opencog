INSTALL (FILES
	opencog.conf
	DESTINATION "${CONFDIR}"
)

INSTALL (FILES
	FindCSockets.cmake
	FindCxxtest.cmake
	FindGuile.cmake
	FindGSL.cmake
	FindIODBC.cmake
	FindPthreads.cmake
	FindSTLPort.cmake
	FindUnixODBC.cmake
	OpenCogConfig.cmake
	DESTINATION "${DATADIR}/cmake"
)
