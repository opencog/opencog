

ADD_LIBRARY(blopex SHARED
	fortran_matrix
	lobpcg
	multivector
	temp_multivector
)

TARGET_LINK_LIBRARIES (blopex
	${LAPACK_LIBRARIES}
	m
)
