ADD_LIBRARY(benchmark SHARED
	BenchmarkModule
)

ADD_DEPENDENCIES(benchmark opencog_atom_types)
ADD_DEPENDENCIES(benchmark attention_atom_types)

TARGET_LINK_LIBRARIES(benchmark
	server
)
