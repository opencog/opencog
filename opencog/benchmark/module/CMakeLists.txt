ADD_LIBRARY(benchmark SHARED
        BenchmarkModule
)

TARGET_LINK_LIBRARIES(benchmark
        server
)
