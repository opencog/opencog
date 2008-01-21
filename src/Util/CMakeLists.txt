add_library(Util
    Logger
    exceptions
    algorithm
    based_variant
    digraph
    dorepeat
    exception
    foreach
    functional
    hashing
    hash_map
    hash_set
    lazy_random_selector
    logging
    lru_cache
    selection
    slist
    tree
    files
    StringTokenizer
    StringManipulator
    PorterStemmer
    mt19937ar
)

include_directories(
${OPENCOG_SOURCE_DIR}/src/Learning/combo
)

target_link_libraries(Util rt)
