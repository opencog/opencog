
IF (HAVE_SERVER)
   ADD_SUBDIRECTORY (dotty)
   ADD_SUBDIRECTORY (tulip)
   IF (HAVE_UBIGRAPH)
      ADD_SUBDIRECTORY (ubigraph)
   ENDIF (HAVE_UBIGRAPH)
ENDIF (HAVE_SERVER)
                                              
