
# The visualizer is stand-alone.
ADD_SUBDIRECTORY (gtk-visualizer)

# None of the other visualizers work if there's no server.
IF (HAVE_SERVER)
	ADD_SUBDIRECTORY (dotty)
	ADD_SUBDIRECTORY (tulip)
ENDIF (HAVE_SERVER)
