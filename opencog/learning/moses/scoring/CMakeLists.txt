#install header files
INSTALL(FILES
	behave_cscore.h
	boost_ascore.h
	bscores.h
	discriminating_bscore.h
	precision_bscore.h
	scoring_base.h

	DESTINATION

	"include/${PROJECT_NAME}/learning/moses/scoring"
)
