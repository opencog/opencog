#install header files
INSTALL(FILES
	behave_cscore.h
	boost_cscore.h
	bscores.h
	discriminating_bscore.h
	instance_scorer.h
	precision_bscore.h
	scoring_base.h

	DESTINATION

	"include/${PROJECT_NAME}/learning/moses/scoring"
)
