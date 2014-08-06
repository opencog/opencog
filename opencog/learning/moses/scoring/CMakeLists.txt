#install header files
INSTALL(FILES
	behave_cscore.h
	bscores.h
	discriminating_bscore.h
	precision_bscore.h
	scoring_base.h
	time_dispersion.h
	ss_bscore.h
	select_bscore.h

	DESTINATION

	"include/${PROJECT_NAME}/learning/moses/scoring"
)
