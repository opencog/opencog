#install header files
INSTALL(FILES
	scoring.h
    scoring_base.h
    discriminating_bscore.h
    precision_bscore.h
    instance_scorer.h
    bscores.h

	DESTINATION

	"include/${PROJECT_NAME}/learning/moses/scoring"
)
