ADD_GUILE_MODULE (sentiment.scm)

INSTALL (FILES
	basic_sentiment_analysis.py
	DESTINATION "lib${LIB_DIR_SUFFIX}/python2.7/dist-packages/sentiment")

INSTALL (FILES
	dicts/positive.yml
	dicts/negative.yml
	dicts/inc.yml
	dicts/dec.yml
	dicts/inv.yml
	dicts/sentiWordNet.txt
	DESTINATION "/usr/local/etc/opencog/dicts")
