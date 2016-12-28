ADD_GUILE_MODULE (chatbot.scm
	chat-utils.scm
	bot-api.scm
)

INSTALL (FILES
    basic_sentiment_analysis.py
    DESTINATION "${DATADIR}/python/")

INSTALL (FILES
    dicts/positive.yml
    dicts/negative.yml
    dicts/inc.yml
    dicts/dec.yml
    dicts/inv.yml
    dicts/sentiWordNet.txt
    DESTINATION "/usr/local/etc/opencog/dicts")
