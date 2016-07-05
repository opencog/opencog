### How To Run
- Install (if you haven't done so) and start the [RelEx server] (https://github.com/opencog/relex)

- Start a Guile interpreter and load the chatbot, e.g.
```
guile -l "chatbot.scm"
```

- Load AIML rules and sentences into the AtomSpace
```
(load "res/load-all.scm")
```

- Finally use the `chat` function to interact with the chatbot, e.g.
```
(chat "Are you conscious?")
```

### Remarks
- If you have [PostgreSQL] (https://github.com/opencog/atomspace/tree/master/opencog/persist/sql) set up, you can store everything you want (e.g. AIML psi-rules / parsed text etc) in the databse and just load from it, which is a lot faster and more convenient than doing the `primitive-load` and sentences parsing every time you run it.

- Add you own AIML rules into OpenCog (visit [opencog/opencog/nlp/aiml] (https://github.com/opencog/opencog/tree/master/opencog/nlp/aiml) if you want to know more about it)
  - Download AIML rules from [HEAD] (https://github.com/hansonrobotics/HEAD/tree/master/src/chatbot/aiml)
  - Convert them into psi-rules by running the `aiml2psi.pl` script, e.g.
```
../aiml/import/aiml2psi.pl --dir /where/the/aiml/files/are
```

- Prepare more content for doing fuzzy matching (and more). It should be a plain text file containing sentences that the robot can use and say, assuming each sentence is in its own line, and parse the text file into the AtomSpace in Guile by using `parse-all`, e.g.
```
(parse-all nlp-parse "/where/the/text/file/is")
```

## Overview
This chatbot is driven by [OpenPsi] (https://github.com/leungmanhin/opencog/tree/master/opencog/openpsi), so the behavior of it depends on the psi-rules we have defined for it. Right now it is very crude, but more features will be added to make it more sophisticated in the near future.

Currently it generates replies by using one of the following components:
- AIML engine
- Fuzzy matcher
- External sources, like [DuckDuckGo] (https://duckduckgo.com)

And it will pick the reply according to the currently context and the weight of the corresponding psi-rules (i.e. the truth values assign to those OpenPsi rules, in the form of ImplicationLinks, as defined in `psi-rules.scm`), so adjusting and contexts and weights could alter the behavior of the chatbot to some extent.

The code consists of:
- chatbot.scm
  - The main, entry point of the chatbot

- psi-rules.scm
  - All the psi-rules we have defined for it

- contexts.scm
  - All the chat-related contexts available for the psi-rules

- actions.scm
  - All the chat-related actions available for the psi-rules

- duckduckgo.scm
  - Can be considered as another available action for the psi-rules as well, which allows that chatbot to query [DuckDuckGo] (https://duckduckgo.com) by using its [API] (https://duckduckgo.com/api)

- utils.scm
  - Utilities that are useful in general
