### How To Run
- Preparation for running AIML in OpenCog (visit [opencog/opencog/nlp/aiml] (https://github.com/opencog/opencog/tree/master/opencog/nlp/aiml) if you want to know more about it)
  - Download AIML rules from [HEAD] (https://github.com/hansonrobotics/HEAD/tree/master/src/chatbot/aiml)
  - Convert them into psi-rules by running the `aiml2psi.pl` script, e.g.
```
../aiml/import/aiml2psi.pl --dir /where/the/aiml/files/are
cp aiml-rules.scm /tmp
```

- Prepare content for doing fuzzy matching (and more). It should be a plain text file containing sentences that the robot can use and say, assuming each sentence is in its own line.

- Install (if you haven't done so) and start the [RelEx server] (https://github.com/opencog/relex)

- [Sign up an account] (http://developer.wolframalpha.com/portal/apisignup.html) and then an AppID if you want the chatbot to send queries to [WolframAlpha] (http://www.wolframalpha.com/)

- Start a Guile interpreter and load the chatbot, e.g.
```
guile -l "main.scm"
```

- Load all of the AIML psi-rules, e.g.
```
(primitive-load "/tmp/aiml-rules.scm")
```

- Parse the text file into the AtomSpace, e.g.
```
(parse-all nlp-parse "/where/the/text/file/is")
```

- Set the WolframAlpha AppID (Without doing this step, the chatbot will not send any query to WolframAlpha, but would still give responses)
```
(set-appid "YOUR-WOLFRAMALPHA-APPID")
```

- Finally use the `chat` function to interact with the chatbot, e.g.
```
(chat "Are you conscious?")
```

### Remarks
- If you have [PostgreSQL] (https://github.com/opencog/atomspace/tree/master/opencog/persist/sql) set up, you can store everything you want (e.g. AIML psi-rules / parsed text etc) in the databse and just load from it, which is a lot faster and more convenient than doing the `primitive-load` and sentences parsing every time you run it.

## Overview
This chatbot is driven by [OpenPsi] (https://github.com/leungmanhin/opencog/tree/master/opencog/openpsi), so the behavior of it depends on the psi-rules we have defined for it. Right now it is very crude, but more features will be added to make it more sophisticated in the near future.

Currently it generates replies by using one of the following components:
- AIML engine
- Fuzzy matcher
- External sources, like [DuckDuckGo] (https://duckduckgo.com) and [WolframAlpha] (http://www.wolframalpha.com/)

And it will pick the reply according to the currently context and the weight of the corresponding psi-rules (i.e. the truth values assign to those OpenPsi rules, in the form of ImplicationLinks, as defined in `psi-rules.scm`), so adjusting and contexts and weights could alter the behavior of the chatbot to some extent.

The code consists of:
- main.scm
  - The main, entry point of the chatbot

- psi-rules.scm
  - All the psi-rules we have defined for it

- contexts.scm
  - All the chat-related contexts available for the psi-rules

- actions.scm
  - All the chat-related actions available for the psi-rules

- external-sources.scm
  - Can be considered as another set of available actions for the psi-rules as well, which allows the chatbot to query [DuckDuckGo] (https://duckduckgo.com) by using its [API] (https://duckduckgo.com/api) and [WolframAlpha] (http://www.wolframalpha.com/)

- utils.scm
  - Utilities that are useful in general
