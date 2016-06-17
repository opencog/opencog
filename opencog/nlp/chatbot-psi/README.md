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

- Start a Guile interpreter and load the chatbot, e.g.
```
guile -l "chatbot.scm"
```

- Load all of the AIML psi-rules, e.g.
```
(primitive-load "/tmp/aiml-rules.scm")
```

- Parse the text file into the AtomSpace, e.g.
```
(parse-all nlp-parse "/where/the/text/file/is")
```

- Finally use the `chat` function to interact with the chatbot, e.g.
```
(chat "Are you conscious?")
```

