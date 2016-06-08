### How To Run
- Generate the aiml rules (visit `opencog/nlp/aiml` for details), e.g.
```
import/aiml2psi.pl --dir /where/the/aiml/files/are
cp aiml-rules.scm /tmp
```
- Load the chatbot, e.g.
```
(load "chatbot.scm")
```
- Load the aiml rules, e.g.
```
(primitive-load "/tmp/aiml-rules.scm")
```
Use the `chat` function to chat with it, e.g.
```
(chat "Are you conscious?")
```
