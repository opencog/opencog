### How To Run
- Generate the aiml rules (visit `opencog/nlp/aiml` for details), e.g.
```
import/aiml2psi.pl --dir import/aiml-xml
cp aiml-rules.scm /tmp
```
- Load the chatbot, e.g.
```
(load "chatbot.scm")
```
- Set the relex-server-host accordingly if you are running it using Docker, e.g.
```
(set! relex-server-host "172.17.0.2")
```
- Load the aiml rules, e.g.
```
(primitive-load "/tmp/aiml-rules.scm")
```
- Load the canned-rules for testing, e.g.
```
(primitive-load "psi-canned-rules.scm")
```
