## How To Run
For better user experience, it is recommended to run it with [HEAD] (https://github.com/hansonrobotics/HEAD) and interact with it via the web interface. Please refer to the README file in HEAD for details.

## Overview
This chatbot is driven by [OpenPsi] (https://github.com/leungmanhin/opencog/tree/master/opencog/openpsi), so the behavior of it depends on the psi-rules we have defined for it. Right now it is very crude, but more features will be added to make it more sophisticated in the near future.

Currently it generates replies by using one of the following components:
- AIML engine
- Fuzzy matcher
- External sources, like [DuckDuckGo] (https://duckduckgo.com) and [WolframAlpha] (http://www.wolframalpha.com/)
- PLN

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
  - Can be considered as another set of available actions for the psi-rules as well, which allows the chatbot to query [DuckDuckGo] (https://duckduckgo.com) by using its [API] (https://duckduckgo.com/api), [WolframAlpha] (http://www.wolframalpha.com/) and [OpenWeatherMap] (http://openweathermap.org/)

- utils.scm
  - Utilities that are useful in general

- pln-*.scm
  - For doing reasoning using [PLN] (https://github.com/opencog/opencog/tree/master/opencog/pln)
