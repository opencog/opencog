Tutorial
===

This tutorial provides a hands-on introduction to the OpenCog framework including interactive tutorials and related background material. It is meant to be read from start to finish since each section introduces a variety of related topics. Each step should actually be executed on your computer as you are reading it.

Getting started
---

##### Getting the dependencies and source code
* Clone OpenCog and install OpenCog dependencies as described in the [README](https://github.com/opencog/opencog/#prerequisites).
* Build the system and run the unit tests.

##### Basic operation
* Spend time getting familiar with [running the CogServer](http://wiki.opencog.org/w/Shell) and [using the Scheme shell](http://wiki.opencog.org/w/Scheme).

##### Setting up the Python development environment
* Instructions for setting up Python OpenCog environment can be found [here](http://wiki.opencog.org/w/Python#Setting_up).
* Download the [PyCharm IDE](http://www.jetbrains.com/pycharm/).
* Open the *~/opencog* folder containing the OpenCog repository.
* Configure the project to recognize the OpenCog libraries, by navigating to *File - Settings - Project Structure*, and then right clicking on both the *opencog/python* and *build/opencog/cython* subfolders and marking them as a *Source Folder*.

Automated Program Learning
---

##### Introduction
* Read about [MOSES](http://wiki.opencog.org/w/MOSES).

##### Example
* Use MOSES to run [moses-example.py](https://github.com/opencog/opencog/blob/master/opencog/python/moses/moses-example.py) experiment from Python in the PyCharm IDE.

##### How Cython is used to expose C++ functionality in Python
* Open [pymoses.pyx](https://github.com/opencog/opencog/blob/master/opencog/cython/opencog/pymoses.pyx) in PyCharm. 
* Briefly observe how Cython is used by taking a quick look at how this Python interface to the C++ MOSES code is implemented.

##### Specifying additional parameters
* In the same *pymoses.pyx* file, observe the documentation and the additional examples that demonstrate how to invoke MOSES with arbitrary configuration parameters or external data files.

Subgraph Matching and Graph Rewriting
---

##### Introduction
* Follow [this presentation](https://docs.google.com/a/singularityu.org/presentation/d/15C2lheJNV6UtrIYcJ2zt5UOui_PhxmMNf2OzwGS5oq8/edit?pli=1#slide=id.p) which describes the pattern matcher.
* While reading the presentation refer to these pages to view more details on the concepts that are presented:
  * [Pattern Matcher](http://wiki.opencog.org/w/Pattern_matcher)
  * [BindLink](http://wiki.opencog.org/w/BindLink)

##### Examples
* Execute each of the examples in the presentation yourself in your CogServer scheme shell.

##### Python interface
* Next, you will learn how to load Scheme files and interact with the Scheme interpreter and the atomspace from Python.

###### Graph visualization example
* Read about [Graph Description](https://github.com/opencog/opencog/tree/master/opencog/python/graph_description).
* Open [example.py](https://github.com/opencog/opencog/blob/master/opencog/python/graph_description/example.py) file in PyCharm.
* Run the script while observing the documentation and contents of the script to understand its behavior.
* Copy the output that is produced.
* Search for GraphViz Fiddle online.
* Paste the output into its interface, and render the graph.
* Observe the hypergraph that is produced, which represents the current atomspace you are working with.
* Note particularly how Scheme files are loaded in this Python example.
* From your command line, run *sudo apt-get install graphviz*.
* Run [example-visualization.py](https://github.com/opencog/opencog/blob/master/opencog/python/graph_description/example-visualization.py) and observe the PNG graphics file that is produced by it.

###### Pattern matching
* Next, open [test_pattern.py](https://github.com/opencog/opencog/blob/master/tests/cython/guile/test_pattern.py) file in PyCharm.
* Observe how the *scheme_eval_h* command is used to invoke the pattern matcher functionality from Python, and how parameters are passed and results are returned. 
* Note that the returned results can then have further operations performed on them in Python.
* Investigate the associated Scheme files that are loaded by this program.
* Invoke this program as a unit test from PyCharm. Note that it has a unique structure (a unit test) rather than a normal Python program. This is an opportunity to think about unit testing, which is covered in the subsequent section.

###### Interesting related material on graph rewriting
* Read [Why Hypergraphs?](http://blog.opencog.org/2013/03/24/why-hypergraphs/) by Linas Vepstas that connects some of the dots between hypergraph representations, programming languages and artificial intelligence.
* Read the beginning of [Graph Rewriting](http://en.wikipedia.org/wiki/Graph_rewriting).
* Make a note of the following articles to refer to later as needed to understand the background of the theory behind the pattern matcher functionality:
  * [Constraint Satisfaction](http://en.wikipedia.org/wiki/Constraint_satisfaction)
  * [Constraint Satisfaction Problem](http://en.wikipedia.org/wiki/Constraint_satisfaction_problem)
  * [Boolean Satisfiability Problem](http://en.wikipedia.org/wiki/Boolean_satisfiability_problem)

Unit testing
---
Note that the project uses unit testing in order to verify that the implemented functionality behaves as intended. Whenever functionality is implemented, it should be accompanied by unit tests that test “typical” cases as well as “edge” cases.

* Note that we are using a continuous integration server that performs automated unit testing on the codebase as described in [Buildbot]( http://wiki.opencog.org/w/Buildbot) article.
* Observe the [web interface](http://buildbot.opencog.org:8010/waterfall) that should be used after your pull requests are merged to verify that your unit tests are passing.
* Try clicking on one of the builds and observe that it links directly to a GitHub commit which can be used to identify what commit broke the build if a unit test is failing.

Python Client API
---
There is a Python client API to allow OpenCog experiments to be written as short Python scripts.

##### Overview
* Take a look at the [REST API](http://opencog.github.io/rest-api-documentation/) documentation.
* Click on the GET operation and inspect the JSON structure of the atom representation and look at the different query parameters that can be utilized in your requests.
* Clone the [python-client](https://github.com/opencog/python-client) repository.
* Follow the instructions there to install the dependencies.
* Read the documentation in the README file describing the API interface allowing complete OpenCog experiments to be scripted in Python.

##### Example experiment script integrating attention allocation and reasoning steps
* Run the provided example in example.py and observe that attention allocation and PLN inference are occurring.
* Inspect the Python script and identify what behavior correlates with what script commands, while referring to the API documentation.

Mining frequent and interesting patterns
---
* Read about [Pattern Miner](http://wiki.opencog.org/w/Pattern_Miner#Introduction).
* Follow the [Pattern Miner Tutorial](http://wiki.opencog.org/w/Pattern_Miner#Tutorial_of_running_a_test_of_Pattern_Miner_in_Opencog).

Further exploration
---
Refer to the [OpenCog wiki](http://wiki.opencog.org/w/The_Open_Cognition_Project) for additional examples and tutorials. It is also recommended to obtain the PDF or hard-copy versions of the two-volume series **Engineering General Intelligence**, which provides a detailed overview of the architecture and example experiments that can be conducted within.

*Pull requests with additions or corrections to this tutorial are welcome. For support, you can e-mail the [OpenCog mailing list](https://groups.google.com/forum/#!forum/opencog).*