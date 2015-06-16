# Progress summary of this week
* Beautiful version is available in [my wiki](http://wiki.dong-min.kim/GSoC_2015_-_Conceptual_Blending).

## Now
### In Progress
* Change algorithms to work only within given atoms, not in overall atomspace.
* Detect and improve conflict links in newly blended node.
  * Implement checking for some values in HebbianLink and SimilarityLink when 
   select link
  * Select one by random in 2^k possible things

### To do
**Think**

* Search for good algorithms to use in each step.

**Study**

* Read the research papers.
 * Goguen, *Mathematical Models of Cognitive Space and Time*
 * Markus, et al, *A computational account of conceptual blending in basic 
  mathematics*

**CodingCodingCoding**

* Detect and improve conflict links in newly blended node.
  * Implement evaluating degree of reasonable fitness using 
   information-theoretic method
  * Try simple PLN inference to see if contradictions are unveiled
   
### Pause
* Design about public API of my project, and make its document.
* Try to use Unified Rule Engine, PLN in python.
* Select nodes to blending.
  * Make the waiting queue of ConceptNode
  * Currently queue is not need for blending, because now agent make only one 
   new blended node in each step.
  * It will be useful when optimizing blending agent speed by save candidate 
   cache.
  * (15/06/14) I stopped to making Conceptual Blending with MindAgent design.
   It will be resumed when I finish to making Conceptual Blending API.
* Make the links in newly blended node and correct some attribute value
  * Make correction of value level in each source node.
  * (15/06/14) What does it works for?

## Result
### Done
* Make inheritance-config check algorithm.
* Reorganize project directory structure.

### Fail & Deadline Missed
* (empty)
  
### Cancel
* (empty)
