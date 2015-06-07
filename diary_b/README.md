# Progress summary of this week
* Beautiful version is available in my wiki
 (http://wiki.dong-min.kim/GSoC_2015_-_Conceptual_Blending)

## Now
### In Progress
* Decide whether or not to execute blending and prepare
  * Implement selecting ConceptNode which has best value (proper value)
* Check the conflict links in each node and remove.
  * Implement removing conflict links

### To do
**Think**

* Use python or change to C++?
* How to make blending agent to really useful to OpenCog?

**Study**

* Read thesis written by Goguen and another thesis written by Markus, et al.

**CodingCodingCoding**

* Detect and improve conflict links in newly blended node
  * Select one by random in 2^k possible things
  * Implement checking for some values in HebbianLink and SimilarityLink when 
   select link
  * Implement evaluating degree of reasonable fitness using 
  information-theoretic method

### Pause
* Search for good algorithms to use in each step.
* Design about public API of my project, and make its document.
* Select nodes to blending.
  * Make the waiting queue of ConceptNode
  * Currently queue is not need for blending, because now agent make only one 
   new blended node in each step.
  * It will be useful when optimizing blending agent speed by save candidate 
   cache.
* Try to use Unified Rule Engine, PLN in python.
* Change config system to use cogserver's config system.

## Result
### Done
* (empty)

### Fail & Deadline Missed
* (empty)
  
### Cancel
* (empty)
