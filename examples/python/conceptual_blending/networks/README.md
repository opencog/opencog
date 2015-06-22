# Sample Networks
* Provide the sample input network data.

### Paul and Sally test case
* Sample network from the book 'The Way We Think'.
* Purpose is make new blended space 
  * The father paul, The daughter sally.

#### Input
![Image of paul-sally example's input]
(https://github.com/kim135797531/opencog-python-blending/blob/master/examples/python/conceptual/blending/networks/paul_sally_input.png)

#### Output Goal
![Image of paul-sally example's output goal]
(https://github.com/kim135797531/opencog-python-blending/blob/master/examples/python/conceptual/blending/networks/paul_sally_output.png)


### Debate with Kant test case
* Sample network from the book 'The Way We Think'.
* Purpose is make new blended space
  * Two people talking in the same place at the same time.

#### Input
![Image of debate with kant example's input]
(https://github.com/kim135797531/opencog-python-blending/blob/master/examples/python/conceptual/blending/networks/debate_with_kant_input.png)

#### Output Goal

### MIT ConceptNetwork
* Simple ConceptNetwork Loader.
* It needs opencog's [test-dataset](https://github.com/opencog/test-datasets).

## Folders
* paul_sally: Paul and Sally sample network.
* debate_with_kant: Debate with Kant sample network.
* concept_net: ConceptNet sample network.

## Files
* base_network.py: Abstract class to provide 'make()' interface.
* network_loader.py: Make the pre-defined sample network to blend.
* network_util.py: Utils for sample networks loader.
