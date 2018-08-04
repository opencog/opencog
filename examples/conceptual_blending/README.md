# Conceptual Blending Examples
* Examples using the Conceptual Blending API.


## Files
1. blend_simple.py: Simple example to use blend API.

2. blend_with_config.py: Request to blend API with custom config.
  * Atoms that have STI value above 12 will be considered to blend.
  * Force to start blend, and choose 2 nodes randomly.

3. blend_with_conflict_links.py: Simple example describes conflict input case.
  * Force to start blend, and choose 2 nodes have highest STI value.
  * Make 2^k available(viable) new blend atoms if there exists k conflicts,
    * If two similarity links strength value have difference above 0.3 and 
      confidence value both above 0.7, blender thinks they conflict to each 
      other.

4. blend_with_information_interaction.py: Simple 'bat-man' example using 
  Interaction Information theory to make new blend.
  * Choose one link set which has largest interaction information, and connect
    them to new blend.


## See Also
* [Conceptual Blending Config Format]
 (../../../opencog/python/blending/blend-config-format.md)
