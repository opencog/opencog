# blender_b
* Directory to save the available blenders.

## Folders
* chooser: Atom choosers to use for choose atoms in blender.
* decider: Blending deciders to use for decide whether to blend or not.
* maker: Blend atom makers to use for make new blend atom.
* connector: Link connectors to use for connect links in blender.

## Files
* base_blender.py: Abstract class to provide 'blend()' interface.
* no_rule_blender.py: Blend target atoms if targets were given by other. 

* blender_finder.py: Provide blender instance for user.
