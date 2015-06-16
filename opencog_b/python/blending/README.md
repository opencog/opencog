# blending
* **(15/06/16) Below contents are not useful at current state, until this 
 project will have been merged with OpenCog Framework.**
 
## Description
* Implement of Conceptual Blending algorithm in [OpenCog](https://github.com/opencog/opencog) Framework.

## Usage
```python
from blending.blend import ConceptualBlending
ConceptualBlending(atomspace).run(focus_atoms, config_base)
```
* Make new blend with atoms from given atoms list.
* Use the custom configs. See [Conceptual Blending Config Format].
* Options
  * focus_atoms: If it is None, then blender will try to blend with all atoms 
  in atomspace.
  * config_base: If it is None, then blender will try to blend with default
  (hard-coded) configs.

## Folders
* chooser: Atom choosers to use for choose atoms in blender.
* decider: Blending deciders to use for decide whether to blend or not.
* maker: Blend atom makers to use for make new blend atom.
* connector: Link connectors to use for connect links in blender.
* util: Several util files.
  
## Files
* blend.py: Code of Conceptual Blending algorithm.
  
