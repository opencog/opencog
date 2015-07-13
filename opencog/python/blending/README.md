# Conceptual Blending
## Description
* Implement of [Conceptual Blending](https://en.wikipedia.org/wiki/Conceptual_blending) algorithm in [OpenCog Framework](https://github.com/opencog/opencog).

## Usage
```python
from blending.blend import ConceptualBlending
ConceptualBlending(atomspace).run(focus_atoms, config_base)
```
* Make a new blend with atoms from given atoms list.
* Support the custom configs. See [Conceptual Blending Config Format](blend-config-format.md).
* Arguments
 * focus_atoms: If it is None, then blender will try to blend with all atoms in atomspace.
 * config_base: If it is None, then blender will try to blend with default (hard-coded) configs.

## Examples
* See [Conceptual Blending Examples](../../../examples/python/conceptual_blending)

## Development Progress (For GSoC 2015)
* See [Development Progress](http://wiki.dong-min.kim/GSoC_2015_-_Conceptual_Blending#Progress)

## Files
* blend.py: Conceptual Blending algorithm.

## Folders
* src/chooser: Atom choosers to use for choose atoms in blender.
* src/decider: Blending deciders to use for decide whether to blend or not.
* src/maker: Blend atom makers to use for make new blend atom.
* src/connector: Link connectors to use for connect links in blender.
* util: Several utils.
