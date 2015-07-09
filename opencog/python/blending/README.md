# Conceptual Blending
## Description
* Implement of [Conceptual Blending](https://en.wikipedia.org/wiki/Conceptual_blending) algorithm in [OpenCog Framework](https://github.com/opencog/opencog).

## Usage
* Currently it supports a random blending.
```python
from blending.blend import ConceptualBlending
ConceptualBlending(a).run(focus_atoms, config_base)
```
* Make a new blend with atoms from given atoms list.
* Support the custom configs. See [Conceptual Blending Config Format](doc/blend-config-format.md).
* Arguments
 * focus_atoms: If it is None, then blender will try to blend with all atoms in atomspace.
 * config_base: If it is None, then blender will try to blend with default (hard-coded) configs.

## Examples
* See [Conceptual Blending Examples](../../examples/python/conceptual_blending)

## Development Progress (For GSoC 2015)
* See [Development Progress](http://wiki.dong-min.kim/GSoC_2015_-_Conceptual_Blending#Progress)

## Files
* blend.py: Conceptual Blending algorithm.

## Folders
* src/chooser: Atom choosers to use for choose atoms in blender.
* util: Several utils.
