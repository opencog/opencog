# Conceptual Blending
## Description
* Implement of [Conceptual Blending](https://en.wikipedia.org/wiki/Conceptual_blending) algorithm in [OpenCog Framework](https://github.com/opencog/opencog).

## Usage
* Currently it supports a random blending.
```python
from blending.blend import ConceptualBlending
ConceptualBlending(a).run({
    'atoms-chooser': 'ChooseInSTIRange'
})
```

## Examples
* See [Conceptual Blending Examples](../../examples/python/conceptual_blending)

## Development Progress (For GSoC 2015)
* See [Development Progress](http://wiki.dong-min.kim/GSoC_2015_-_Conceptual_Blending#Progress)

## Files
* blend.py: Conceptual Blending algorithm.

## Folders
* src/chooser: Atom choosers to use for choose atoms in blender.
* util: Several utils.
