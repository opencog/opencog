# opencog-python-blending
## Description
* Implement of Conceptual Blending algorithm in [OpenCog](https://github.com/opencog/opencog) Framework.

## Prerequisites
* [Python 2.7.x](https://www.python.org/downloads/release/python-2710)
* [AtomSpace](https://github.com/opencog/atomspace)
* [OpenCog](https://github.com/opencog/opencog) (with Cython binding)

## Usage
```python
from opencog_b.python.blending.blend import ConceptualBlending
ConceptualBlending(atomspace).run(focus_atoms, config_base)
```
* Make new blend with atoms from given atoms list.
* Use the custom configs. See [Conceptual Blending Config Format](https://github.com/kim135797531/opencog-python-blending/tree/master/doc/blend-config-format.md).
* Options
  * focus_atoms: If it is None, then blender will try to blend with all atoms 
  in atomspace.
  * config_base: If it is None, then blender will try to blend with default
  (hard-coded) configs.
  
## Development Progress  
* See [doc/diary/README.md](https://github.com/kim135797531/opencog-python-blending/tree/master/doc/diary)

## Folders
* doc: API documents, project progress reports, etc.
* examples: Various examples that use the Conceptual Blending algorithm.
* opencog_b: Code of Conceptual Blending algorithm.
  * '_b' is temporary postfix to avoid conflict with real OpenCog Framework.
 
