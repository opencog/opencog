# opencog-python-blending
## Description
* Temp repository to test early version of blending algorithm with python shell.
* Can be executed both in cogserver, linux shell.

## Usage
### Running the program in cogserver shell
```bash
loadpy blending_agent
agents-step blending_agent.BlendingAgent
```
### Running the program in linux shell
```bash
ipython blending_agent.py
```
### Alternatives
* If cogserver can't find your agent, please check ${PYTHON_EXTENSION_DIRS}.
  Or, you can use absolute path.
```bash
loadpy /your/path/opencog-python-blending/blending_agent
agents-step /your/path/opencog-python-blending/blending_agent.BlendingAgent
```
* If you don't have IPython, just run with default python interpreter.
```bash
py blending_agent.py
```
* If you want load agent in your own code, you can use below code.
```python
from util.shell_wrapper import ShellWrapper
inst = ShellWrapper()
inst.run('blending_agent.BlendingAgent')
```

## Files
* blending_agent.py: Main program. (Conceptual Blending MindAgent)
* blending_shell.py: Auto load and run blending for debug.
* blending.conf: Config file.

## Folders
* blender: Available blenders.
* diary: My working progress reports for GSoC 2015 project.
* note: Various notes to save useful information for my project.
* tests: Input data for test conceptual blending.
* util: Several util files.