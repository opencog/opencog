# opencog-python-blending
## Description
* Temp repository to test early version of blending algorithm with python shell.
* Can be executed both in cogserver, linux shell.

## Usage
### Running the program in cogserver shell
```bash
agents-stop-loop
loadpy /your/path/opencog-python-blending/blending_agent
agents-start /your/path/opencog-python-blending/blending_agent.BlendingAgent
agents-step opencog::PyMindAgent(/your/path/opencog-python-blending/blending_agent.BlendingAgent)
```
### Running the program in linux shell
```bash
ipython blending_shell.py
```
### Alternatives
* If you don't have IPython, just run with default python interpreter.
```bash
py blending_shell.py
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
* blending.conf.example: Example of config file.

## Folders
* blender_b: Available blenders.
* diary_b: My working progress reports for GSoC 2015 project.
* note_b: Various notes to save useful information for my project.
* tests_b: Input data for test conceptual blending.
* util_b: Several util files.