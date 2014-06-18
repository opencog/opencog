"""
Extends example.py to demonstrate how to generate a visualization from the
DOT file using Graphviz

The visualization is stored in 'example.png'

Refer to README.md
"""

import example

with open('example.dot', 'w') as outfile:
    outfile.write(example.dot_output)

from subprocess import check_call
check_call(['dot', '-Tsvg', 'example.dot', '-o', 'example.svg'])
