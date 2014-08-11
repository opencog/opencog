__author__ = 'sebastian'

"""
Module to run RelEx and load the RelEx2Logic representation of a sentence
directly in Python
"""

from attention_interface import clear_atomspace, scheme, dump_atomspace_scheme

file_name = "syllogisms.txt"

with open(file_name, "r") as f:
    for line in f:
        line = line.strip("\n")
        if line.startswith("#") or line == "":
            continue
        print line
        if line.startswith("a") or line.startswith("b"):
            clear_atomspace()
            scheme("""(relex-parse "{0}")""".format(line.split(".")[1].strip(" ")))
            scheme("(delete-sentences)")
            atomspace_content = dump_atomspace_scheme()
            print atomspace_content

        #elif line.startswith("|-"):