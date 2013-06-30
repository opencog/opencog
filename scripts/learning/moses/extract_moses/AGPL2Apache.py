#!/usr/bin/env python
#
# Script to replace the AGPL notice in header of a file by the Apache notice.

from optparse import OptionParser

def main(fileName):
    with open(fileName, "r") as f:
        lines = f.readlines()
        
    # find AGPL header if any
    has_AGPL_header = False
    i = 0
    for l in lines:
        if "This program is free software; you can redistribute it and/or modify" in l:
            has_AGPL_header = True
            start_idx = i
        elif "*/" in l:
            end_idx = i
            break
        i += 1

    # replace AGPL by Apache license
    lines_apache = \
        [ " * Licensed under the Apache License, Version 2.0 (the \"License\");\n",
          " * you may not use this file except in compliance with the License.\n",
          " * You may obtain a copy of the License at\n",
          " *\n",
          " *     http://www.apache.org/licenses/LICENSE-2.0\n",
          " *\n",
          " * Unless required by applicable law or agreed to in writing, software\n",
          " * distributed under the License is distributed on an \"AS IS\" BASIS,\n",
          " * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.\n",
          " * See the License for the specific language governing permissions and\n",
          " * limitations under the License.\n",
          " */\n"]
    
    if has_AGPL_header:
        lines_with_apache = lines[:start_idx] + lines_apache + lines[end_idx+1:]
        with open(fileName, "w") as f:
            f.write("".join(lines_with_apache))
    
if __name__ == "__main__":
    parser = OptionParser()
    (options, args) = parser.parse_args()

    fileName = args[0]
    
    main(fileName)

