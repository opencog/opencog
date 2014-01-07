#!/usr/bin/env python
import sys
from common import datetime_from_str
from optparse import OptionParser
import re

def parse_log(logFileName, options):
    of = open(options.output_file, "w") if options.output_file else sys.stdout
    prefix = options.prefix
    header_written = False
    has_init_time = False

    # Define the regex for header and content
    timestamp_re = r'\[\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}:\d{3}\]'
    info_re = r'\[INFO\]'
    header_re = r'{}: # (\w+(?:\t\w+)+)'.format(prefix)
    content_re = r'{}: ([^#\t]+(?:\t.+))+'.format(prefix)
    all_header_re = r'(?:{} {} )?{}'.format(timestamp_re, info_re, header_re)
    all_content_re = r'({}) {} {}'.format(timestamp_re, info_re, content_re)
    all_header_cre = re.compile(all_header_re)
    all_content_cre = re.compile(all_content_re)

    # Read all lines of the log file and fill header and content
    for l in open(logFileName):
        ls = l.strip()
        # write header
        if not header_written:
            hm = all_header_cre.match(ls)
            if hm:
                header = ["time"] + hm.group(1).split('\t')
                of.write(",".join(header) + "\n")
                header_written = True
        else:
            cm = all_content_cre.match(ls)
            if cm:
                # get time
                time_str = cm.group(1)[1:-1]
                if not has_init_time:
                    init_time = datetime_from_str(time_str)
                    has_init_time = True
                time = datetime_from_str(time_str)
                rel_time = (time - init_time).total_seconds()
                content = [str(rel_time)] + cm.group(2).split('\t')
                of.write(",".join(content) + "\n")

if __name__ == "__main__":
    # define options
    usage = "usage: %prog MOSES_LOG [options]\n" + \
        "Parse a log file MOSES_LOG and outputs a CSV file" + \
        " describing the learning performance."
    parser = OptionParser(usage)
    #parse options
    parser.add_option("-p", "--prefix", default="Demes",
                      help="Prefix of statistic to gather (possible prefixes are 'Demes' and 'Stats'). [default = %default]")
    parser.add_option("-o", "--output-file", default="",
                      help="File where to write CSV file. If none is provided it writes in the stdout.")
    (options, args) = parser.parse_args()

    if len(args) != 1:
        parser.error("incorrect number of arguments. Use --help to get more information")

    logFileName = args[0]

    parse_log(logFileName, options)
