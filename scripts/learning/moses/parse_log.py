#!/usr/bin/env python
import sys
from common import datetime_from_str
from optparse import OptionParser

def parse_log(logFileName, options):
    of = open(options.output_file, "w") if options.output_file else sys.stdout
    prefix = options.prefix
    prefix_delim = options.prefix + ":"
    header_written = False
    has_init_time = False
    for l in open(logFileName):
        # write header
        if (not header_written) and all(x in l for x in [prefix_delim, "#", "\t"]):
            header = ["time"] + l.partition("#")[2].strip().split("\t")
            of.write(",".join(header) + "\n")
            header_written = True
        elif (prefix_delim in l) and ("#" not in l):
            dp = l.partition(prefix_delim)
            # get time
            time_str = dp[0].partition("[INFO]")[0].strip()[1:-1]
            if not has_init_time:
                init_time = datetime_from_str(time_str)
                has_init_time = True
            time = datetime_from_str(time_str)
            rel_time = (time - init_time).total_seconds()
            content = [str(rel_time)] + dp[2].strip().split("\t")
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
