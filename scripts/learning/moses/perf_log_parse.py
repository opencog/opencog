#!/usr/bin/env python
import sys
import datetime
from optparse import OptionParser

def _datetime_from_str(time_str):
    """Return <datetime.datetime() instance> for the given
    datetime string given in OpenCog's date time log format
    >>> _datetime_from_str("2009-12-25 13:05:14:453")
    datetime.datetime(2009, 12, 25, 13, 5, 14, 453000)
    """
    fmt = "%Y-%m-%d %H:%M:%S:%f"
    return datetime.datetime.strptime(time_str, fmt)

def perf_log_parse(logFileName, options):
    of = open(options.output_file, "w") if options.output_file else sys.stdout
    header_written = False
    has_init_time = False
    for l in open(logFileName):
        if (not header_written) and ("Demes: # " in l):    # write header
            header = ["time"] + l.partition("Demes: # ")[2].strip().split("\t")
            of.write(",".join(header) + "\n")
            header_written = True
        elif ("Demes: " in l) and ("Demes: # " not in l):
            dp = l.partition("Demes: ")
            # get time
            time_str = dp[0].partition("[INFO]")[0].strip()[1:-1]
            if not has_init_time:
                init_time = _datetime_from_str(time_str)
                has_init_time = True
            time = _datetime_from_str(time_str)
            rel_time = (time - init_time).total_seconds()
            content = [str(rel_time)] + l.partition("Demes: ")[2].strip().split("\t")
            of.write(",".join(content) + "\n")

if __name__ == "__main__":
    # define options
    usage = "usage: %prog MOSES_LOG [options]\n" + \
        "Parse a log file MOSES_LOG and outputs a CSV file" + \
        " describing the learning performance."
    parser = OptionParser(usage)
    #parse options
    parser.add_option("-o", "--output-file", default="",
                      help="File where to write CSV file. If none is provided it writes in the stdout.")
    (options, args) = parser.parse_args()

    if len(args) != 1:
        parser.error("incorrect number of arguments. Use --help to get more information")

    logFileName = args[0]

    perf_log_parse(logFileName, options)
