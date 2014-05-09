#!/usr/bin/env python

# Reorder chronologically a log.

import re
import sys
import datetime
import argparse

def datetime_from_str(time_str):
    """Return <datetime.datetime() instance> for the given
    datetime string given in OpenCog's date time log format
    >>> _datetime_from_str("2009-12-25 13:05:14:453")
    datetime.datetime(2009, 12, 25, 13, 5, 14, 453000)
    """
    fmt = "%Y-%m-%d %H:%M:%S:%f"
    return datetime.datetime.strptime(time_str, fmt)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Reorder chronologically the log.')
    parser.add_argument('logfile', help='Log file to reorder')
    parser.add_argument('-o', '--output',
                        help='Output file. If unused stdout is used instead')
    args = parser.parse_args()

    timestamp_re = r'\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}:\d{3})\]'
    timestamp_prog = re.compile(timestamp_re)

    # Output file
    of = open(args.output, "w") if args.output else sys.stdout
    
    # Put the structure in a dict (datetime, line number) -> chunk of text
    dt2txt = {}
    dt, dt_line_num = None, None
    line_num = 0
    for l in open(args.logfile):
        m = timestamp_prog.match(l)
        if m:
            dt = datetime_from_str(m.group(1))
            dt_line_num = line_num
            dt2txt[(dt, dt_line_num)] = l            
        elif dt:
            dt2txt[(dt, dt_line_num)] += l
        else:
            # Corner case, it seems the log's first line can be without
            # timestamp
            of.write(l)
        line_num += 1

    # Rewrite it in order
    for dt, dt_line_num in sorted(dt2txt.keys()):
        of.write(dt2txt[(dt, dt_line_num)])
