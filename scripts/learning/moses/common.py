import datetime

def datetime_from_str(time_str):
    """Return <datetime.datetime() instance> for the given
    datetime string given in OpenCog's date time log format
    >>> _datetime_from_str("2009-12-25 13:05:14:453")
    datetime.datetime(2009, 12, 25, 13, 5, 14, 453000)
    """
    fmt = "%Y-%m-%d %H:%M:%S:%f"
    return datetime.datetime.strptime(time_str, fmt)
