##
# @file m_util.py
# @brief developing python library
# @author Dingjie.Wang
# @version 1.0
# @date 2012-08-04


import re
import inspect
from pprint import pprint

# ---------------------------------------------------------------------
def format_log(offset, dsp_caller = True, *args):
    '''  '''
    caller = ""
    if dsp_caller:
        stack = inspect.stack()
        caller = " -- %s %s" % (stack[1][2], stack[1][3])

    out =  ' ' * offset +  ' '.join(map(str, args)) + caller
    return out

# Note that this has the same name, but works differently, than the
# logger class in opencog/cython/opencog/logger.pyx
#
# Note that there is yet a third logger in
# opencog/python/util/util.py that is like this, but not colorized.
# XXX FIXME All these different versions should be merged to one
# version.
#
class Logger(object):
    DEBUG = 0
    INFO = 1
    WARNING = 2
    ERROR = 3
    # colorful output
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    COLOR_END = '\033[0m'
    def __init__(self, f = None):
        # open logging file
        if f:
            try:
                self._file = open(f,'w')
            except IOError:
                print " error: can't open logging file %s " % f
        self._filename = f
        # default setting
        self.offset = 0
        self.to_stdout = True
        self.to_file = True
        self._levels = set()
        self.add_level(Logger.ERROR)

    def debug(self,msg, head = "" ):
        try:
            if self.to_file and Logger.DEBUG in self._levels:
                temp = "[DEBUG]" + str(head) + ":" + str(msg) if head else "[DEBUG]" + str(msg)
                print >>self._file, temp
        except IOError:
            print  Logger.RED + " error: can't write logging file %s " % self._filename + Logger.COLOR_END

        if self.to_stdout and Logger.DEBUG in self._levels:
            temp = "[DEBUG]" + str(head) + ":" + str(msg) if head else "[DEBUG]" + str(msg)
            print Logger.BLUE + temp + Logger.COLOR_END

    def info(self, msg, head = "" ):
        try:
            if self.to_file and Logger.INFO in self._levels:
                temp = "[INFO]" + str(head) + ":" + str(msg) if head else "[INFO]" + str(msg)
                print >>self._file, temp
        except IOError:
            print  Logger.RED + " error: can't write logging file %s " % self._filename + Logger.COLOR_END

        if self.to_stdout and Logger.INFO in self._levels:
            temp = "[INFO]" + str(head) + ":" + str(msg) if head else "[INFO]" + str(msg)
            print Logger.GREEN + temp + Logger.COLOR_END


    def warning(self,msg, head = "" ):
        try:
            if self.to_file and Logger.WARNING in self._levels:
                temp = "[WARNING]" + str(head) + ":" + str(msg) if head else "[WARNING]" + str(msg)
                print >>self._file, temp
        except IOError:
            print  Logger.RED + " error: can't write logging file %s " % self._filename + Logger.COLOR_END


        if self.to_stdout and Logger.WARNING in self._levels:
            temp = "[WARNING]" + str(head) + ":" + str(msg) if head else "[WARNING]" + str(msg)
            print Logger.YELLOW + temp + Logger.COLOR_END

    def error(self, msg, head = "" ):
        try:
            if self.to_file and Logger.ERROR in self._levels:
                temp = "[ERROR]" + str(head) + ":" + str(msg) if head else "[ERROR]" + str(msg)
                print >>self._file, temp
        except IOError:
            print  Logger.RED + " error: can't write logging file %s " % self._filename + Logger.COLOR_END

        if self.to_stdout and Logger.ERROR in self._levels:
            temp = "[ERROR]" + str(head) + ":" + str(msg) if head else "[ERROR]" + str(msg)
            print Logger.RED + temp + Logger.COLOR_END

    def pprint(self, obj, head = "" ):
        '''docstring for pprint()'''
        try:
            if self.to_file:
                #print head
                pprint(obj, self._file)
        except IOError:
            print  Logger.RED + " error: can't write logging file %s " % self._filename + Logger.COLOR_END
        if self.to_stdout:
            #print str(head)
            pprint(obj)

    def flush(self):
        self._file.flush()

    def use_stdout(self, use):
        self.to_stdout = use

        #def setLevel(self, level):
        #self._levels.append(level)
    def add_level(self, level):
        self._levels.add(level)
        '''docstring for add_level'''

log = Logger("default.log")

# --------------------------------------------------------------------------------------------------------------
def dict_sub(text, d):
    """ Replace in 'text' non-overlapping occurences of REs whose patterns are keys
    in dictionary 'd' by corresponding values (which must be constant strings: may
    have named backreferences but not numeric ones). The keys must not contain
    anonymous matching-groups.
    Returns the new string."""
    try:
    # Create a regular expression  from the dictionary keys
        regex = re.compile("|".join("(%s)" % k for k in d))
        # Facilitate lookup from group number to value
        lookup = dict((i+1, v) for i, v in enumerate(d.itervalues()))

        # For each match, find which group matched and expand its value
        return regex.sub(lambda mo: mo.expand(lookup[mo.lastindex]), text)
    except Exception:
        return text
# --------------------------------------------------------------------------------------------------------------
class hs_dict(dict):
    """hashable dict 
    @attention: should not be modified after added to a set or dict!"""
    def __init__(self, arg = None):
        super(hs_dict, self).__init__(arg)
        self._hash = None
        #self._dict = { }

    def __cmp__(self, other):
        '''docstring for __c'''
        if len(self) < len(other):
            return -1
        elif len(self) > len(other):
            return 1

    def __eq__(self, other):
        return tuple(sorted(self.items())) == tuple(sorted(other.items()))
        #def __setitem__(self, key, value):
        #'''docstring for __setitem__''' 
        #pass

    def __hash__(self):
        if not self._hash:
            self._hash = hash(tuple(sorted(self.items())))
            return self._hash
        else:
            return self._hash
# --------------------------------------------------------------------------------------------------------------
from datetime import datetime

class Simple_Time_Interval(object):
    """ help to make a rough estimate about the time interval of two time points in seconds"""
    def __init__(self):
        self.start_time_stamp = None
        self.end_time_stamp = None
        # in seonds
        self.interval_time_stamp = None

    def start(self):
        self.start_time_stamp = datetime.now()

    def end(self):
        self.end_time_stamp = datetime.now()

    def interval(self):
        ''' return interval in seconds'''
        self.interval_time_stamp = (self.end_time_stamp - self.start_time_stamp).seconds
        return self.interval_time_stamp

time_interval = Simple_Time_Interval()
# --------------------------------------------------------------------------------------------------------------

def rough_compare_files(s_filename, t_filename):
    ''' roughly estimate if two files is the same consider lines in random order'''
    try:
        s_file = open(s_filename, 'r')
        t_file = open(t_filename, 'r')
        diff_file = open("diff.log", 'w')
        print s_filename + " not including:"
        print >> diff_file, s_filename + " not including:"
        # 
        source = set()
        for line in s_file.readlines():
            source.add(line)
            # compare it with output of atomspace load with cogserver
        for i,line in enumerate(t_file.readlines()):
            if line not in source:
                print "line %s failed: %s"%(i+1,line)
                print >> diff_file, "line %s failed: %s"%(i+1,line)
    except IOError,e:
        print e
        #print >> diff_file, e
        #s_file.close()
        #t_file.close()
        #diff_file.close()
        return False
    else:
        return True
# --------------------------------------------------------------------------------------------------------------

from SimpleXMLRPCServer import SimpleXMLRPCServer

class RpcServer(object):
    """docstring for RpcServer"""
    def __init__(self, port = 8000, ip = "localhost"):
        _ip = ip
        _port = port
        _server = SimpleXMLRPCServer((ip, port))
    def register_function(func, funcName):
        '''docstring for register_function'''
        _server.register_function(func, funcName)
    def run(self):
        '''docstring for run'''
        server.serve_forever()
        log.info("server is Listening on port " + port)

rpcServer = RpcServer()
# --------------------------------------------------------------------------------------------------------------
__all__ = ["log", "time_interval", "Logger", "dict_sub","hs_dict","format_log", "RpcServer", "rpcServer" ]
