##
# @file m_util.py
# @brief developing python library
# @author Dingjie.Wang
# @version 1.0
# @date 2012-08-04


import re
import inspect

def replace_with_dict(s, d):
    '''replace key words in a string with dictionary '''
    pattern = re.compile(r'\b(' + '|'.join(d.keys()) + r')\b')
    return pattern.sub(lambda x: d[x.group()], s)

def format_log(offset, dsp_caller = True, *args):
    '''  '''
    caller = "" 
    if dsp_caller:
        stack = inspect.stack()
        caller = " -- %s %s" % (stack[1][2], stack[1][3])

    out =  ' ' * offset +  ' '.join(map(str, args)) + caller
    return out

class Logger(object):
    def __init__(self, f = 'opencog-python.log'):
        self._file = open(f,'w')
        self.to_stdout = True
        self._leves = []
        self.DEBUG = 0
        self.INFO = 1
        self.WARNING = 2
        self.ERROR = 3
        #
        self.trace = True
        self.ident = 0
    
    def debug(self,msg):
        if self.trace and self.DEBUG in self._leves:
            print >>self._file, "[DEBUG]:"  + str(msg)
        if self.to_stdout:
            print "[DEBUG]:" +  str(msg)
            #pprint("[DEBUG]:"  + str(msg))

    def info(self, msg):
        if self.trace and self.INFO in self._leves:
            print >>self._file, "[INFO]:"  + str(msg)
        if self.to_stdout:
            print "[INFO]:" +  str(msg)

    def warning(self,msg):
        if self.trace and self.WARNING in self._leves:
            print >>self._file, "[WARNING]:"  + str(msg)
        if self.to_stdout:
            print "[WARNING]:" +  str(msg)

    def error(self, msg):
        if self.trace and self.ERROR in self._leves:
            print >>self._file, "[ERROR]:"  + str(msg)
        if self.to_stdout:
            print "[ERROR]:" +  str(msg)

    def flush(self):
        self._file.flush()
    
    def use_stdout(self, use):
        self.to_stdout = use

    def setLevel(self, level):
        self._leves.append(level)
log = Logger()
log.setLevel(log.ERROR)
