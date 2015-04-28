from libcpp cimport bool
from cython.operator cimport dereference as deref, preincrement as inc

# basic wrapping for std::string conversion
cdef extern from "<string>" namespace "std":
    cdef cppclass string:
        string()
        string(char *)
        char * c_str()
        int size()


cdef extern from "opencog/util/Logger.h" namespace "opencog":
    # Need to get around cython's lack of support for nested types
    enum loglevel "opencog::Logger::Level":
        NONE "opencog::Logger::NONE"
        ERROR "opencog::Logger::ERROR"
        WARN "opencog::Logger::WARN"
        INFO "opencog::Logger::INFO"
        DEBUG "opencog::Logger::DEBUG"
        FINE "opencog::Logger::FINE"
        BAD_LEVEL "opencog::Logger::BAD_LEVEL"
    cdef cppclass cLogger "opencog::Logger":
        cLogger()
        cLogger(string s)
        void setLevel(loglevel lvl)
        loglevel getLevel()
        void setPrintToStdoutFlag(bool flag)

        void log(loglevel lvl, string txt)

        bool isEnabled(loglevel lvl)

    cdef loglevel string_to_log_level "opencog::Logger::getLevelFromString"(string s)
    cdef string log_level_to_string "opencog::Logger::getLevelString"(loglevel lvl)
    cLogger& logger()

def create_logger(filename):
    cdef Logger l = Logger.__new__(Logger)
    py_byte_string = filename.encode('UTF-8')
    # create temporary cpp string
    cdef string *c_filename = new string(py_byte_string)
    l.clog = new cLogger(deref(c_filename))
    l.owns_atomspace = True
    # delete temporary string
    del c_filename 
    return l

cdef class Logger:
    cdef cLogger *clog
    cdef owns_atomspace
    
    def __cinit__(self):
        self.owns_atomspace = False
    def __dealloc__(self):
        if self.owns_atomspace: del self.clog
    def __init__(self):
        self.clog = &logger()
    property NONE:
        def __get__(self): return NONE
    property ERROR:
        def __get__(self): return ERROR
    property WARN:
        def __get__(self): return WARN
    property INFO:
        def __get__(self): return INFO
    property DEBUG:
        def __get__(self): return DEBUG
    property FINE:
        def __get__(self): return FINE
    cdef _set_level(self,int lvl):
        self.clog.setLevel(<loglevel>lvl)
    def set_level(self,level_name):
        level_name = level_name.upper()
        py_byte_string = level_name.encode('UTF-8')
        # create temporary cpp string
        cdef string *c_level_name = new string(py_byte_string)
        loglvl = string_to_log_level(deref(c_level_name))
        del c_level_name
        if (loglvl == BAD_LEVEL): raise ValueError("Bad level name")
        self.clog.setLevel(loglvl)
        return loglvl

    def string_as_level(self, lvlname):
        level_name = lvlname.upper()
        py_byte_string = level_name.encode('UTF-8')
        # create temporary cpp string
        cdef string *c_level_name = new string(py_byte_string)
        loglvl = string_to_log_level(deref(c_level_name))
        del c_level_name
        return loglvl
    cdef _level_as_string(self):
        cdef string level_name
        level_name = log_level_to_string(<loglevel>self.clog.getLevel())
        return level_name.c_str()[:level_name.size()].decode('UTF-8')
    def get_level(self):
        return self._level_as_string()

    def level_order(self):
        return ["NONE","ERROR","WARN","INFO","DEBUG","FINE"]

    def log(self, int lvl, txt):
        # why do we use an int for lvl? it is due to a weird cython issue that tries
        # to convert lvl into a long if the type of lvl is loglevel.
        # convert to string
        py_byte_string = txt.encode('UTF-8')
        # create temporary cpp string
        cdef string *msg = new string(py_byte_string)
        self.clog.log(<loglevel>lvl,deref(msg))
        # delete temporary string
        del msg
    def error(self, txt):
        self.log(ERROR,txt)
    def warn(self, txt):
        self.log(WARN,txt)
    def info(self, txt):
        self.log(INFO,txt)
    def debug(self, txt):
        self.log(DEBUG,txt)
    def fine(self, txt):
        self.log(FINE,txt)

    def is_enabled(self, int lvl):
        return self.clog.isEnabled(<loglevel>lvl)
    def use_stdout(self,use_it=True):
        self.clog.setPrintToStdoutFlag(use_it)

log = Logger()
        





