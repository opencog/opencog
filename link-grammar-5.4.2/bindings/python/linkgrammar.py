# -*- coding: utf8 -*-
"""
High-level Python bindings build on top of the low-level C API (clinkgrammar)
See http://www.abisource.com/projects/link-grammar/api/index.html to get
more information about C API.
"""

try:
    #pylint: disable=no-name-in-module
    import linkgrammar.clinkgrammar as clg
except ImportError:
    #pylint: disable=import-error
    import clinkgrammar as clg

Clinkgrammar = clg
__all__ = ['ParseOptions', 'Dictionary', 'Link', 'Linkage', 'Sentence',
           'LG_Error', 'LG_DictionaryError', 'LG_TimerExhausted', 'Clinkgrammar']

# A decorator to ensure keyword-only arguments to __init__ (besides self).
# In Python3 it can be done by using "*" as the second __init__ argument,
# but here it is done with a decorator so it will work also in Python2.
def kwargs_only(init):
    def new_init(self_, *args, **kwargs):
        if args:
            raise TypeError("{}: Positional arguments are "
                            "not allowed".format(self_.__class__.__name__))
        return init(self_, **kwargs)
    return new_init

class ParseOptions(object):
    @kwargs_only
    def __init__(self, verbosity=0,
                 linkage_limit=100,
                 min_null_count=0,
                 max_null_count=0,
                 islands_ok=False,
                 short_length=16,
                 all_short_connectors=False,
                 display_morphology=False,
                 spell_guess=False,
                 use_sat=False,
                 max_parse_time=-1,
                 disjunct_cost=2.7):

        self._obj = clg.parse_options_create()
        self.verbosity = verbosity
        self.linkage_limit = linkage_limit
        self.min_null_count = min_null_count
        self.max_null_count = max_null_count
        self.islands_ok = islands_ok
        self.short_length = short_length
        self.all_short_connectors = all_short_connectors
        self.display_morphology = display_morphology
        self.spell_guess = spell_guess
        self.use_sat = use_sat
        self.max_parse_time = max_parse_time
        self.disjunct_cost = disjunct_cost

    # Allow only the attribute names listed below.
    def __setattr__(self, name, value):
        if not hasattr(self, name) and name != "_obj":
            # TypeError for consistency. It maybe should have been NameError.
            raise TypeError('Unknown parse option "{}".'.format(name))
        super(self.__class__, self).__setattr__(name, value)

    def __del__(self):
        if hasattr(self, '_obj'):
            clg.parse_options_delete(self._obj)
            del self._obj

    @property
    def verbosity(self):
        """
        This is the level of description printed to stderr/stdout about the
        parsing process.
        """
        return clg.parse_options_get_verbosity(self._obj)

    @verbosity.setter
    def verbosity(self, value):
        if not isinstance(value, int):
            raise TypeError("Verbosity must be set to an integer")
        if value not in range(0,15):
            raise ValueError("Verbosity levels can be any integer between 0 and 15 inclusive")
        clg.parse_options_set_verbosity(self._obj, value)

    @property
    def linkage_limit(self):
        """
        This parameter determines the maximum number of linkages that are
        considered in post-processing. If more than linkage_limit linkages
        found, then a random sample of linkage_limit is chosen for
        post-processing. When this happen a warning is displayed at verbosity
        levels bigger than 1.
        """
        return clg.parse_options_get_linkage_limit(self._obj)

    @linkage_limit.setter
    def linkage_limit(self, value):
        if not isinstance(value, int):
            raise TypeError("Linkage Limit must be set to an integer")
        if value < 0:
            raise ValueError("Linkage Limit must be positive")
        clg.parse_options_set_linkage_limit(self._obj, value)

    @property
    def disjunct_cost(self):
        """
        Determines the maximum disjunct cost used during parsing, where the
        cost of a disjunct is equal to the maximum cost of all of its connectors.
        The default is that only disjuncts of cost 2.7 or less are considered.
        """
        return clg.parse_options_get_disjunct_cost(self._obj)

    @disjunct_cost.setter
    def disjunct_cost(self, value):
        if not isinstance(value, float):
            raise TypeError("Distjunct cost must be set to a float")
        clg.parse_options_set_disjunct_cost(self._obj, value)

    @property
    def min_null_count(self):
        """
         The minimum number of null links that a parse might have. A call to
         sentence_parse will find all linkages having the minimum number of
         null links within the range specified by this parameter.
        """
        return clg.parse_options_get_min_null_count(self._obj)

    @min_null_count.setter
    def min_null_count(self, value):
        if not isinstance(value, int):
            raise TypeError("min_null_count must be set to an integer")
        if value < 0:
            raise ValueError("min_null_count must be positive")
        clg.parse_options_set_min_null_count(self._obj, value)

    @property
    def max_null_count(self):
        """
         The maximum number of null links that a parse might have. A call to
         sentence_parse will find all linkages having the minimum number of
         null links within the range specified by this parameter.
        """
        return clg.parse_options_get_max_null_count(self._obj)

    @max_null_count.setter
    def max_null_count(self, value):
        if not isinstance(value, int):
            raise TypeError("max_null_count must be set to an integer")
        if value < 0:
            raise ValueError("max_null_count must be positive")
        clg.parse_options_set_max_null_count(self._obj, value)

    @property
    def short_length(self):
        """
         The short_length parameter determines how long the links are allowed
         to be. The intended use of this is to speed up parsing by not
         considering very long links for most connectors, since they are very
         rarely used in a correct parse. An entry for UNLIMITED-CONNECTORS in
         the dictionary will specify which connectors are exempt from the
         length limit.
        """
        return clg.parse_options_get_short_length(self._obj)

    @short_length.setter
    def short_length(self, value):
        if not isinstance(value, int):
            raise TypeError("short_length must be set to an integer")
        if value < 0:
            raise ValueError("short_length must be positive")
        clg.parse_options_set_short_length(self._obj, value)

    @property
    def islands_ok(self):
        """
        This option determines whether or not "islands" of links are allowed.
        For example, the following linkage has an island:
            +------Wd-----+
            |     +--Dsu--+---Ss--+-Paf-+      +--Dsu--+---Ss--+--Pa-+
            |     |       |       |     |      |       |       |     |
          ///// this sentence.n is.v false.a this sentence.n is.v true.a
        """
        return clg.parse_options_get_islands_ok(self._obj) == 1

    @islands_ok.setter
    def islands_ok(self, value):
        if not isinstance(value, bool):
            raise TypeError("islands_ok must be set to a bool")
        clg.parse_options_set_islands_ok(self._obj, 1 if value else 0)

    @property
    def max_parse_time(self):
        """
         Determines the approximate maximum time (in seconds) that parsing is
         allowed to take. After this time has expired, the parsing process is
         artificially forced to complete quickly by pretending that no further
         solutions can be constructed. The actual parsing time might be
         slightly longer.
        """
        return clg.parse_options_get_max_parse_time(self._obj)

    @max_parse_time.setter
    def max_parse_time(self, value):
        if not isinstance(value, int):
            raise TypeError("max_parse_time must be set to an integer")
        clg.parse_options_set_max_parse_time(self._obj, value)

    @property
    def display_morphology(self):
        """
        Whether or not to show word morphology when a linkage diagram is printed.
        """
        return clg.parse_options_get_display_morphology(self._obj) == 1

    @display_morphology.setter
    def display_morphology(self, value):
        if not isinstance(value, bool):
            raise TypeError("display_morphology must be set to a bool")
        clg.parse_options_set_display_morphology(self._obj, 1 if value else 0)

    @property
    def spell_guess(self):
        """
         If greater then 0, the spelling guesser is used on unknown words.
         In that case, it performs at most this number of spell corrections
         per word, and performs run-on corrections which are not limited in
         their number. If 0 - the spelling guesser would not be used.
        """
        return clg.parse_options_get_spell_guess(self._obj)

    @spell_guess.setter
    def spell_guess(self, value):
        """
         If the value is an int, it is the maximum number of spell corrections
         per word. If it is True, an int value of 7 is assumed. A value of
         0 or False disables the spelling guesser.
         In case the spelling guesser is not disabled, run-on corrections will
         be issued too, not limited in their number.
        """
        if not isinstance(value, bool) and (not isinstance(value, int) or value < 0):
            raise TypeError("spell_guess must be set to bool or a non-negative integer")
        if isinstance(value, bool):
            value = 7 if value else 0
        clg.parse_options_set_spell_guess(self._obj, value)

    @property
    def use_sat(self):
        """
        To be used after enabling the use of the SAT solver in order to
        validate that it is supported by the LG library.
        """
        return clg.parse_options_get_use_sat_parser(self._obj)

    @use_sat.setter
    def use_sat(self, value):
        if not isinstance(value, bool):
            raise TypeError("use_sat must be set to a bool")
        clg.parse_options_set_use_sat_parser(self._obj, value)

    @property
    def all_short_connectors(self):
        """
         If true, then all connectors have length restrictions imposed on
         them -- they can be no farther than short_length apart. This is
         used when parsing in \"panic\" mode, for example.
        """
        return clg.parse_options_get_all_short_connectors(self._obj) == 1

    @all_short_connectors.setter
    def all_short_connectors(self, value):
        if not isinstance(value, bool):
            raise TypeError("all_short_connectors must be set to a bool")
        clg.parse_options_set_all_short_connectors(self._obj, 1 if value else 0)


class LG_Error(Exception):
    @staticmethod
    def set_handler(ehandler_function, ehandler_data=None):
        old_handler = clg._py_error_set_handler((ehandler_function, ehandler_data))
        if isinstance(old_handler, str):
            return LG_Error._default_handler
        return old_handler

    # lg_error_formatmsg is implemented as method "formatmsg" on errinfo
    #@staticmethod
    #def format(lgerror):
    #    return clg.lg_error_formatmsg(lgerror)

    @staticmethod
    def flush():
        return clg.lg_error_flush()

    @staticmethod
    def printall(ehandler_func, ehandler_data=None):
        return clg._py_error_printall((ehandler_func, ehandler_data))

    @staticmethod
    def clearall():
        return clg.lg_error_clearall()

    @staticmethod
    def message(msg):
        """Print a message through the LG error facility"""
        # Propagate a possible ending "\n" into the format, from which the LG
        # error facility determine if this is a partial or a complete message.
        if msg[-1:] == "\n":      # a newline-ended complete message
            _local_eol = "\n";
            msg = msg[:-1]
        elif msg[-2:] == "\n\\":  # a newline-ended partial message
            _local_eol = "";
            msg = msg[:-1]
        else:
            _local_eol = ""       # a partial message

        return clg._prt_error('%s'+_local_eol, msg)

    @staticmethod
    def _default_handler(errinfo, data):
        # Exceptions (on data): TypeError, ValueError
        clg._py_error_default_handler(errinfo, data)

class LG_DictionaryError(LG_Error):
    pass

class Dictionary(object):
    def __init__(self, lang='en'):
        self._obj = clg.dictionary_create_lang(lang)
        if not self._obj:
            # We should get the error message from the library.
            raise LG_DictionaryError('Error: Failed to open dictionary {!r}'.format(lang))

    def __str__(self):
        return clg.dictionary_get_lang(self._obj)

    def __del__(self):
        if hasattr(self, '_obj'):
            clg.dictionary_delete(self._obj)
            del self._obj

    def __nonzero__(self):
        """Return False iff the dictionary could not be opened or has been closed"""
        if not hasattr(self, '_obj'):
            return False
        return bool(self._obj)

    __bool__ = __nonzero__      # Account python3

    def linkgrammar_get_dict_version(self):
        return clg.linkgrammar_get_dict_version(self._obj)

    def linkgrammar_get_dict_locale(self):
        return clg.linkgrammar_get_dict_locale(self._obj)


class Link(object):
    def __init__(self, linkage, index, left_word, left_label, right_label, right_word):
        self.linkage, self.index = linkage, index
        self.left_word, self.right_word, self.left_label, self.right_label = \
            left_word, right_word, left_label, right_label

    def __eq__(self, other):
        return self.left_word == other.left_word and self.left_label == other.left_label and \
               self.right_word == other.right_word and self.right_label == other.right_label

    def __str__(self):
        if self.left_label == self.right_label:
            return u"%s-%s-%s" % (self.left_word, self.left_label, self.right_word)
        else:
            return u"%s-%s-%s-%s" % (self.left_word, self.left_label, self.right_label, self.right_word)

    def __unicode__(self):
        return self.__str__()

    def __repr__(self):
        return u"Link: %s" % self.__str__()

    def __len__(self):
        return clg.linkage_get_link_length(self.linkage._obj, self.index)

    def num_domains(self):
        return clg.linkage_get_link_num_domains(self.linkage._obj, self.index)



class Linkage(object):

    def __init__(self, idx, sentence, parse_options):
        # Keep all args passed into clg.* functions.
        self.sentence, self.parse_options = sentence, parse_options
        self._obj = clg.linkage_create(idx, sentence._obj, parse_options)

    def __del__(self):
        if hasattr(self, '_obj'):
            clg.linkage_delete(self._obj)
            del self._obj

    def __nonzero__(self):
        """Return False for SAT sentinel value (NULL); else return True."""
        return bool(self._obj)

    __bool__ = __nonzero__      # Account python3


    def num_of_words(self):
        return clg.linkage_get_num_words(self._obj)

    def num_of_links(self):
        return clg.linkage_get_num_links(self._obj)

    def words(self):
        for i in range(self.num_of_words()):
            yield self.word(i)

    def word(self, i):
        return clg.linkage_get_word(self._obj, i)

    def unused_word_cost(self):
        return clg.linkage_unused_word_cost(self._obj)

    def link_cost(self):
        return clg.linkage_link_cost(self._obj)

    def disjunct_cost(self):
        return clg.linkage_disjunct_cost(self._obj)

    def link(self, i):
        return Link(self, i, self.word(clg.linkage_get_link_lword(self._obj, i)),
                    clg.linkage_get_link_llabel(self._obj, i),
                    clg.linkage_get_link_rlabel(self._obj, i),
                    self.word(clg.linkage_get_link_rword(self._obj, i)))

    def links(self):
        for i in range(self.num_of_links()):
            yield self.link(i)

    def violation_name(self):
        return clg.linkage_get_violation_name(self._obj)

    def diagram(self, display_walls=False, screen_width=180):
        return clg.linkage_print_diagram(self._obj, display_walls, screen_width)

    def postscript(self, display_walls=True, print_ps_header=False):
        return clg.linkage_print_postscript(self._obj, display_walls, print_ps_header)

    def senses(self):
        return clg.linkage_print_senses(self._obj)

    def constituent_tree(self, mode=1):
        return clg.linkage_print_constituent_tree(self._obj, mode)

    def word_byte_start(self, w):
        return clg.linkage_get_word_byte_start(self._obj, w)

    def word_byte_end(self, w):
        return clg.linkage_get_word_byte_end(self._obj, w)

    def word_char_start(self, w):
        return clg.linkage_get_word_char_start(self._obj, w)

    def word_char_end(self, w):
        return clg.linkage_get_word_char_end(self._obj, w)


class LG_TimerExhausted(LG_Error):
    pass

class Sentence(object):
    """
    sent = Sentence("This is a test.", Dictionary(), ParseOptions())
    # split() before parse() is optional.
    # split() has ParseOptions as an optional argument
    # (defaults to that of Sentence)
    if sent.split(ParseOptions(verbosity=2)) < 0:
        print "Cannot split sentence"
    else
        linkages = sent.parse()
        print "English: found ", sent.num_valid_linkages(), "linkages"
        for linkage in linkages:
            print linkage.diagram()
    """
    def __init__(self, text, lgdict, parse_options):
        # Keep all args passed into clg.* functions.
        self.text, self.dict, self.parse_options = text, lgdict, parse_options
        self._obj = clg.sentence_create(self.text, self.dict._obj)

    def __del__(self):
        if hasattr(self, '_obj'):
            clg.sentence_delete(self._obj)
            del self._obj

    def split(self, parse_options=None):
        """Split a sentence. If an error occurs, return a negative number."""
        if not parse_options:
            parse_options = self.parse_options
        return clg.sentence_split(self._obj, parse_options._obj)

    def __len__(self):
        """The number of tokens in the sentence."""
        return clg.sentence_length(self._obj)

    def null_count(self):
        """Number of null links in the linkages of this sentence."""
        return clg.sentence_null_count(self._obj)

    class sentence_parse(object):
        def __init__(self, sent):
            self.sent = sent
            self.num = 0
            self.rc = clg.sentence_parse(sent._obj, sent.parse_options._obj)
            if clg.parse_options_timer_expired(sent.parse_options._obj):
                raise LG_TimerExhausted()

        def __nonzero__(self):
            """Return False if there was a split or parse error; else return True."""
            return self.rc >= 0

        __bool__ = __nonzero__      # Account python3

        def __iter__(self):
            if 0 == clg.sentence_num_valid_linkages(self.sent._obj):
                return iter(())
            return self

        def __len__(self):
            return clg.sentence_num_valid_linkages(self.sent._obj)

        def next(self):
            if self.num == clg.sentence_num_valid_linkages(self.sent._obj):
                raise StopIteration()
            linkage = Linkage(self.num, self.sent, self.sent.parse_options._obj)
            if not linkage:  # SAT sentinel value
                raise StopIteration()
            self.num += 1
            return linkage

        __next__ = next             # Account python3

    def parse(self):
        return self.sentence_parse(self)
