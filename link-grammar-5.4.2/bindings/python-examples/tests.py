#!/usr/bin/env python
# coding: utf8
"""Python link-grammar test script"""

from __future__ import print_function
import sys, os
import locale
import unittest

import lg_testutils # Found in the same directory of this test script

# Show information on this program run
print('Running by:', sys.executable)
print('Running {} in:'.format(sys.argv[0]), os.getcwd())
for v in 'PYTHONPATH', 'srcdir', 'LINK_GRAMMAR_DATA':
    print('{}={}'.format(v, os.environ.get(v)))
#===


from linkgrammar import (Sentence, Linkage, ParseOptions, Link, Dictionary,
                         LG_Error, LG_DictionaryError, LG_TimerExhausted,
                         Clinkgrammar as clg)


# Show the location and version of the bindings modules
for module in 'linkgrammar', '_clinkgrammar', 'lg_testutils':
    if module in sys.modules:
        print("Using", sys.modules[module], end='')
        if hasattr(sys.modules[module], '__version__'):
            print(' version', sys.modules[module].__version__, end='')
        print()
    else:
        print("Warning: Module", module,  "not loaded.")

sys.stdout.flush()
#===

def setUpModule():
    datadir = os.getenv("LINK_GRAMMAR_DATA", "")
    if datadir:
        clg.dictionary_set_data_dir(datadir)

    clg.test_data_srcdir = os.getenv("srcdir", os.path.dirname(sys.argv[0]))
    if clg.test_data_srcdir:
        clg.test_data_srcdir += "/"

# The tests are run in alphabetical order....
#
# First test: test the test framework itself ...
class AAALinkTestCase(unittest.TestCase):
    def test_link_display_with_identical_link_type(self):
        self.assertEqual(str(Link(None, 0, 'Left','Link','Link','Right')),
                         u'Left-Link-Right')

    def test_link_display_with_identical_link_type2(self):
        self.assertEqual(str(Link(None, 0, 'Left','Link','Link*','Right')),
                         u'Left-Link-Link*-Right')

class AADictionaryTestCase(unittest.TestCase):
    def test_open_nonexistent_dictionary(self):
        self.assertRaises(LG_DictionaryError, Dictionary, 'No such language test 1')
        self.assertRaises(LG_Error, Dictionary, 'No such language test 2')

class BParseOptionsTestCase(unittest.TestCase):
    def test_setting_verbosity(self):
        po = ParseOptions()
        po.verbosity = 2
        #Ensure that the PO object reports the value correctly
        self.assertEqual(po.verbosity, 2)
        #Ensure that it's actually setting it.
        self.assertEqual(clg.parse_options_get_verbosity(po._obj), 2)

    def test_setting_verbosity_to_not_allow_value_raises_value_error(self):
        po = ParseOptions()
        self.assertRaises(ValueError, setattr, po, "verbosity", 16)

    def test_setting_verbosity_to_non_integer_raises_type_error(self):
        po = ParseOptions()
        self.assertRaises(TypeError, setattr, po, "verbosity", "a")

    def test_setting_linkage_limit(self):
        po = ParseOptions()
        po.linkage_limit = 3
        self.assertEqual(clg.parse_options_get_linkage_limit(po._obj), 3)

    def test_setting_linkage_limit_to_non_integer_raises_type_error(self):
        po = ParseOptions()
        self.assertRaises(TypeError, setattr, po, "linkage_limit", "a")

    def test_setting_linkage_limit_to_negative_number_raises_value_error(self):
        po = ParseOptions()
        self.assertRaises(ValueError, setattr, po, "linkage_limit", -1)

    def test_setting_disjunct_cost(self):
        po = ParseOptions()
        po.disjunct_cost = 3.0
        self.assertEqual(clg.parse_options_get_disjunct_cost(po._obj), 3.0)

    def test_setting_disjunct_cost_to_non_integer_raises_type_error(self):
        po = ParseOptions()
        self.assertRaises(TypeError, setattr, po, "disjunct_cost", "a")

    def test_setting_min_null_count(self):
        po = ParseOptions()
        po.min_null_count = 3
        self.assertEqual(clg.parse_options_get_min_null_count(po._obj), 3)

    def test_setting_min_null_count_to_non_integer_raises_type_error(self):
        po = ParseOptions()
        self.assertRaises(TypeError, setattr, po, "min_null_count", "a")

    def test_setting_min_null_count_to_negative_number_raises_value_error(self):
        po = ParseOptions()
        self.assertRaises(ValueError, setattr, po, "min_null_count", -1)

    def test_setting_max_null_count(self):
        po = ParseOptions()
        po.max_null_count = 3
        self.assertEqual(clg.parse_options_get_max_null_count(po._obj), 3)

    def test_setting_max_null_count_to_non_integer_raises_type_error(self):
        po = ParseOptions()
        self.assertRaises(TypeError, setattr, po, "max_null_count", "a")

    def test_setting_max_null_count_to_negative_number_raises_value_error(self):
        po = ParseOptions()
        self.assertRaises(ValueError, setattr, po, "max_null_count", -1)

    def test_setting_short_length(self):
        po = ParseOptions()
        po.short_length = 3
        self.assertEqual(clg.parse_options_get_short_length(po._obj), 3)

    def test_setting_short_length_to_non_integer_raises_type_error(self):
        po = ParseOptions()
        self.assertRaises(TypeError, setattr, po, "short_length", "a")

    def test_setting_short_length_to_negative_number_raises_value_error(self):
        po = ParseOptions()
        self.assertRaises(ValueError, setattr, po, "short_length", -1)

    def test_setting_islands_ok(self):
        po = ParseOptions()
        po.islands_ok = True
        self.assertEqual(po.islands_ok, True)
        self.assertEqual(clg.parse_options_get_islands_ok(po._obj), 1)
        po.islands_ok = False
        self.assertEqual(po.islands_ok, False)
        self.assertEqual(clg.parse_options_get_islands_ok(po._obj), 0)

    def test_setting_islands_ok_to_non_boolean_raises_type_error(self):
        po = ParseOptions()
        self.assertRaises(TypeError, setattr, po, "islands_ok", "a")

    def test_setting_max_parse_time(self):
        po = ParseOptions()
        po.max_parse_time = 3
        self.assertEqual(clg.parse_options_get_max_parse_time(po._obj), 3)

    def test_setting_max_parse_time_to_non_integer_raises_type_error(self):
        po = ParseOptions()
        self.assertRaises(TypeError, setattr, po, "max_parse_time", "a")

    def test_setting_spell_guess_to_non_integer_raises_type_error(self):
        po = ParseOptions()
        self.assertRaises(TypeError, setattr, po, "spell_guess", "a")

    def test_setting_display_morphology(self):
        po = ParseOptions()
        po.display_morphology = True
        self.assertEqual(po.display_morphology, True)
        self.assertEqual(clg.parse_options_get_display_morphology(po._obj), 1)
        po.display_morphology = False
        self.assertEqual(po.display_morphology, False)
        self.assertEqual(clg.parse_options_get_display_morphology(po._obj), 0)

    def test_setting_all_short_connectors(self):
        po = ParseOptions()
        po.all_short_connectors = True
        self.assertEqual(po.all_short_connectors, True)
        self.assertEqual(clg.parse_options_get_all_short_connectors(po._obj), 1)
        po.all_short_connectors = False
        self.assertEqual(po.all_short_connectors, False)
        self.assertEqual(clg.parse_options_get_all_short_connectors(po._obj), 0)

    def test_setting_all_short_connectors_to_non_boolean_raises_type_error(self):
        po = ParseOptions()
        self.assertRaises(TypeError, setattr, po, "all_short_connectors", "a")

    def test_setting_spell_guess(self):
        po = ParseOptions(spell_guess=True)
        if po.spell_guess == 0:
            raise unittest.SkipTest("Library is not configured with spell guess")
        self.assertEqual(po.spell_guess, 7)
        po = ParseOptions(spell_guess=5)
        self.assertEqual(po.spell_guess, 5)
        po = ParseOptions(spell_guess=False)
        self.assertEqual(po.spell_guess, 0)

    def test_specifying_parse_options(self):
        po = ParseOptions(linkage_limit=99)
        self.assertEqual(clg.parse_options_get_linkage_limit(po._obj), 99)

class CParseOptionsTestCase(unittest.TestCase):

    def test_that_sentence_can_be_destroyed_when_linkages_still_exist(self):
        """
        If the parser is deleted before the associated swig objects
        are, there will be bad pointer dereferences (as the swig
        objects will be pointing into freed memory).  This test ensures
        that parsers can be created and deleted without regard for
        the existence of PYTHON Linkage objects
        """
        #pylint: disable=unused-variable
        s = Sentence('This is a sentence.', Dictionary(), ParseOptions())
        linkages = s.parse()
        del s

    def test_that_invalid_options_are_disallowed(self):
        self.assertRaisesRegexp(TypeError, "unexpected keyword argument",
                                ParseOptions, invalid_option=1)

    def test_that_invalid_option_properties_cannot_be_used(self):
        po = ParseOptions()
        self.assertRaisesRegexp(TypeError, "Unknown parse option",
                                setattr, po, "invalid_option", 1)

    def test_that_ParseOptions_cannot_get_positional_arguments(self):
        self.assertRaisesRegexp(TypeError, "Positional arguments are not allowed",
                                ParseOptions, 1)

class DBasicParsingTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.d = Dictionary()
        cls.po = None

    @classmethod
    def tearDownClass(cls):
        del cls.d
        del cls.po

    def parse_sent(self, text, po=ParseOptions()):
        return list(Sentence(text, self.d, po).parse())

    def test_that_parse_returns_empty_iterator_on_no_linkage(self):
        """Parsing a bad sentence with no null-links shouldn't give any linkage."""
        result = self.parse_sent("This this doesn't parse")
        linkage_exists = False
        for _ in result:
            linkage_exists = True
            self.assertFalse(linkage_exists, "Unparsable sentence has linkages.")

    def test_that_parse_returns_empty_iterator_on_no_linkage_sat(self):
        """Parsing a bad sentence with no null-links shouldn't give any linkage (sat)"""
        self.po = ParseOptions(use_sat=True)
        if self.po.use_sat != True:
            raise unittest.SkipTest("Library not configured with SAT parser")
        result = self.parse_sent("This this doesn't parse", self.po)
        linkage_exists = False
        for _ in result:
            linkage_exists = True
            self.assertFalse(linkage_exists, "SAT: Unparsable sentence has linkages.")

    def test_that_parse_sent_returns_list_of_linkage_objects_for_valid_sentence(self):
        result = self.parse_sent("This is a relatively simple sentence.")
        self.assertTrue(isinstance(result[0], Linkage))
        self.assertTrue(isinstance(result[1], Linkage))

    def test_utf8_encoded_string(self):
        result = self.parse_sent("I love going to the café.")
        self.assertTrue(1 < len(result))
        self.assertTrue(isinstance(result[0], Linkage))
        self.assertTrue(isinstance(result[1], Linkage))

        # def test_unicode_encoded_string(self):
        if sys.version_info > (3, 0):
            result = self.parse_sent(u"I love going to the caf\N{LATIN SMALL LETTER E WITH ACUTE}.")
        else:
            result = self.parse_sent(u"I love going to the caf\N{LATIN SMALL LETTER E WITH ACUTE}.".encode('utf8'))
        self.assertTrue(1 < len(result))
        self.assertTrue(isinstance(result[0], Linkage))
        self.assertTrue(isinstance(result[1], Linkage))

        # def test_unknown_word(self):
        result = self.parse_sent("I love going to the qertfdwedadt.")
        self.assertTrue(1 < len(result))
        self.assertTrue(isinstance(result[0], Linkage))
        self.assertTrue(isinstance(result[1], Linkage))

        # def test_unknown_euro_utf8_word(self):
        result = self.parse_sent("I love going to the qéáéğíóşúüñ.")
        self.assertTrue(1 < len(result))
        self.assertTrue(isinstance(result[0], Linkage))
        self.assertTrue(isinstance(result[1], Linkage))

        # def test_unknown_cyrillic_utf8_word(self):
        result = self.parse_sent("I love going to the доктором.")
        self.assertTrue(1 < len(result))
        self.assertTrue(isinstance(result[0], Linkage))
        self.assertTrue(isinstance(result[1], Linkage))

    def test_getting_link_distances(self):
        linkage = self.parse_sent("This is a sentence.")[0]
        self.assertEqual([len(l) for l in linkage.links()], [5,2,1,1,2,1,1])
        linkage = self.parse_sent("This is a silly sentence.")[0]
        self.assertEqual([len(l) for l in linkage.links()], [6,2,1,1,3,2,1,1,1])

    def test_dictionary_locale_definition(self):
        tr_locale = 'tr_TR.UTF-8' if os.name != 'nt' else 'Turkish'
        oldlocale = locale.setlocale(locale.LC_CTYPE, tr_locale)
        self.assertEqual(list(self.parse_sent('Is it fine?')[0].words()),
             ['LEFT-WALL', 'is.v', 'it', 'fine.a', '?', 'RIGHT-WALL'])
        locale.setlocale(locale.LC_CTYPE, oldlocale)

    # If \w is supported, other \ shortcuts are hopefully supported too.
    def test_regex_class_shortcut_support(self):
        r"""Test that regexes support \w"""
        linkage = self.parse_sent("This is a _regex_ive regex test")[0]
        self.assertEqual(linkage.word(4), '_regex_ive[!].a')

    def test_timer_exhausted_exception(self):
        self.po = ParseOptions(max_parse_time=1)
        self.assertRaises(LG_TimerExhausted,
                          self.parse_sent,
                          "This should take more than one second to parse! " * 20,
                          self.po)

# The tests here are are numbered since their order is important.
# They depend on the result and state of the previous ones as follows:
# - set_handler() returned a value that depend on it previous invocation.
# - A class variable "handler" to record its previous results.
class EErrorFacilityTestCase(unittest.TestCase):
    # Initialize class variables to invalid (for the test) values.
    handler = {
        "default":  lambda x, y=None: None,
        "previous": lambda x, y=None: None
    }

    def setUp(self): # pylint: disable=attribute-defined-outside-init,no-member
        self.testit = "testit"
        self.testleaks = 0  # A repeat count for validating no memory leaks

    @staticmethod
    def error_handler_test(errinfo, data):
        # A test error handler.  It assigns the errinfo struct as an attribute
        # of its data so it can be examined after the call. In addition, the
        # ability of the error handler to use its data argument is tested by
        # the "testit" attribute.
        if data is None:
            return
        data.errinfo = errinfo
        data.gotit = data.testit

    def test_10_set_error_handler(self):
        # Set the error handler and validate that it
        # gets the error info and the data.
        self.__class__.handler["default"] = \
            LG_Error.set_handler(self.error_handler_test, self)
        self.assertEqual(self.__class__.handler["default"].__name__,
                         "_default_handler")
        self.gotit = None
        self.assertRaises(LG_Error, Dictionary, "seh_dummy1")
        self.assertEqual((self.errinfo.severity, self.errinfo.severity_label), (clg.lg_Error, "Error"))
        self.assertEqual(self.gotit, "testit")
        self.assertRegexpMatches(self.errinfo.text, "Could not open dictionary.*seh_dummy1")

    def test_20_set_error_handler_None(self):
        # Set the error handler to None and validate that printall()
        # gets the error info and the data and returns the number of errors.
        self.__class__.handler["previous"] = LG_Error.set_handler(None)
        self.assertEqual(self.__class__.handler["previous"].__name__, "error_handler_test")
        self.assertRaises(LG_Error, Dictionary, "seh_dummy2")
        self.gotit = None
        for i in range(0, 2+self.testleaks):
            self.numerr = LG_Error.printall(self.error_handler_test, self)
            if 0 == i:
                self.assertEqual(self.numerr, 1)
            if 1 == i:
                self.assertEqual(self.numerr, 0)
        self.assertEqual((self.errinfo.severity, self.errinfo.severity_label), (clg.lg_Error, "Error"))
        self.assertEqual(self.gotit, "testit")
        self.assertRegexpMatches(self.errinfo.text, ".*seh_dummy2")

    def test_21_set_error_handler_None(self):
        # Further test of correct number of errors.
        self.numerr = 3
        for _ in range(0, self.numerr):
            self.assertRaises(LG_Error, Dictionary, "seh_dummy2")
        self.numerr = LG_Error.printall(self.error_handler_test, None)
        self.assertEqual(self.numerr, self.numerr)

    def test_22_defaut_handler_param(self):
        """Test bad data parameter to default error handler"""
        # (It should be an integer >=0 and <= lg_None.)
        # Here the error handler is still set to None.

        # This test doesn't work - TypeError is somehow raised inside
        # linkgrammar.py when _default_handler() is invoked as a callback.
        #
        #LG_Error.set_handler(self.__class__.handler["default"], "bad param")
        #with self.assertRaises(TypeError):
        #    try:
        #        Dictionary("a visible dummy dict name (bad param test)")
        #    except LG_Error:
        #        pass

        # So test it directly.
        self.assertRaises(LG_Error, Dictionary, "a visible dummy dict name (bad param test)")
        LG_Error.printall(self.error_handler_test, self) # grab a valid errinfo
        self.assertRaisesRegexp(TypeError, "must be an integer",
                                self.__class__.handler["default"],
                                self.errinfo, "bad param")
        self.assertRaisesRegexp(ValueError, "must be an integer",
                                self.__class__.handler["default"],
                                self.errinfo, clg.lg_None+1)
        self.assertRaises(ValueError, self.__class__.handler["default"],
                          self.errinfo, -1)
        try:
            self.param_ok = False
            self.__class__.handler["default"](self.errinfo, 1)
            self.param_ok = True
        except (TypeError, ValueError):
            self.assertTrue(self.param_ok)

    def test_23_prt_error(self):
        LG_Error.message("Info: prt_error test\n")
        LG_Error.printall(self.error_handler_test, self)
        self.assertRegexpMatches(self.errinfo.text, "prt_error test\n")
        self.assertEqual((self.errinfo.severity, self.errinfo.severity_label), (clg.lg_Info, "Info"))

    def test_24_prt_error_in_parts(self):
        LG_Error.message("Trace: part one... ")
        LG_Error.message("part two\n")
        LG_Error.printall(self.error_handler_test, self)
        self.assertEqual(self.errinfo.text, "part one... part two\n")
        self.assertEqual((self.errinfo.severity, self.errinfo.severity_label), (clg.lg_Trace, "Trace"))

    def test_25_prt_error_in_parts_with_embedded_newline(self):
        LG_Error.message("Trace: part one...\n\\")
        LG_Error.message("part two\n")
        LG_Error.printall(self.error_handler_test, self)
        self.assertEqual(self.errinfo.text, "part one...\npart two\n")
        self.assertEqual((self.errinfo.severity, self.errinfo.severity_label), (clg.lg_Trace, "Trace"))

    def test_26_prt_error_plain_message(self):
        LG_Error.message("This is a regular output line.\n")
        LG_Error.printall(self.error_handler_test, self)
        self.assertEqual(self.errinfo.text, "This is a regular output line.\n")
        self.assertEqual((self.errinfo.severity, self.errinfo.severity_label), (clg.lg_None, ""))

    def test_30_formatmsg(self):
        # Here the error handler is still set to None.
        for _ in range (0, 1+self.testleaks):
            self.assertRaises(LG_Error, Dictionary, "formatmsg-test-dummy-dict")
            LG_Error.printall(self.error_handler_test, self)
            self.assertRegexpMatches(self.errinfo.formatmsg(), "link-grammar: Error: .*formatmsg-test-dummy-dict")

    def test_40_clearall(self):
        # Here the error handler is still set to None.
        # Call LG_Error.clearall() and validate it indeed clears the error.
        self.assertRaises(LG_Error, Dictionary, "clearall-test-dummy-dict")
        LG_Error.clearall()
        self.testit = "clearall"
        self.numerr = LG_Error.printall(self.error_handler_test, self)
        self.assertEqual(self.numerr, 0)
        self.assertFalse(hasattr(self, "gotit"))

    def test_41_flush(self):
        # Here the error handler is still set to None.
        # First validate that nothing gets flushed (no error is buffered at this point).
        self.flushed = LG_Error.flush()
        self.assertEqual(self.flushed, False)
        # Now generate a partial error message that is still buffered.
        LG_Error.message("This is a partial error message.")
        # Validate that it is still hidden.
        self.numerr = LG_Error.printall(self.error_handler_test, self)
        self.assertEqual(self.numerr, 0)
        self.assertFalse(hasattr(self, "gotit"))
        # Flush it.
        self.flushed = LG_Error.flush()
        self.assertEqual(self.flushed, True)
        self.numerr = LG_Error.printall(self.error_handler_test, self)
        self.assertEqual(self.numerr, 1)
        self.assertRegexpMatches(self.errinfo.text, "partial")

    def test_50_set_orig_error_handler(self):
        # Set the error handler back to the default handler.
        # The error message is now visible (but we cannot test that).
        self.__class__.handler["previous"] = LG_Error.set_handler(self.__class__.handler["default"])
        self.assertIsNone(self.__class__.handler["previous"])
        for _ in range(0, 1+self.testleaks):
            self.__class__.handler["previous"] = LG_Error.set_handler(self.__class__.handler["default"])
        self.assertEqual(self.__class__.handler["previous"].__name__, "_default_handler")
        self.errinfo = "dummy"
        self.assertRaises(LG_Error, Dictionary, "a visible dummy dict name (default handler test)")
        self.assertEqual(self.errinfo, "dummy")

class FSATsolverTestCase(unittest.TestCase):
    def setUp(self):
        self.d, self.po = Dictionary(lang='en'), ParseOptions()
        self.po = ParseOptions(use_sat=True)
        if self.po.use_sat != True:
            raise unittest.SkipTest("Library not configured with SAT parser")

    def test_SAT_getting_links(self):
        linkage_testfile(self, self.d, self.po, 'sat')

class HEnglishLinkageTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.d, cls.po = Dictionary(), ParseOptions()

    @classmethod
    def tearDownClass(cls):
        del cls.d
        del cls.po

    def parse_sent(self, text):
        return list(Sentence(text, self.d, self.po).parse())

    def test_a_getting_words(self):
        self.assertEqual(list(self.parse_sent('This is a sentence.')[0].words()),
             ['LEFT-WALL', 'this.p', 'is.v', 'a', 'sentence.n', '.', 'RIGHT-WALL'])

    def test_b_getting_num_of_words(self):
        #Words include punctuation and a 'LEFT-WALL' and 'RIGHT_WALL'
        self.assertEqual(self.parse_sent('This is a sentence.')[0].num_of_words(), 7)

    def test_c_getting_links(self):
        sent = 'This is a sentence.'
        linkage = self.parse_sent(sent)[0]
        self.assertEqual(linkage.link(0),
                         Link(linkage, 0, 'LEFT-WALL','Xp','Xp','.'))
        self.assertEqual(linkage.link(1),
                         Link(linkage, 1, 'LEFT-WALL','hWV','dWV','is.v'))
        self.assertEqual(linkage.link(2),
                         Link(linkage, 2, 'LEFT-WALL','Wd','Wd','this.p'))
        self.assertEqual(linkage.link(3),
                         Link(linkage, 3, 'this.p','Ss*b','Ss','is.v'))
        self.assertEqual(linkage.link(4),
                         Link(linkage, 4, 'is.v','O*m','Os','sentence.n'))
        self.assertEqual(linkage.link(5),
                         Link(linkage, 5, 'a','Ds**c','Ds**c','sentence.n'))
        self.assertEqual(linkage.link(6),
                         Link(linkage, 6, '.','RW','RW','RIGHT-WALL'))

    def test_d_spell_guessing_on(self):
        self.po.spell_guess = 7
        if self.po.spell_guess == 0:
            raise unittest.SkipTest("Library is not configured with spell guess")
        result = self.parse_sent("I love going to shoop.")
        resultx = result[0] if result else []
        for resultx in result:
            if resultx.word(5) == 'shop[~].v':
                break
        self.assertEqual(list(resultx.words()) if resultx else [],
             ['LEFT-WALL', 'I.p', 'love.v', 'going.v', 'to.r', 'shop[~].v', '.', 'RIGHT-WALL'])

    def test_e_spell_guessing_off(self):
        self.po.spell_guess = 0
        result = self.parse_sent("I love going to shoop.")
        self.assertEqual(list(result[0].words()),
             ['LEFT-WALL', 'I.p', 'love.v', 'going.v', 'to.r', 'shoop[?].v', '.', 'RIGHT-WALL'])

    # Stress-test first-word-capitalized in various different ways.
    # Roughly, the test matrix is this:
    # -- word is/isn't in dict as lower-case word
    # -- word is/isn't in dict as upper-case word
    # -- word is/isn't matched with CAPITALIZED_WORDS regex
    # -- word is/isn't split by suffix splitter
    # -- the one that is in the dict is not the grammatically appropriate word.
    #
    # Let's is NOT split into two! Its in the dict as one word, lower-case only.
    def test_f_captilization(self):
        self.assertEqual(list(self.parse_sent('Let\'s eat.')[0].words()),
             ['LEFT-WALL', 'let\'s', 'eat.v', '.', 'RIGHT-WALL'])

        # He's is split into two words, he is in dict, lower-case only.
        self.assertEqual(list(self.parse_sent('He\'s going.')[0].words()),
             ['LEFT-WALL', 'he', '\'s.v', 'going.v', '.', 'RIGHT-WALL'])

        self.assertEqual(list(self.parse_sent('You\'re going?')[0].words()),
             ['LEFT-WALL', 'you', '\'re', 'going.v', '?', 'RIGHT-WALL'])

        # Jumbo only in dict as adjective, lower-case, but not noun.
        self.assertEqual(list(self.parse_sent('Jumbo\'s going?')[0].words()),
             ['LEFT-WALL', 'Jumbo[!]', '\'s.v', 'going.v', '?', 'RIGHT-WALL'])

        self.assertEqual(list(self.parse_sent('Jumbo\'s shoe fell off.')[0].words()),
             ['LEFT-WALL', 'Jumbo[!]',
              '\'s.p', 'shoe.n', 'fell.v-d', 'off', '.', 'RIGHT-WALL'])

        self.assertEqual(list(self.parse_sent('Jumbo sat down.')[0].words()),
             ['LEFT-WALL', 'Jumbo[!]', 'sat.v-d', 'down.r', '.', 'RIGHT-WALL'])

        # Red is in dict, lower-case, as noun, too.
        # There's no way to really know, syntactically, that Red
        # should be taken as a proper noun (given name).
        #self.assertEqual(list(self.parse_sent('Red\'s going?')[0].words()),
        #     ['LEFT-WALL', 'Red[!]', '\'s.v', 'going.v', '?', 'RIGHT-WALL'])
        #
        #self.assertEqual(list(self.parse_sent('Red\'s shoe fell off.')[0].words()),
        #     ['LEFT-WALL', 'Red[!]',
        #      '\'s.p', 'shoe.n', 'fell.v-d', 'off', '.', 'RIGHT-WALL'])
        #
        #self.assertEqual(list(self.parse_sent('Red sat down.')[1].words()),
        #     ['LEFT-WALL', 'Red[!]', 'sat.v-d', 'down.r', '.', 'RIGHT-WALL'])

        # May in dict as noun, capitalized, and as lower-case verb.
        self.assertEqual(list(self.parse_sent('May\'s going?')[0].words()),
             ['LEFT-WALL', 'May.f', '\'s.v', 'going.v', '?', 'RIGHT-WALL'])

        self.assertEqual(list(self.parse_sent('May sat down.')[0].words()),
             ['LEFT-WALL', 'May.f', 'sat.v-d', 'down.r', '.', 'RIGHT-WALL'])

        # McGyver is not in the dict, but is regex-matched.
        self.assertEqual(list(self.parse_sent('McGyver\'s going?')[0].words()),
             ['LEFT-WALL', 'McGyver[!]', '\'s.v', 'going.v', '?', 'RIGHT-WALL'])

        self.assertEqual(list(self.parse_sent('McGyver\'s shoe fell off.')[0].words()),
             ['LEFT-WALL', 'McGyver[!]',
              '\'s.p', 'shoe.n', 'fell.v-d', 'off', '.', 'RIGHT-WALL'])

        self.assertEqual(list(self.parse_sent('McGyver sat down.')[0].words()),
             ['LEFT-WALL', 'McGyver[!]', 'sat.v-d', 'down.r', '.', 'RIGHT-WALL'])

        self.assertEqual(list(self.parse_sent('McGyver Industries stock declined.')[0].words()),
             ['LEFT-WALL', 'McGyver[!]', 'Industries[!]',
              'stock.n-u', 'declined.v-d', '.', 'RIGHT-WALL'])

        # King in dict as both upper and lower case.
        self.assertEqual(list(self.parse_sent('King Industries stock declined.')[0].words()),
             ['LEFT-WALL', 'King.b', 'Industries[!]',
              'stock.n-u', 'declined.v-d', '.', 'RIGHT-WALL'])

        # Jumbo in dict only lower-case, as adjective
        self.assertEqual(list(self.parse_sent('Jumbo Industries stock declined.')[0].words()),
             ['LEFT-WALL', 'Jumbo[!]', 'Industries[!]',
              'stock.n-u', 'declined.v-d', '.', 'RIGHT-WALL'])

        # Thomas in dict only as upper case.
        self.assertEqual(list(self.parse_sent('Thomas Industries stock declined.')[0].words()),
             ['LEFT-WALL', 'Thomas.b', 'Industries[!]',
              'stock.n-u', 'declined.v-d', '.', 'RIGHT-WALL'])

    # Some parses are fractionally preferred over others...
    def test_g_fractions(self):
        self.assertEqual(list(self.parse_sent('A player who is injured has to leave the field')[0].words()),
             ['LEFT-WALL', 'a', 'player.n', 'who', 'is.v', 'injured.a', 'has.v', 'to.r', 'leave.v', 'the', 'field.n', 'RIGHT-WALL'])

        self.assertEqual(list(self.parse_sent('They ate a special curry which was recommended by the restaurant\'s owner')[0].words()),
             ['LEFT-WALL', 'they', 'ate.v-d', 'a', 'special.a', 'curry.s',
              'which', 'was.v-d', 'recommended.v-d', 'by', 'the', 'restaurant.n',
              '\'s.p', 'owner.n', 'RIGHT-WALL'])

    # Verify that we are getting the linkages that we want
    # See below, remainder of parses are in text files
    def test_h_getting_links(self):
        sent = 'Scientists sometimes may repeat experiments or use groups.'
        linkage = self.parse_sent(sent)[0]
        self.assertEqual(linkage.diagram(),
"\n    +---------------------------------------Xp--------------------------------------+"
"\n    +---------------------------->WV---------------------------->+                  |"
"\n    |           +-----------------------Sp-----------------------+                  |"
"\n    |           |                  +------------VJlpi------------+                  |"
"\n    +-----Wd----+          +---E---+---I---+----Op----+          +VJrpi+---Op--+    |"
"\n    |           |          |       |       |          |          |     |       |    |"
"\nLEFT-WALL scientists.n sometimes may.v repeat.v experiments.n or.j-v use.v groups.n . "
"\n\n")
        sent = 'I enjoy eating bass.'
        linkage = self.parse_sent(sent)[0]
        self.assertEqual(linkage.diagram(),
"\n    +-----------------Xp----------------+"
"\n    +---->WV---->+                      |"
"\n    +--Wd--+-Sp*i+---Pg---+---Ou---+    |"
"\n    |      |     |        |        |    |"
"\nLEFT-WALL I.p enjoy.v eating.v bass.n-u . "
"\n\n")


        sent = 'We are from the planet Gorpon'
        linkage = self.parse_sent(sent)[0]
        self.assertEqual(linkage.diagram(),
"\n    +--->WV--->+     +---------Js--------+"
"\n    +--Wd--+Spx+--Pp-+   +--DD--+---GN---+"
"\n    |      |   |     |   |      |        |"
"\nLEFT-WALL we are.v from the planet.n Gorpon[!] "
"\n\n")


class ZENLangTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.d, cls.po = Dictionary(lang='en'), ParseOptions()

    @classmethod
    def tearDownClass(cls):
        del cls.d
        del cls.po

    def test_getting_links(self):
        linkage_testfile(self, self.d, self.po)

    def test_quotes(self):
        linkage_testfile(self, self.d, self.po, 'quotes')

    def test_null_link_range_starting_with_zero(self):
        """Test parsing with a minimal number of null-links, including 0."""
        # This sentence has no complete linkage. Validate that the library
        # doesn't mangle parsing with null-count>0 due to power_prune()'s
        # connector-discard optimization at null-count==0.  Without commit
        # "Allow calling classic_parse() with and w/o nulls", the number of
        # linkages here is 1 instead of 2 and the unused_word_cost is 5.
        self.po = ParseOptions(min_null_count=0, max_null_count=999)
        linkages = Sentence('about people attended', self.d, self.po).parse()
        self.assertEqual(len(linkages), 2)
        self.assertEqual(linkages.next().unused_word_cost(), 1)
        # Expected parses:
        # 1:
        #    +------------>WV------------>+
        #    +--------Wd-------+----Sp----+
        #    |                 |          |
        #LEFT-WALL [about] people.p attended.v-d
        # 2:
        #
        #            +----Sp----+
        #            |          |
        #[about] people.p attended.v-d


class ZENConstituentsCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.d, cls.po = Dictionary(lang='en'), ParseOptions()

    @classmethod
    def tearDownClass(cls):
        del cls.d
        del cls.po

    def test_a_constiuents_after_parse_list(self):
        """
        Validate that the post-processing data of the first linkage is not
        getting clobbered by later linkages.
        """
        linkages = list(Sentence("This is a test.", self.d, self.po).parse())
        self.assertEqual(linkages[0].constituent_tree(),
                "(S (NP this.p)\n   (VP is.v\n       (NP a test.n))\n   .)\n")

# Tests are run in alphabetical order; do the language tests last.
class ZDELangTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.d, cls.po = Dictionary(lang='de'), ParseOptions()

    @classmethod
    def tearDownClass(cls):
        del cls.d
        del cls.po

    def parse_sent(self, text):
        return list(Sentence(text, self.d, self.po).parse())

    def test_a_getting_num_of_words(self):
        #Words include punctuation and a 'LEFT-WALL' and 'RIGHT_WALL'
        self.assertEqual(self.parse_sent('Dies ist den Traum.')[0].num_of_words(), 7)
        self.assertEqual(self.parse_sent('Der Hund jagte ihn durch den Park.')[0].num_of_words(), 10)

    def test_b_getting_words(self):
        self.assertEqual(list(self.parse_sent('Der Hund jagte ihn durch den Park.')[0].words()),
            ['LEFT-WALL', 'der.d', 'Hund.n', 'jagte.s', 'ihn', 'durch',
               'den.d', 'Park.n', '.', 'RIGHT-WALL'])

    def test_c_getting_links(self):
        sent = 'Dies ist den Traum.'
        linkage = self.parse_sent(sent)[0]
        self.assertEqual(linkage.link(0),
                         Link(linkage, 0, 'LEFT-WALL','Xp','Xp','.'))
        self.assertEqual(linkage.link(1),
                         Link(linkage, 1, 'LEFT-WALL','W','W','ist.v'))
        self.assertEqual(linkage.link(2),
                         Link(linkage, 2, 'dies','Ss','Ss','ist.v'))
        self.assertEqual(linkage.link(3),
                         Link(linkage, 3, 'ist.v','O','O','Traum.n'))
        self.assertEqual(linkage.link(4),
                         Link(linkage, 4, 'den.d','Dam','Dam','Traum.n'))
        self.assertEqual(linkage.link(5),
                         Link(linkage, 5, '.','RW','RW','RIGHT-WALL'))

class ZLTLangTestCase(unittest.TestCase):
    def setUp(self):
        self.d, self.po = Dictionary(lang='lt'), ParseOptions()

    # Reads linkages from a test-file.
    def test_getting_links(self):
        linkage_testfile(self, self.d, self.po)

# Tests are run in alphabetical order; do the language tests last.
class ZRULangTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.d, cls.po = Dictionary(lang='ru'), ParseOptions()

    @classmethod
    def tearDownClass(cls):
        del cls.d
        del cls.po

    def parse_sent(self, text):
        return list(Sentence(text, self.d, self.po).parse())

    def test_a_getting_num_of_words(self):
        #Words include punctuation and a 'LEFT-WALL' and 'RIGHT_WALL'
        self.assertEqual(self.parse_sent('это тести.')[0].num_of_words(), 5)
        self.assertEqual(self.parse_sent('вверху плыли редкие облачка.')[0].num_of_words(), 7)

    def test_b_getting_words(self):
        self.assertEqual(list(self.parse_sent('вверху плыли редкие облачка.')[0].words()),
            ['LEFT-WALL', 'вверху.e', 'плыли.vnndpp', 'редкие.api',
                'облачка.ndnpi', '.', 'RIGHT-WALL'])

    def test_c_getting_links(self):
        sent = 'вверху плыли редкие облачка.'
        linkage = self.parse_sent(sent)[0]
        self.assertEqual(linkage.link(0),
                         Link(linkage, 0, 'LEFT-WALL','Xp','Xp','.'))
        self.assertEqual(linkage.link(1),
                         Link(linkage, 1, 'LEFT-WALL','W','Wd','плыли.vnndpp'))
        self.assertEqual(linkage.link(2),
                         Link(linkage, 2, 'вверху.e','EI','EI','плыли.vnndpp'))
        self.assertEqual(linkage.link(3),
                         Link(linkage, 3, 'плыли.vnndpp','SIp','SIp','облачка.ndnpi'))
        self.assertEqual(linkage.link(4),
                         Link(linkage, 4, 'редкие.api','Api','Api','облачка.ndnpi'))
        self.assertEqual(linkage.link(5),
                         Link(linkage, 5, '.','RW','RW','RIGHT-WALL'))

    # Expect morphological splitting to apply.
    def test_d_morphology(self):
        self.po.display_morphology = True
        self.assertEqual(list(self.parse_sent('вверху плыли редкие облачка.')[0].words()),
            ['LEFT-WALL',
             'вверху.e',
             'плы.=', '=ли.vnndpp',
             'ре.=', '=дкие.api',
             'облачк.=', '=а.ndnpi',
             '.', 'RIGHT-WALL'])

def linkage_testfile(self, lgdict, popt, desc = ''):
    """
    Reads sentences and their corresponding
    linkage diagrams / constituent printings.
    """
    self.__class__.longMessage = True
    if '' != desc:
        desc = desc + '-'
    testfile = clg.test_data_srcdir + "parses-" + desc + clg.dictionary_get_lang(lgdict._obj) + ".txt"
    parses = open(testfile, "rb")
    diagram = None
    sent = None
    lineno = 0
    opcode_detected = 0 # function sanity check
    for line in parses:
        lineno += 1
        if sys.version_info > (3, 0):
            line = line.decode('utf-8')
        # Lines starting with I are the input sentences
        if 'I' == line[0]:
            opcode_detected += 1
            sent = line[1:]
            diagram = ""
            constituents = ""
            linkages = Sentence(sent, lgdict, popt).parse()
            linkage = next(linkages, None)
            self.assertTrue(linkage, "at {}:{}: Sentence has no linkages".format(testfile, lineno))

        # Generate the next linkage of the last input sentence
        if 'N' == line[0]:
            opcode_detected += 1
            diagram = ""
            constituents = ""
            linkage = next(linkages, None)
            self.assertTrue(linkage, "at {}:{}: Sentence has too few linkages".format(testfile, lineno))

        # Lines starting with O are the parse diagram
        # It ends with an empty line
        if 'O' == line[0]:
            opcode_detected += 1
            diagram += line[1:]
            if '\n' == line[1] and 1 < len(diagram):
                self.assertEqual(linkage.diagram(), diagram, "at {}:{}".format(testfile, lineno))

        # Lines starting with C are the constituent output (type 1)
        # It ends with an empty line
        if 'C' == line[0]:
            opcode_detected += 1
            if '\n' == line[1] and 1 < len(constituents):
                self.assertEqual(linkage.constituent_tree(), constituents, "at {}:{}".format(testfile, lineno))
            constituents += line[1:]
    parses.close()

    self.assertGreaterEqual(opcode_detected, 2, "Nothing has been done for " + testfile)

def warning(*msg):
    progname = os.path.basename(sys.argv[0])
    print("{}: Warning:".format(progname), *msg, file=sys.stderr)


# Decorate Sentence.parse with eqcost_soretd_parse.
lg_testutils.add_eqcost_linkage_order(Sentence)

unittest.main()
