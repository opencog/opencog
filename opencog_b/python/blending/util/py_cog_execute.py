from os.path import expanduser

from opencog.scheme_wrapper import *
from opencog_b.python.blending.util.general_util import Singleton

__author__ = 'DongMin Kim'


# noinspection PyTypeChecker
class PyCogExecute(Singleton):
    def __init__(cls):
        # noinspection PyArgumentList
        super(PyCogExecute, cls).__init__()
        cls.a = None
        cls.is_initialized = False

    def __initialize(cls, a):
        if cls.a is not a:
            cls.is_initialized = False
            cls.a = a

        if cls.is_initialized is not True:
            cls.is_initialized = True

            # TODO: How to find the scheme module in beautiful?
            scheme_eval(
                cls.a,
                '(add-to-load-path "' + expanduser("~/atomspace") + '")' +
                '(add-to-load-path "' + expanduser("~/atomspace/build") + '")' +
                '(add-to-load-path "' + expanduser(
                    "~/atomspace/opencog/scm") + '")' +
                '(add-to-load-path "' + expanduser("~/opencog") + '")' +
                '(add-to-load-path "' + expanduser("~/opencog/build") + '")' +
                '(add-to-load-path "' + expanduser(
                    "~/opencog/opencog/scm") + '")' +
                '(add-to-load-path ".")'
            )

            scheme_eval(
                cls.a,
                '(use-modules (opencog))' +
                '(use-modules (opencog query))' +
                '(use-modules (opencog exec))' +
                '(use-modules (opencog rule-engine))' +
                '(load-from-path "utilities.scm")'
            )

    def execute(cls, a, execute_link):
        cls.__initialize(a)

        result_set_uuid = scheme_eval_h(
            cls.a,
            '(cog-execute! ' +
            '  (cog-atom ' + str(execute_link.handle_uuid()) + ')' +
            ')'
        )

        if cls.a.is_valid(result_set_uuid):
            return cls.a[result_set_uuid]
        else:
            return None
