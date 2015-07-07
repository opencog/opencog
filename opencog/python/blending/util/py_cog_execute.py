import opencog.atomspace
from opencog.scheme_wrapper import *
from blending.util.general_util import Singleton

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
            cls.load_scheme()
            cls.is_initialized = True

    def load_scheme(cls):
        # TODO: FIXME: Hacky method to find scheme framework..
        # 1. Find by current loaded atomspace library's file path
        atomspace_lib_file = opencog.atomspace.__file__
        path_of_opencog_lib = '/'.join(atomspace_lib_file.split('/')[0:-3])
        path_of_scm_lib = path_of_opencog_lib + '/scm'

        scheme_eval(
            cls.a,
            '(add-to-load-path "' + path_of_scm_lib + '")' +
            '(add-to-load-path "' + path_of_scm_lib + "/opencog" + '")'
        )

        # 2. Find by current python file's path
        current_file = __file__
        path_of_opencog_lib = '/'.join(current_file.split('/')[0:-5])
        scheme_eval(
            cls.a,
            '(add-to-load-path "' + path_of_opencog_lib + '")' +
            '(add-to-load-path "' + path_of_opencog_lib +
            "/opencog/scm/opencog" + '")'
        )

        # 3. Find by common uses path
        scheme_eval(
            cls.a,
            '(add-to-load-path "/usr/local/share/opencog/scm")'
        )

        scheme_eval(
            cls.a,
            '(use-modules (opencog))' +
            '(use-modules (opencog query))' +
            '(use-modules (opencog exec))' +
            '(use-modules (opencog rule-engine))'
        )

    def execute(cls, a, execute_link):
        try:
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
        except RuntimeError as e:
            raise RuntimeError(
                str(e) +
                "\n\n" +
                "Can't load scheme's cog-execute function.\n" +
                "Make sure the you had installed atomspace "
                "to /usr/local/share/opencog."
            )
