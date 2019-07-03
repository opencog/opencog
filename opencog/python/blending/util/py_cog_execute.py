import opencog.atomspace
from opencog.scheme_wrapper import *

from blending.util.blending_util import Singleton

__author__ = 'DongMin Kim'


# noinspection PyTypeChecker
class PyCogExecute(Singleton):
    """Wrap the scheme's (cog-execute! ) function.

    Since there not exists python binding of cog-execute function, it calls
    to scheme directly.

    Attributes:
        a: An instance of AtomSpace.
        is_initialized: Displays whether wrapper was initialized or not.
        :type a: AtomSpace
        :type is_initialized: bool
    """

    # noinspection PyArgumentList
    def __init__(cls):
        super(PyCogExecute, cls).__init__()
        cls.a = None
        cls.is_initialized = False

    def __initialize(cls, a):
        """Initializes a singleton object for specified AtomSpace.

        If the given AtomSpace is different from AtomSpace in class, then
        it starts re-initialize.

        Args:
            a: An instance of AtomSpace to find Atom.
            :param a: AtomSpace
        """
        if cls.a is not a:
            cls.is_initialized = False
            cls.a = a

        if cls.is_initialized is True:
            return

        cls.load_scheme()
        cls.is_initialized = True

    def load_scheme(cls):
        """Load scheme framework to use (cog-execute! ) function.

        TODO: FIXME: Hacky method to find scheme framework!
        If someone uses OpenCog only for Python, maybe he had not make the
        environment to use scheme environment, so this method try to load it
        manually. When the OpenCog framework(with Scheme) stabilized, this
        hacky method should be wipe out from OpenCog.
        """
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
            '(use-modules (opencog exec))' +
            '(use-modules (opencog ure))'
        )

    def execute(cls, a, execute_link):
        """Execute the given link and return an Atom.

        Args:
            a: An instance of AtomSpace.
            execute_link: A link to execute.
            :param a: AtomSpace
            :param execute_link: Link
        Returns:
            A result of executing the link.
            :rtype: Atom
        Raises:
            RuntimeError: An error occurred in loading scheme environment.
        """
        try:
            cls.__initialize(a)

            # (cog-execute! (cog-atom <UUID: xxx>))
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
