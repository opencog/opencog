from blending.src.maker.base_maker import BaseMaker
from blending.src.connector.connect_util import *
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class MakeSimple(BaseMaker):
    """Atom maker that making new simple atom.

    This maker will make a new simple atom by merging the given decided atoms.
    """

    def __init__(self, a):
        super(self.__class__, self).__init__(a)

    def make_default_config(self):
        """Initialize a default config for this class."""
        super(self.__class__, self).make_default_config()

    def __make_atom_from_all(
            self, decided_atoms, atom_prefix, atom_separator, atom_postfix
    ):
        """Actual algorithm for making new atom.

        Args:
            decided_atoms: The source atoms to make new atom.
            atom_prefix: A prefix string for new atom.
            atom_separator: A separator string for new atom.
            atom_postfix: A postfix string for new atom.
            :param decided_atoms: list[Atom]
            :param atom_prefix: string
            :param atom_separator: string
            :param atom_postfix: string
        """
        if len(decided_atoms) < 2:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            return

        # Prefix
        new_blend_atom_name = atom_prefix

        # Make a name for new atom with separator.
        for atom in decided_atoms:
            new_blend_atom_name += str(atom.name)
            new_blend_atom_name += atom_separator

        # Delete the last separator string.
        new_blend_atom_name = new_blend_atom_name[0:-1]

        # Postfix
        new_blend_atom_name += atom_postfix

        # Make new atom with new name.
        self.ret = self.a.add_node(
            types.ConceptNode,
            new_blend_atom_name,
            get_weighted_tv(decided_atoms)
        )

        # TODO: Give proper attention value.
        # new_blend_atom_name.av = {}

    def new_blend_make_impl(self, decided_atoms, config_base):
        """Implemented factory method to making new atom.

        Args:
            decided_atoms: The source atoms to make new atom.
            config_base: A Node to save custom config.
            :param decided_atoms: list[Atom]
            :param config_base: Atom
        """
        atom_prefix = BlendConfig().get_str(
            self.a, "make-atom-prefix", config_base
        )
        atom_separator = BlendConfig().get_str(
            self.a, "make-atom-separator", config_base
        )
        atom_postfix = BlendConfig().get_str(
            self.a, "make-atom-postfix", config_base
        )

        self.__make_atom_from_all(
            decided_atoms, atom_prefix, atom_separator, atom_postfix
        )
