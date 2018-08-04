from blending.src.decider.base_decider import BaseDecider
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class DecideBestSTI(BaseDecider):
    """Blending decider that deciding to blend or not by checking the existence
    of atoms within proper STI range.

    This decider estimates the chosen atoms are worth, when they have the
    STI value higher than given.
    """

    def __init__(self, a):
        super(self.__class__, self).__init__(a)

    def make_default_config(self):
        """Initialize a default config for this class."""
        super(self.__class__, self).make_default_config()
        BlendConfig().update(self.a, "decide-sti-min", "1")
        BlendConfig().update(self.a, "decide-sti-max", "None")

    def __decide_atoms_best_sti(
            self, chosen_atoms, result_atoms_count, sti_min, sti_max
    ):
        """Actual algorithm for deciding blend.

        Args:
            chosen_atoms: The atoms to decide.
            result_atoms_count: Threshold value for minimum count of decided
            atoms.
            sti_min: Threshold value for minimum value of STI.
            sti_max: Threshold value for maximum value of STI.
            :param chosen_atoms: list[Atom]
            :param result_atoms_count: int
            :param sti_min: int
            :param sti_max: int
        """
        # Filter the atoms with specified STI range.
        if sti_max is None:
            self.ret = filter(
                lambda atom:
                sti_min <= atom.av["sti"],
                chosen_atoms
            )
        else:
            self.ret = filter(
                lambda atom:
                sti_min <= atom.av["sti"] < sti_max,
                chosen_atoms
            )

        if len(self.ret) < result_atoms_count:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            return

        # Find the atom has biggest STI value.
        self.ret = sorted(
            self.ret,
            key=lambda x: x.av['sti'],
            reverse=True
        )
        self.ret = self.ret[0:result_atoms_count]

    def blending_decide_impl(self, chosen_atoms, config_base):
        """Implemented factory method to deciding atoms.

        Args:
            chosen_atoms: The atoms to decide.
            config_base: A Node to save custom config.
            :param chosen_atoms: list[Atom]
            :param config_base: Atom
        """
        result_atoms_count = BlendConfig().get_int(
            self.a, "decide-result-atoms-count", config_base
        )
        sti_min = BlendConfig().get_str(self.a, "decide-sti-min", config_base)
        sti_max = BlendConfig().get_str(self.a, "decide-sti-max", config_base)

        # Check if given range of STI value is valid or not.
        sti_min = int(sti_min) if sti_min.isdigit() else 1
        sti_max = int(sti_max) if sti_max.isdigit() else None

        self.__decide_atoms_best_sti(
            chosen_atoms, result_atoms_count, sti_min, sti_max
        )
