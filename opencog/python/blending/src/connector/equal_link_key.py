from opencog.atomspace import Handle

__author__ = 'DongMin Kim'


class EqualLinkKey:
    """Defines the custom unique key of link to evaluate whether links are
    duplicate or not.

    Links are treated as duplicated link if they have same properties below:
    1. link type,
    2. outgoing list(and its order),
    3. position of Node which requested to making this key.
    -> In source list, EqualLinkKey doesn't save the destination atom's Handle
    but saves only position in source list.

    In other words, EqualLinkKey is abbreviation version of Link, to help
    blend algorithm in concisely and boost speed of algorithm.

    Attributes:
        h: A Handle of original link.
        t: A type of original link.
        tv: A TruthValue of original link.
        src_h_list_str: A serialized outgoing list in original link.
        dst_pos_in_outgoing: A Node position which was request making this key.
        :type h: Handle
        :type t: int
        :type tv: TruthValue
        :type src_h_list_str: str
        :type dst_pos_in_outgoing: int
    """

    def __init__(self, link_h=None, link_type=None, truth_value=None):
        self.h = link_h
        self.t = link_type
        self.tv = truth_value
        self.src_h_list_str = ''
        self.dst_pos_in_outgoing = -1

    def __key(self):
        """Defines a property of sameness.

        It is equal if
        (link type, outgoing list, self position in outgoing list) is same.
        Returns:
            :rtype : int, str, int
        """
        return self.t, self.src_h_list_str, self.dst_pos_in_outgoing

    def __eq__(self, other):
        """Provides __eq__ function to Python."""
        if self.__key() == other.__key():
            return True

        return False

    def __hash__(self):
        """Provides __hash__ function to Python."""
        return hash(self.__key())

    def add_src_h(self, src_node):
        """Add a source node to EqualLinkKey's source node list string.

        EqualLinkKey manages source list as string. So it converts the source
        node to string of Atom's Handle.

        Args:
            src_node: An source node.
            :param src_node: Atom
        """
        src_node_h_str = str(src_node.h.value())
        self.src_h_list_str += src_node_h_str + ', '

    def get_src_list(self, a):
        """Get a source node list in EqualLinkKey.

        It converts source node string to list of Atom.

        Args:
            a: An instance of AtomSpace to find Atom.
            :param a: AtomSpace
        Returns:
            The atoms that saved in EqualLink.
            :rtype : list[Atom]
        """
        ret = []

        src_node_h_int_list = self.src_h_list_str.split(', ')
        # Delete end of string
        src_node_h_int_list.pop()

        for src_node_h_int in src_node_h_int_list:
            ret.append(a[Handle(int(src_node_h_int))])

        return ret


def link_to_keys(a, original_links, dst_atom):
    """Convert the original link list to EqualLinkKey list.

    It receives a destination atom in outgoing of link. Because the links are
    treated as duplicated link if they have some of sameness but only different
    in destination atom.

    Args:
        a: An instance of AtomSpace to find Atom.
        original_links: The original link list to be converted to EqualLinkKeys.
        dst_atom: A destination atom in outgoing of link.
        :param a: AtomSpace
        :param original_links: list[Atom]
        :param dst_atom: Atom
    Returns:
        An EqualLink list.
        :rtype : list[EqualLink]
    """
    # TODO: How to check and merge link which has ingoing atoms?
    ret = list()
    for link in original_links:
        equal_link_key = EqualLinkKey(link.h, link.t, link.tv)

        xget_link_src = a.xget_outgoing(link.h)
        for i, link_src in enumerate(xget_link_src):
            if link_src.h == dst_atom.h:
                equal_link_key.dst_pos_in_outgoing = i
            else:
                equal_link_key.add_src_h(link_src)

        ret.append(equal_link_key)

    return ret


def key_to_link(a, link_key, dst_atom, tv=None):
    """Convert an EqualLinkKey to original link.

    It receives a destination atom in outgoing of link. Because an EqualLinkKey
    doesn't know what is destination atom(it is just key) so we have to define
    the destination atom in EqualLinkKey's source atom list.

    Args:
        a: An instance of AtomSpace to find Atom.
        link_key: An EqualLinkKey to be converted to link.
        dst_atom: A destination atom in outgoing of link.
        tv: A TruthValue to be saved in new link.
        :param a: AtomSpace
        :param link_key: EqualLinkKey
        :param dst_atom: Atom
        :param tv: TruthValue
    Returns:
        :rtype: Link
    """
    tv = link_key.tv if tv is None else tv
    src_list = link_key.get_src_list(a)

    # Blender requests converting a link with destination atom,
    # but sometimes EqualLink has pointing to destination atom itself.
    # In this case, this method doesn't make new link to avoid self-pointing.
    if dst_atom in src_list:
        return

    src_list.insert(link_key.dst_pos_in_outgoing, dst_atom)

    return a.add_link(link_key.t, src_list, tv)
