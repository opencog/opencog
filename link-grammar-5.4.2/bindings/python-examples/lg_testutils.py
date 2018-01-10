from operator import methodcaller # Used in sorted()

def add_eqcost_linkage_order(original_class):
    """
    Decorate the parse method of class Sentence (to be given as argument)
    with a new parse function, defined below, so equal-cost linkages will
    be in a deterministic order.
    Usage: lg_testutils.add_eqcost_linkage_order(Sentence)
    """
    class eqcost_soretd_parse(original_class.sentence_parse):
        """
        Sort equal-cost linkages according to the alphabetic order of their
        diagram string, on demand.  We need it because the order of linkages
        with equal costs is not guaranteed and it may get changed due to subtle
        changes in the parsing code and/or in the qsort() library function.
        """
        def __init__(self, linkages):
            self.linkages = linkages
            self.sent = linkages.sent
            self.rc = linkages.rc
            self.linkage_list = [] # List of equal-cost linkages
            self.num = 0
            self.cost = None
            self.saved_next = None

        def __iter__(self):
            return self

        def next(self):
            # If the list of equal-cost linkages has exhausted, fetch a new one.
            if self.num >= len(self.linkage_list)-1:
                if self.linkage_list and not self.saved_next:
                    raise StopIteration()
                self.linkage_list = []
                self.cost = None
                while True:
                    if self.saved_next:
                        linkage = self.saved_next
                        self.saved_next = None
                    else:
                        try:
                            linkage = self.linkages.next()
                        except StopIteration:
                            break
                    if not self.sent.parse_options.use_sat:
                        cost = [linkage.unused_word_cost(),
                                linkage.disjunct_cost(),
                                linkage.link_cost()]
                        if not self.cost:
                            self.cost = cost
                        else:
                            if self.cost != cost:
                                self.saved_next = linkage
                                break
                    self.linkage_list.append(linkage)

                if not self.linkage_list:
                    raise StopIteration()
                # We have here a new list of equal-cost linkages.
                self.num = -1
                self.linkage_list.sort(key=methodcaller('diagram', screen_width=9999))

            # Return the next linkage from the sorted list of equal-cost linkages.
            self.num += 1
            return self.linkage_list[self.num]

        __next__ = next      # Account python3

    original_class.original_parse = original_class.parse

    def parse(self):
        """A decoration for the original Sentence.parse"""
        linkages = self.original_parse()
        return eqcost_soretd_parse(linkages)

    original_class.parse = parse
