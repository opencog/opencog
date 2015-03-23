__author__ = 'keyvan'

from globals import MINUS_INFINITY, PLUS_INFINITY


class Discretiser(object):

    def __init__(self, lower_bound=MINUS_INFINITY, upper_bound=PLUS_INFINITY):
        self.lower_bound, self.upper_bound = lower_bound, upper_bound
        self.thresholds = [lower_bound,upper_bound]
        self.quantity_per_class = []

    def new_data(self, value):
        if len(self.thresholds) == 2:
            if value in (self.lower_bound, self.upper_bound):
                return
            self.thresholds.insert(1, value)
            self.quantity_per_class.append(1)
            return
        print 'kir'

do = Discretiser(0,10)
do.new_data(2)
print do.thresholds