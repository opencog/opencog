from itertools import chain
from spatiotemporal.temporal_events.composition.emperical_distribution import overlaps
from spatiotemporal.temporal_events.util import Dijkstra
from utility.functions import almost_equals

__author__ = 'keyvan'

EPSILON = 1e-12


class Wagon(object):
    def __init__(self, a, b):
        self._a = float(a)
        self.length = float(b - a)
        self.bound_wagons = set()
        self._parent = None
        self.heads = []
        self.tails = []
        self.is_moving = False

    @property
    def parent(self):
        return self._parent

    @property
    def is_root(self):
        return self._parent is None

    @property
    def a(self):
        return self._a
    
    @property
    def b(self):
        return self._a + self.length

    @property
    def root(self):
        wagon = self
        while not wagon.is_root:
            # print wagon.name
            wagon = wagon._parent
        return wagon

    @property
    def bounds(self):
        return self.a, self.b
    
    def bind(self, other):
        root = self.root
        root_other = other.root
        if root_other is root:
            return
        root.bound_wagons.add(root_other)
        for wagon in root_other.bound_wagons:
            root.bound_wagons.add(wagon)
            wagon._parent = root
        root_other.bound_wagons = set()
        root_other._parent = root

    def move_to(self, a, b=None):
        if b is None:
            b = a + self.length
        root = self.root
        scale = (b - a) / self.length
        bias = a - scale * self.a
        if not (scale == 1.0 and bias == 0.0):
            root.transform(scale, bias)

    def find_move_cover_areas(self, a, b):
        return (min(self.b, b), max(self.b, b)), (min(self.a, a), max(self.a, a))

    def _move_heads_and_tails(self, a, b):
        head_bounds, tail_bounds = self.find_move_cover_areas(a, b)

        for head in self.heads:
            if overlaps(head.bounds, head_bounds):
                head.move_to(b, b + head.length)
        for tail in self.tails:
            if overlaps(tail.bounds, tail_bounds):
                tail.move_to(a - tail.length, a)

    def _move_to(self, a, b):
        self._move_heads_and_tails(a, b)
        self._a, self.length = a, b - a

    def transform(self, scale, bias):
        if self.is_moving:
            return
        self.is_moving = True
        transform_start = self.a * scale + bias
        transform_end = self.b * scale + bias

        for wagon in self.bound_wagons:
            wagon_a = wagon.a * scale + bias
            wagon_b = wagon.b * scale + bias
            wagon._move_to(wagon_a, wagon_b)

        self._move_heads_and_tails(transform_start, transform_end)

        self._a, self.length = transform_start, transform_end - transform_start

        self.is_moving = False

    def __str__(self, indent=0):
        children = indent * ' ' + repr(self) + '\n'
        for wagon in self.bound_wagons:
            children += (indent + 1) * ' ' + wagon.__str__(indent + 1)
        return children

    def __repr__(self):
        return 'Wagon(a: {0}, b: {1})'.format(self.a, self.b)


class Rail(list):
    def __init__(self, iterable=None):
        wagons = None
        if iterable is not None:
            for wagon in iterable:
                if wagons is None:
                    wagons = [wagon]
                    previous_wagon = wagon
                    continue
                previous_wagon.heads.append(wagon)
                wagons.append(wagon)
                wagon.tails.append(previous_wagon)
                previous_wagon = wagon
        list.__init__(self, wagons)

    def append(self, wagon):
        list.append(self, wagon)
        wagon.tails.append(list.__getitem__(self, -2))
        list.__getitem__(self, -2).heads.append(wagon)

    def insert(self, index, wagon):
        if index < 0:
            raise NotImplementedError
        list.insert(self, index, wagon)
        if index > 0:
            wagon.tails.append(list.__getitem__(self, index - 1))
            list.__getitem__(self, index - 1).heads.append(wagon)
            if index + 1 < len(self):
                list.__getitem__(self, index - 1).heads.remove(list.__getitem__(self, index + 1))

        if index < len(self):
            wagon.heads.append(list.__getitem__(self, index + 1))
            list.__getitem__(self, index + 1).tails.append(wagon)
            if index - 1 >= 0:
                list.__getitem__(self, index + 1).tails.remove(list.__getitem__(self, index - 1))


class RailwaySystem(object):
    def __init__(self):
        self.rails = {}
        self.memo = []
        self.dag = {}

    def add_rail(self, rail_key):
        self.memo.append(('add_rail', [rail_key]))
        wagon_1 = Wagon(0, 10)
        wagon_2 = Wagon(10, 20)
        wagon_1.name = rail_key + '0'
        wagon_2.name = rail_key + '1'
        self.rails[rail_key] = Rail([wagon_1, wagon_2])
        self.dag[wagon_1] = {wagon_2: 1}
        self.dag[wagon_2] = {}

    def move_wagon(self, rail_key, wagon_index, a, b):
        self.memo.append(('move_wagon', (rail_key, wagon_index, a, b)))
        self.rails[rail_key][wagon_index].move_to(a, b)

    def bind_wagons_before_horizontal(self, rail_1_key, wagon_1_index, rail_2_key, wagon_2_index):
        self.memo.append(('bind_wagons_before_horizontal', (rail_1_key, wagon_1_index, rail_2_key, wagon_2_index)))
        wagon_1 = self.rails[rail_1_key][wagon_1_index]
        wagon_2 = self.rails[rail_2_key][wagon_2_index]

        if wagon_1.b > wagon_2.a:
            wagon_2.move_to(wagon_1.b)

        wagon_2.tails.append(wagon_1)
        wagon_1.heads.append(wagon_2)

        self.dag[wagon_1][wagon_2] = 1

    def bind_wagons_after_horizontal(self, rail_1_key, wagon_1_index, rail_2_key, wagon_2_index):
        self.bind_wagons_before_horizontal(rail_2_key, wagon_2_index, rail_1_key, wagon_1_index)

    def bind_wagons_vertical(self, rail_1_key, wagon_1_index, rail_2_key, wagon_2_index):
        wagon_1 = self.rails[rail_1_key][wagon_1_index]
        wagon_2 = self.rails[rail_2_key][wagon_2_index]
        self.memo.append(('bind_wagons_vertical', (rail_1_key, wagon_1_index, rail_2_key, wagon_2_index)))
        wagon_2.bind(wagon_1)

        equals_a = almost_equals(wagon_1.a, wagon_2.a, EPSILON)
        equals_b = almost_equals(wagon_1.b, wagon_2.b, EPSILON)
        if (wagon_1.a < wagon_2.a or equals_a) and (wagon_1.b < wagon_2.b or equals_b):
            self.dag[wagon_1][wagon_2] = 1

        if (wagon_1.a > wagon_2.a or equals_a) and (wagon_1.b > wagon_2.b or equals_b):
            self.dag[wagon_2][wagon_1] = 1

    def move_and_bind_vertical(self, rail_1_key, wagon_1_index, rail_2_key, wagon_2_index, a, b):
        self.move_wagon(rail_1_key, wagon_1_index, a, b)
        self.bind_wagons_vertical(rail_1_key, wagon_1_index, rail_2_key, wagon_2_index)

    def are_in_same_vertical_tree(self, wagon_1, wagon_2):
        return wagon_1.root is wagon_2.root

    def are_in_same_horizontal_tree(self, wagon_1, wagon_2):
        start = wagon_1
        end = wagon_2
        dijkstra_1 = Dijkstra(self.dag, start, end)
        dijkstra_2 = Dijkstra(self.dag, end, start)
        return end in dijkstra_1[0] or start in dijkstra_2[0]

    def compress(self):
        for rail_key, rail in self.rails.items():
            wagon_tail = rail[0]
            wagon_head = rail[-1]
            self.move_wagon(rail_key, 0, wagon_head.a - wagon_tail.length, wagon_head.a)

    def __getitem__(self, rail_key):
        return self.rails[rail_key]

    def __str__(self):
        result = ''
        for key, rail in self.rails.items():
            result += str(key) + ': ' + str(rail) + '\n'
        return result

    def __deepcopy__(self, memo):
        copy = RailwaySystem()
        for action in self.memo:
            copy.__getattribute__(action[0])(*action[1])
        return copy

    def __repr__(self):
        return str(self)

    def __iter__(self):
        return chain(*self.rails.values())
