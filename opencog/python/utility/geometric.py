from math import fabs
from utility.numeric.globals import MINUS_INFINITY, PLUS_INFINITY, EPSILON
from scipy.integrate import quad

__author__ = 'keyvan'


def equals(a, b):
    if fabs(a - b) < EPSILON:
        return True
    return False


def index_of_first_local_maximum(sequence):
        first_time = True
        index = 0
        for element in sequence:
            if first_time:
                previous = element
                first_time = False
                continue
            if element <= previous:
                return index
            previous = element
            index += 1
        return None


class Function(object):
    def __call__(self, x):
        return 0

    def integrate(self, start, end):
        assert end >= start, "'stop' should be greater than 'end'"
        area, error = quad(self.__call__, start, end)
        return area

    def derivative(self, point):
        return None


class _function_undefined(Function):
    def __call__(self, x):
        return None

    def integrate(self, start, end):
        return None


FUNCTION_UNDEFINED = _function_undefined()


class FunctionHorizontalLinear(Function):
    def __init__(self, y_intercept):
        self.y_intercept = y_intercept
        self.a = 0

    def __call__(self, x):
        return self.y_intercept

    def integrate(self, start, end):
        assert end >= start, "'stop' should be greater than 'end'"
        return float(self.y_intercept) * (end - start)

    def derivative(self, point):
        return 0

    @property
    def b(self):
        return self.y_intercept


class FunctionLinear(Function):
    def __init__(self, a=None, b=None, x_0=None, y_0=None, x_1=None, y_1=None):
        #(x_0, y_0), (x_1, y_1) = sorted([(x_0, y_0), (x_1, y_1)])
        if (a, b) == (None, None):
            a = (float(y_1) - y_0) / (x_1 - x_0)
            b = y_0 - a * x_0
        self.a = a
        self.b = b

    def __call__(self, x):
        return float(self.a * x + self.b)

    def intersect(self, other):
        x = float(other.b) - self.b / self.a - other.a
        return x, self(x)

    def integrate(self, start, end):
        assert end >= start, "'stop' should be greater than 'end'"
        if self.a == 0:
            return self.b * (end - start)

        x_intercept = self.x_intercept

        if start > x_intercept or equals(start, x_intercept):
            area_of_triangle = (end - start) * (self(end) - self(start)) / 2
            area_of_rectangle = (end - start) * self(start)
            return area_of_triangle + area_of_rectangle

        if end < x_intercept or equals(end, x_intercept):
            area_of_triangle = (end - start) * (self(start) - self(end)) / 2
            area_of_rectangle = (end - start) * self(end)
            return area_of_triangle + area_of_rectangle

        minus_triangle = (x_intercept - start) * self(start)
        plus_triangle = (end - x_intercept) * self(end)
        return minus_triangle + plus_triangle

    def derivative(self, point):
        return self.a

    @property
    def x_intercept(self):
        return - float(self.b) / self.a

    @property
    def y_intercept(self):
        return self(0)


class FunctionComposite(Function):
    _function_undefined = FUNCTION_UNDEFINED

    def __init__(self, bounds_function_dictionary, function_undefined=None):
        assert isinstance(bounds_function_dictionary, dict)
        if function_undefined is not None:
            self.function_undefined = function_undefined
        for bounds in bounds_function_dictionary:
            assert isinstance(bounds, (tuple, list)) and len(bounds) is 2
            a, b = bounds
            assert isinstance(a, (float, int, long)) and isinstance(b, (float, int, long))
            assert isinstance(bounds_function_dictionary[bounds], Function)
        self.functions = bounds_function_dictionary

    def __call__(self, x):
        for function_bounds in self.functions:
            (a, b) = function_bounds
            if a <= x:
                if b >= x:
                    return self.functions[function_bounds](x)
        return self.function_undefined(x)

    def integrate(self, start, end):
        assert end >= start, "'stop' should be greater than 'end'"
        result = 0
        for function_bounds in self.functions:
            (a, b) = function_bounds
            if a <= start:
                if b >= end:
                    return self.functions[function_bounds].integrate(start, end)
            not_ordered = {
                (start, 0): 's', (end, 0): 'e',
                (a, 1): 'a', (b, 1): 'b'
            }
            order = ''.join([not_ordered[i] for i in sorted(not_ordered)])
            if (a == start or a == end) and order == 'saeb' or (b == start or b == end) and order == 'asbe':
                continue
            if order in 'seab abse':
                continue
            if order == 'saeb':
                b = end
            elif order == 'asbe':
                a = start
            result += self.functions[function_bounds].integrate(a, b)
        return result

    def find_bounds_for(self, point):
        for bounds in self.functions:
            (a, b) = bounds
            if a <= point and b >= point:
                return bounds

    def derivative(self, point):
        return self.functions[self.find_bounds_for(point)].derivative(point)

    def plot(self, x_datetime=False):
        import matplotlib.pyplot as plt
        from spatiotemporal.unix_time import UnixTime

        x = []
        y = []
        for bounds in sorted(self.functions):
            (a, b) = bounds
            if a not in [PLUS_INFINITY, MINUS_INFINITY] and b not in [PLUS_INFINITY, MINUS_INFINITY]:
                y.append(self.functions[bounds](a))
                y.append(self.functions[bounds](b))
                if x_datetime:
                    a, b = UnixTime(a).to_datetime(), UnixTime(b).to_datetime()
                x.append(a)
                x.append(b)
        plt.plot(x, y)
        return plt

    @property
    def function_undefined(self):
        return self._function_undefined

    @function_undefined.setter
    def function_undefined(self, value):
        assert isinstance(value, Function)
        self._function_undefined = value


class FunctionPiecewiseLinear(FunctionComposite):
    def __init__(self, input_list, output_list, function_undefined=None):
        if function_undefined is not None:
            self.function_undefined = function_undefined
        self.input_list = input_list
        self.output_list = output_list
        self.invalidate()

    def invalidate(self):
        assert len(self.input_list) == len(self.output_list) >= 2
        bounds_function_dictionary = {}
        for i in xrange(1, len(self.input_list)):
            x_0, x_1 = self.input_list[i - 1], self.input_list[i]
            y_0, y_1 = self.output_list[i - 1], self.output_list[i]
            bounds_function_dictionary[(x_0, x_1)] = FunctionLinear(x_0=x_0, x_1=x_1, y_0=y_0, y_1=y_1)
        bounds_function_dictionary[(MINUS_INFINITY, self.input_list[0])] = self.function_undefined
        bounds_function_dictionary[(self.input_list[-1], PLUS_INFINITY)] = self.function_undefined
        FunctionComposite.__init__(self, bounds_function_dictionary)

    def normalised(self):
        area = self.integrate(MINUS_INFINITY, PLUS_INFINITY)
        output_list = [v / area for v in self.output_list]
        return FunctionPiecewiseLinear(self.input_list, output_list, self.function_undefined)

    def __getitem__(self, index):
        return self.output_list.__getitem__(index)

    def __len__(self):
        return len(self.output_list)

    def __iter__(self):
        return iter(self.output_list)

if __name__ == '__main__':
    #from spatiotemporal.temporal_events import TemporalEventTrapezium
    #e = TemporalEventTrapezium(1, 10, 3, 8)
    #print e
    #mf = e.membership_function()
    #print 'degree in [1 : 3]:', e.degree_in_interval(1, 3)
    #print 'degree in [3 : 4]:', e.degree_in_interval(3, 4)
    #print 'degree in [8 : 9]:', e.degree_in_interval(8, 9)
    #print 'degree in [2 : 9]:', e.degree_in_interval(2, 9)
    #print 'degree in [1 : 10]:', e.degree_in_interval()
    #print 'degree in [11 : 17]:', e.degree_in_interval(11, 17)

    a = FunctionLinear(None, None, 3, 0, -1, 1.0/9)
    print (a(-0.25) + a(0.25))/4
    print a(0.25) * 2.75 / 2

