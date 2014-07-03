from math import fabs, isnan
from datetime import datetime
from spatiotemporal.unix_time import UnixTime
from utility.generic import convert_dict_to_sorted_lists
from utility.numeric.globals import EPSILON
from numpy import NINF as NEGATIVE_INFINITY, PINF as POSITIVE_INFINITY
from scipy.integrate import quad

__author__ = 'keyvan'


def integral(function, start, end):
    if hasattr(function, 'integral'):
        return function.integral(start, end)
    area, error = quad(function, start, end)
    return area


def almost_equals(a, b, epsilon=EPSILON):
    if fabs(a - b) < epsilon:
        return True
    return False


def invoke_method_on(method, sequence_or_point):
    if method is None:
        return None
    if not callable(method):
        raise TypeError("'method' is not callable")
    result = []
    try:
        for point in sequence_or_point:
            if type(point) is datetime:
                point = UnixTime(point)
            result.append(method(point))
    except TypeError:
        if type(sequence_or_point) is datetime:
            sequence_or_point = UnixTime(sequence_or_point)
        return method(sequence_or_point)
    return result


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
    _domain = None
    _range = None
    _function_undefined = None

    def __init__(self, function_undefined=None, domain=None):
        if function_undefined is not None:
            self.function_undefined = function_undefined
        if domain is not None:
            if not hasattr(domain, '__iter__') or not hasattr(domain, '__getitem__'):
                    raise TypeError("'domain' should be iterable and support indexing")
            self._domain = domain

    def call_on_single_point(self, x):
        """
        to override, __call__ invokes this to handle both points and sequences
        """
        return 0

    def derivative(self, point):
        return None

    def _check_domain_for(self, feature_name):
        if self.domain is None:
            raise TypeError("'{0}' object does not support {1}, 'domain' should be specified".format(
                self.__class__.__name__, feature_name))

    def plot(self, plt=None):
        self._check_domain_for('plotting')
        if plt is None:
            import matplotlib.pyplot as plt
        plt.plot(self.domain, self.range)
        return plt

    @property
    def function_undefined(self):
        return self._function_undefined

    @function_undefined.setter
    def function_undefined(self, value):
        if value is not None and not isinstance(value, Function):
            raise TypeError("'function_undefined' should be of type 'Function'")
        self._function_undefined = value

    @property
    def domain(self):
        return self._domain

    @property
    def range(self):
        return self()

    def __call__(self, x=None):
        if x is None:
            self._check_domain_for("call with 'None'")
            x = self.domain

        return invoke_method_on(self.call_on_single_point, x)

    def __getitem__(self, index):
        self._check_domain_for('indexing')
        return self.range[index]

    def __len__(self):
        self._check_domain_for('len()')
        return len(self.range)

    def __iter__(self):
        self._check_domain_for('iter()')
        return iter(self.range)

    def __reversed__(self):
        self._check_domain_for('reversed()')
        return reversed(self.range)


class FunctionLinear(Function):
    def __init__(self, a=None, b=None, x_0=None, y_0=None, x_1=None, y_1=None):
        #(x_0, y_0), (x_1, y_1) = sorted([(x_0, y_0), (x_1, y_1)])
        if (a, b) == (None, None):
            a = (float(y_1) - y_0) / (x_1 - x_0)
            b = y_0 - a * x_0
            if isnan(a) or isnan(b):
                pass
        self.a = a
        self.b = b

    def call_on_single_point(self, x):
        return float(self.a * x + self.b)

    def intersect(self, other):
        if almost_equals(self.a, other.a):
            return None
        x = (float(other.b) - self.b) / (self.a - other.a)
        return x, self(x)

    def integral(self, start, end):
        if start >= end:
            return 0
        if self.a == 0:
            return self.b * (end - start)

        x_intercept = self.x_intercept

        if start > x_intercept or end < x_intercept or almost_equals(end, x_intercept) or almost_equals(start, x_intercept):
            return (self(start) + self(end)) * (end - start) / 2.0

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


class FunctionHorizontalLinear(FunctionLinear):
    def __init__(self, y_intercept):
        FunctionLinear.__init__(self, a=0, b=y_intercept)

    def call_on_single_point(self, x):
        return self.b

    def integral(self, start, end):
        if start >= end:
            return 0
        if almost_equals(self.b, 0):
            return 0
        return float(self.b) * (end - start)

    def derivative(self, point):
        return 0

FUNCTION_ZERO = FunctionHorizontalLinear(0)
FUNCTION_ONE = FunctionHorizontalLinear(1)


class FunctionComposite(Function):
    is_normalised = False

    def __init__(self, dictionary_bounds_function, function_undefined=None, domain=None, is_normalised=False):
        if is_normalised is not False:
            self.is_normalised = True

        Function.__init__(self, function_undefined=function_undefined, domain=domain)
        if not isinstance(dictionary_bounds_function, dict):
            raise TypeError("'dictionary_bounds_function' should be a dictionary with (lower_bound, higher_bound) "
                            "tuple keys and values of type 'Function'")
        self._dictionary_bounds_function = dictionary_bounds_function

    def call_on_single_point(self, x):
        for function_bounds in self.dictionary_bounds_function:
            (a, b) = function_bounds
            if a <= x:
                if b >= x:
                    if self.dictionary_bounds_function[function_bounds] is None:
                        return None
                    return self.dictionary_bounds_function[function_bounds](x)
        return self.function_undefined(x)

    def integral(self, start, end):
        if self.is_normalised and self.domain is not None:
            if (start < self.domain[0] or almost_equals(start, self.domain[0])) and (
                    end > self.domain[-1] or almost_equals(end, self.domain[-1])):
                return 1.0

        if start >= end:
            return 0
        result = 0
        for function_bounds in self.dictionary_bounds_function:
            (a, b) = function_bounds
            if a <= start:
                if b >= end:
                    return self.dictionary_bounds_function[function_bounds].integral(start, end)
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
            result += self.dictionary_bounds_function[function_bounds].integral(a, b)
        return result

    def find_bounds_for(self, point):
        for bounds in self.dictionary_bounds_function:
            (a, b) = bounds
            if a <= point and b >= point:
                return bounds

    def derivative(self, point):
        return self.dictionary_bounds_function[self.find_bounds_for(point)].derivative(point)

    def function_in_point(self, point):
        for bounds in self.dictionary_bounds_function:
            a, b = bounds
            if a <= point <= b:
                return self.dictionary_bounds_function[bounds]
        return None

    # def functions_in_interval(self, interval_start, interval_end):
    #     dictionary_bounds_function = {}
    #     for bounds in self.dictionary_bounds_function:
    #         a, b = bounds
    #         if (interval_start < a or almost_equals(interval_start, a)) and (
    #
    #         ):

    @property
    def dictionary_bounds_function(self):
        return self._dictionary_bounds_function


class FunctionPiecewiseLinear(FunctionComposite):
    def __init__(self, dictionary_input_output, function_undefined=None, is_normalised=False):
        self.input_list, self.output_list = convert_dict_to_sorted_lists(dictionary_input_output)

        dictionary_bounds_function = {}
        for i in xrange(1, len(self.input_list)):
            x_0, x_1 = self.input_list[i - 1], self.input_list[i]
            y_0, y_1 = self.output_list[i - 1], self.output_list[i]
            dictionary_bounds_function[(x_0, x_1)] = FunctionLinear(x_0=x_0, x_1=x_1, y_0=y_0, y_1=y_1)
        if NEGATIVE_INFINITY not in self.input_list:
            dictionary_bounds_function[(NEGATIVE_INFINITY, self.input_list[0])] = function_undefined
        if POSITIVE_INFINITY not in self.input_list:
            dictionary_bounds_function[(self.input_list[-1], POSITIVE_INFINITY)] = function_undefined
        FunctionComposite.__init__(self, dictionary_bounds_function,
                                   function_undefined=function_undefined,
                                   domain=self.input_list,
                                   is_normalised=is_normalised)

    def normalised(self):
        area = self.integral(NEGATIVE_INFINITY, POSITIVE_INFINITY)
        if almost_equals(area, 0):
            area = self.integral(NEGATIVE_INFINITY, POSITIVE_INFINITY)
        dictionary_input_output = {}
        output_list = [y / area for y in self.output_list]
        for i in xrange(len(self.input_list)):
            dictionary_input_output[self.input_list[i]] = output_list[i]
        result = FunctionPiecewiseLinear(dictionary_input_output, function_undefined=self.function_undefined)
        result.is_normalised = True
        return result

    def __and__(self, other):
        for bounds in self.dictionary_bounds_function:
            a, b = bounds
            linear_function = self.dictionary_bounds_function[bounds]



if __name__ == '__main__':
    a = FunctionLinear(1, 0)
    b = FunctionLinear(-1, 1)
    print a.intersect(b)
