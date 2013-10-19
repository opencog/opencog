from utility.numeric.globals import MINUS_INFINITY, PLUS_INFINITY

__author__ = 'keyvan'


class Function(object):
    def __call__(self, x):
        return 0

    def integrate(self, start, end):
        assert end >= start, "'stop' should be greater than 'start'"
        return 0


class FunctionHorizontalLinear(Function):
    def __init__(self, y_intercept):
        self.y_intercept = y_intercept
        self.a = 0

    def __call__(self, x):
        return self.y_intercept

    def integrate(self, start, end):
        Function.integrate(self, start, end)
        return float(self.y_intercept) * (end - start)

    @property
    def b(self):
        return self.y_intercept


class FunctionLinear(Function):
    def __init__(self, a=None, b=None, x_0=None, x_1=None, y_0=None, y_1=None):
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
        Function.integrate(self, start, end)
        if self.a == 0:
            return self.b * (end - start)

        x_intercept = self.x_intercept

        if start >= x_intercept:
            area_of_triangle = (end - start) * (self(end) - self(start)) / 2
            area_of_rectangle = (end - start) * self(start)
            return area_of_triangle + area_of_rectangle

        if end <= x_intercept:
            area_of_triangle = (end - start) * (self(start) - self(end)) / 2
            area_of_rectangle = (end - start) * self(end)
            return area_of_triangle + area_of_rectangle

        minus_triangle = (x_intercept - start) * self(start)
        plus_triangle = (end - x_intercept) * self(end)
        return minus_triangle + plus_triangle

    @property
    def x_intercept(self):
        return - float(self.b) / self.a

    @property
    def y_intercept(self):
        return self(0)


class FunctionComposite(Function):
    function_undefined = lambda x: None

    def __init__(self, bounds_function_dictionary):
        assert isinstance(bounds_function_dictionary, dict)
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
        Function.integrate(self, start, end)
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


class FunctionPiecewiseLinear(FunctionComposite):
    def __init__(self, input_list, output_list):
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

    def __getitem__(self, index):
        return self.output_list.__getitem__(index)

    def __len__(self):
        return len(self.output_list)

    def __iter__(self):
        return iter(self.output_list)

if __name__ == '__main__':
    from spatiotemporal.temporal_events import TemporalEventTrapezium
    e = TemporalEventTrapezium(1, 10, 3, 8)
    mf = e.membership_function()
    d = e.degree_in_interval(1, 3)
    d = e.degree_in_interval(3, 4)
    d = e.degree_in_interval(8, 9)
    d = e.degree_in_interval(2, 9)
    d = e.degree_in_interval(11, 17)
    d = e.degree_in_interval()
    pass
