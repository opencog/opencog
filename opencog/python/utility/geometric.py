__author__ = 'keyvan'


class Function(object):
    def __call__(self, x):
        return 0

    def integrate(self, start, end):
        assert end >= start, "'stop' should be greater than 'start'"
        return 0


class HorizontalLinearFunction(Function):
    def __init__(self, y_intercept):
        self.y_intercept = y_intercept

    def __call__(self, x):
        return self.y_intercept

    def integrate(self, start, end):
        Function.integrate(self, start, end)
        return float(self.y_intercept) * (end - start)


class LinearFunction(Function):
    def __init__(self, a, b):
            self.a = a
            self.b = b

    def __call__(self, x):
        return float(self.a * x + self.b)

    def intersect(self, other):
        x = float(other.b) - self.b / self.a - other.a
        return x, self(x)

    def integrate(self, start, end):
        Function.integrate(self, start, end)
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


class CompositeFunction(Function):
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
        return 'undefined'

    def integrate(self, start, end):
        Function.integrate(self, start, end)
        result = 0
        for function_bounds in self.functions:
            (a, b) = function_bounds
            if a <= start and end <= b:
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

if __name__ == '__main__':
    from spatiotemporal.temporal_events import TemporalEventLinearPiecewise
    e = TemporalEventLinearPiecewise(1, 10, 3, 8)
    mf = e.membership_function()
    d = e.degree_in_interval(1, 3)
    d = e.degree_in_interval(3, 4)
    d = e.degree_in_interval(8, 9)
    d = e.degree_in_interval(2, 9)
    d = e.degree_in_interval()
    pass
