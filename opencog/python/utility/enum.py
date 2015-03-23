# -*- coding: utf-8 -*-

# enum.py
# Part of enum, a package providing enumerated types for Python.
#
# Copyright © 2007–2009 Ben Finney <ben+python@benfinney.id.au>
# This is free software; you may copy, modify and/or distribute this work
# under the terms of the GNU General Public License, version 2 or later
# or, at your option, the terms of the Python license.

""" Robust enumerated type support in Python.

This package provides a module for robust enumerations in Python.

An enumeration object is created with a sequence of string arguments
to the Enum() constructor::

    >>> from enum import Enum
    >>> Colours = Enum('red', 'blue', 'green')
    >>> Weekdays = Enum('mon', 'tue', 'wed', 'thu', 'fri', 'sat', 'sun')

The return value is an immutable sequence object with a value for each
of the string arguments. Each value is also available as an attribute
named from the corresponding string argument::

    >>> pizza_night = Weekdays[4]
    >>> shirt_colour = Colours.green

The values are constants that can be compared only with values from
the same enumeration; comparison with other values will invoke
Python's fallback comparisons::

    >>> pizza_night == Weekdays.fri
    True
    >>> shirt_colour > Colours.red
    True
    >>> shirt_colour == "green"
    False

Each value from an enumeration exports its sequence index
as an integer, and can be coerced to a simple string matching the
original arguments used to create the enumeration::

    >>> str(pizza_night)
    'fri'
    >>> shirt_colour.index
    2
"""


__author_name__ = "Ben Finney"
__author_email__ = "ben+python@benfinney.id.au"
__author__ = "%(__author_name__)s <%(__author_email__)s>" % vars()

_copyright_year_begin = "2007"
__date__ = "2009-08-26"
_copyright_year_latest = __date__.split('-')[0]
_copyright_year_range = _copyright_year_begin
if _copyright_year_latest > _copyright_year_begin:
    _copyright_year_range += "–%(_copyright_year_latest)s" % vars()
__copyright__ = (
                    "Copyright © %(_copyright_year_range)s"
                    " %(__author_name__)s") % vars()
__license__ = "Choice of GPL or Python license"

__url__ = "http://pypi.python.org/pypi/enum/"
__version__ = "0.4.4"


class EnumException(Exception):
    """ Base class for all exceptions in this module. """

    def __init__(self, *args, **kwargs):
        if self.__class__ is EnumException:
            class_name = self.__class__.__name__
            raise NotImplementedError(
                "%(class_name)s is an abstract base class" % vars())
        super(EnumException, self).__init__(*args, **kwargs)


class EnumEmptyError(AssertionError, EnumException):
    """ Raised when attempting to create an empty enumeration. """

    def __str__(self):
        return "Enumerations cannot be empty"


class EnumBadKeyError(TypeError, EnumException):
    """ Raised when creating an Enum with non-string keys. """

    def __init__(self, key):
        self.key = key

    def __str__(self):
        return "Enumeration keys must be strings: %(key)r" % vars(self)


class EnumImmutableError(TypeError, EnumException):
    """ Raised when attempting to modify an Enum. """

    def __init__(self, *args):
        self.args = args

    def __str__(self):
        return "Enumeration does not allow modification"


def _comparator(func):
    """ Decorator for EnumValue rich comparison methods. """
    def comparator_wrapper(self, other):
        try:
            assert self.enumtype == other.enumtype
            result = func(self.index, other.index)
        except (AssertionError, AttributeError):
            result = NotImplemented

        return result
    comparator_wrapper.__name__ = func.__name__
    comparator_wrapper.__doc__ = getattr(float, func.__name__).__doc__
    return comparator_wrapper

class EnumValue(object):
    """ A specific value of an enumerated type. """

    def __init__(self, enumtype, index, key):
        """ Set up a new instance. """
        self._enumtype = enumtype
        self._index = index
        self._key = key

    @property
    def enumtype(self):
        return self._enumtype

    @property
    def key(self):
        return self._key

    def __str__(self):
        return str(self.key)

    @property
    def index(self):
        return self._index

    def __repr__(self):
        return "EnumValue(%(_enumtype)r, %(_index)r, %(_key)r)" % vars(self)

    def __hash__(self):
        return hash(self._index)

    @_comparator
    def __eq__(self, other):
        return (self == other)

    @_comparator
    def __ne__(self, other):
        return (self != other)

    @_comparator
    def __lt__(self, other):
        return (self < other)

    @_comparator
    def __le__(self, other):
        return (self <= other)

    @_comparator
    def __gt__(self, other):
        return (self > other)

    @_comparator
    def __ge__(self, other):
        return (self >= other)


class Enum(object):
    """ Enumerated type. """

    def __init__(self, *keys, **kwargs):
        """ Create an enumeration instance. """

        value_type = kwargs.get('value_type', EnumValue)

        if not keys:
            raise EnumEmptyError()

        keys = tuple(keys)
        values = [None] * len(keys)

        for i, key in enumerate(keys):
            value = value_type(self, i, key)
            values[i] = value
            try:
                super(Enum, self).__setattr__(key, value)
            except TypeError:
                raise EnumBadKeyError(key)

        self.__dict__['_keys'] = keys
        self.__dict__['_values'] = values

    def __setattr__(self, name, value):
        raise EnumImmutableError(name)

    def __delattr__(self, name):
        raise EnumImmutableError(name)

    def __len__(self):
        return len(self._values)

    def __getitem__(self, index):
        return self._values[index]

    def __setitem__(self, index, value):
        raise EnumImmutableError(index)

    def __delitem__(self, index):
        raise EnumImmutableError(index)

    def __iter__(self):
        return iter(self._values)

    def __contains__(self, value):
        is_member = False
        if isinstance(value, basestring):
            is_member = (value in self._keys)
        else:
            is_member = (value in self._values)
        return is_member


# Local variables:
# mode: python
# time-stamp-format: "%:y-%02m-%02d"
# time-stamp-start: "__date__ = \""
# time-stamp-end: "\"$"
# time-stamp-line-limit: 200
# End:
# vim: filetype=python fileencoding=utf-8 :