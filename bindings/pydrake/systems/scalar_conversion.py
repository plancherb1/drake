"""Provides utilities to aid in scalar type conversion."""

import copy

from pydrake.systems.framework import (
    LeafSystem_,
    SystemScalarConverter,
)
from pydrake.util.cpp_template import (
    _get_module_name_from_stack,
    TemplateClass,
)


def _get_conversion_pairs(T_list):
    # Take subset from supported conversions.
    T_pairs = []
    for T, U in SystemScalarConverter.SupportedConversionPairs:
        if T in T_list and U in T_list:
            T_pairs.append((T, U))
    return T_pairs


class TemplateSystem(TemplateClass):
    """Defines templated systems enabling scalar type conversion in Python.

    This differs from ``TemplateClass`` in that (a) the template must specify
    its parameters at construction time and (b) `.define` is overridden to
    allow defining ``f(T)`` for convenience.

    Any class that is added must:

    * Not define ``__init__``, as this will be overridden.
    * Define ``_construct(self, <args>, converter=None, <kwargs>)`` and
      ``_construct_copy(self, other, converter=None)`` instead. ``converter``
      must be present as an argument to ensure that it propagates properly.

    If any of these constraints are violated, then an error will be thrown
    at the time of the first class instantiation.

    Example::

        @TemplateSystem.define("MySystem_")
        def MySystem_(T):

            class Impl(LeafSystem_[T]):
                def _construct(self, value, converter=None):
                    LeafSystem_[T].__init__(self, converter=converter)
                    self.value = value

                def _construct_copy(self, other, converter=None):
                    Impl._construct(self, other.value, converter=converter)

            return Impl

        MySystem = MySystem_[None]  # Default instantiation.

    Things to note:

    * When defining ``_construct_copy``, if you are delegating to
      ``_construct`` within the same class, you should use
      ``Impl._construct(self, ...)``; if you use ``self._construct``, then you
      may get a child class's constructor if you are inheriting.
    * If you are delegating construction to a parent Python class for both
      constructors, use the parent class's ``__init__`` method, not its
      ``_construct`` or ``_construct_copy`` methods.
    * ``converter`` should always be non-None, as guaranteed by the
      overriding ``__init__``. We use ``converter=None`` to imply it should be
      positional, since Python2 does not have keyword-only arguments.
    """
    # TODO(eric.cousineau): Figure out if there is a way to avoid needing to
    # pass around converters in user code, avoiding the need to have Python
    # users deal with `SystemScalarConverter`.
    def __init__(self, name, T_list=None, T_pairs=None, module_name=None):
        """Constructs ``TemplateSystem``.

        Args:
            T_list: List of T's that the given system supports. By default, it
                is all types supported by `LeafSystem`.
            T_pairs: List of pairs, (T, U), defining a conversion from a
                scalar type of U to T. If None, this will use all possible
                pairs that the Python bindings of ``SystemScalarConverter``
                support.
            module_name: Defining ``module_name``, per ``TemplateClass``'s
                constructor.
        """
        if module_name is None:
            module_name = _get_module_name_from_stack()
        TemplateClass.__init__(self, name, module_name=module_name)

        # Check scalar types and conversions, using defaults if unspecified.
        if T_list is None:
            T_list = SystemScalarConverter.SupportedScalars
        for T in T_list:
            assert T in SystemScalarConverter.SupportedScalars, (
                "Type {} is not a supported scalar type".format(T))
        if T_pairs is None:
            T_pairs = _get_conversion_pairs(T_list)
        for T_pair in T_pairs:
            T, U = T_pair
            assert T in T_list and U in T_list, (
                "Conversion {} is not in the original parameter list"
                .format(T_pair))
            assert T_pair in \
                SystemScalarConverter.SupportedConversionPairs, (
                    "Conversion {} is not supported".format(T_pair))

        self._T_list = list(T_list)
        self._T_pairs = list(T_pairs)
        self._converter = self._make_converter()

    @classmethod
    def define(cls, name, T_list=None, T_pairs=None, *args, **kwargs):
        """Provides a decorator which can be used define a scalar-type
        convertible System as a template.

        The decorated function must be of the form ``f(T)``, which returns a
        class which will be the instantation for type ``T`` of the given
        template.

        Args:
            name: Name of the system template. This should match the name of
                the object being decorated.
            T_list: See ``__init__`` for more information.
            T_pairs: See ``__init__`` for more information.
            args, kwargs: These are passed to the constructor of
                ``TemplateSystem``.
        """
        template = cls(name, T_list, T_pairs, *args, **kwargs)
        param_list = [(T,) for T in template._T_list]

        def decorator(instantiation_func):

            def wrapped(param):
                T, = param
                return instantiation_func(T)

            template.add_instantiations(wrapped, param_list)
            return template

        return decorator

    def _on_add(self, param, cls):
        TemplateClass._on_add(self, param, cls)
        T, = param

        # Check that the user has not defined `__init__`, and has defined
        # `_construct` and `_construct_copy`.
        if not issubclass(cls, LeafSystem_[T]):
            raise RuntimeError(
                "{} must inherit from {}".format(cls, LeafSystem_[T]))

        # Use the immediate `__dict__`, rather than querying the attributes, so
        # that we don't get spillover from inheritance.
        d = cls.__dict__
        no_init = "__init__" not in d
        has_construct = "_construct" in d
        has_copy = "_construct_copy" in d
        if not no_init:
            raise RuntimeError(
                "{} defines `__init__`, but should not. Please implement "
                "`_construct` and `_construct_copy` instead."
                .format(cls.__name__))
        if not has_construct:
            raise RuntimeError(
                "{} does not define `_construct`. Please ensure this is "
                "defined.".format(cls.__name__))
        if not has_copy:
            raise RuntimeError(
                "{} does not define `_construct_copy`. Please ensure this "
                "is defined.".format(cls.__name__))

        # Patch `__init__`.
        template = self

        def system_init(self, *args, **kwargs):
            converter = None
            if "converter" in kwargs:
                converter = kwargs.pop("converter")
            if converter is None:
                # Use default converter.
                converter = template._converter
            if template._check_if_copying(self, *args, **kwargs):
                other = args[0]
                cls._construct_copy(self, other, converter=converter)
            else:
                cls._construct(
                    self, *args, converter=converter, **kwargs)

        cls.__init__ = system_init
        return cls

    def _check_if_copying(self, obj, *args, **kwargs):
        # Checks if a function signature implies a copy constructor.
        # Since this is called after `converter` has been removed from
        # `kwargs`, we just ensure that there are no additional arguments.
        if len(args) == 1 and len(kwargs) == 0:
            if self.is_subclass_of_instantiation(type(args[0])):
                return True
        return False

    def _make_converter(self):
        # Creates system scalar converter for the template class.
        converter = SystemScalarConverter()

        # Define capture to ensure the current values are bound, and do not
        # change through iteration.
        # N.B. This does not directly instantiate the template; it is deferred
        # to when the conversion is called.
        def add_captured(T_pair):
            T, U = T_pair

            def conversion(system):
                assert isinstance(system, self[U])
                return self[T](system)

            converter.Add[T, U](conversion)

        for T_pair in self._T_pairs:
            add_captured(T_pair)
        return converter
