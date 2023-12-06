from collections import defaultdict
from contextlib import contextmanager

import traits.api as ta

from stack_of_tasks.utils import ClassRegister


@contextmanager
def matrix_edit(obj: ta.HasTraits, trait: str, capture_old=True):
    old = obj.trait_get(trait) if capture_old else ta.Undefined
    yield
    obj.trait_property_changed(trait, old, obj.trait_get(trait))


register_all = ClassRegister("all")


@register_all.base
class BaseSoTHasTraits(ta.HasTraits):
    __init_subclass_hooks__ = []

    # Make 'private' traits (leading '_') have no type checking:
    __ = ta.Any(private=True, transient=True)

    trait_removed = ta.Event()

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        for f in cls.__init_subclass_hooks__:
            f(cls)

    def remove_trait(self, name):
        r = super().remove_trait(name)
        if r:
            self.trait_removed = name
        return r

    def __init__(self, **traits):
        self._update_private_metadata()
        super().__init__(**traits)

    def _update_private_metadata(self):
        prefix_names = list(
            filter(lambda x: not (x == "" or x == "*"), self.__prefix_traits__["*"])
        )

        for name, trait in self.traits().items():
            for prefix in prefix_names:
                if name.startswith(prefix):
                    trait.__dict__.update(self.__prefix_traits__[prefix].__dict__)


class ABCSoTHasTraits(ta.ABCHasTraits, BaseSoTHasTraits):
    pass


class Guard(object):
    class Context(object):
        def __init__(self, guard, items) -> None:
            self._guard = guard
            self._items = items

        def __enter__(self):
            self._guard._enter(self._items)

        def __exit__(self, exc_type, exc_value, traceback):
            self._guard._exit(self._items)

    def __init__(self) -> None:
        self.locked_items = defaultdict(int)

    def __call__(self, *items):
        return self.Context(self, items)

    def _enter(self, items):
        for item in items:
            self.locked_items[item] += 1

    def _exit(self, items):
        for item in items:
            self.locked_items[item] -= 1
            if self.locked_items[item] <= 0:
                del self.locked_items[item]

    def __contains__(self, items):
        return items in self.locked_items
