from collections import defaultdict

import traits.api as ta


class BaseSoTHasTraits(ta.HasTraits):
    trait_removed = ta.Event()

    def remove_trait(self, name):
        r = super().remove_trait(name)
        if r:
            self.trait_removed = name
        return r


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

    def __contains__(self, item):
        return item in self.locked_items
