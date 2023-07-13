from contextlib import contextmanager

import traits.api as ta


@contextmanager
def matrix_edit(obj: ta.HasTraits, trait: str, capture_old=True):
    old = obj.trait_get(trait) if capture_old else ta.Undefined
    yield
    obj.trait_property_changed(trait, old, obj.trait_get(trait))


class BaseSoTHasTraits(ta.HasTraits):
    # Make 'private' traits (leading '_') have no type checking:
    __ = ta.Any(private=True, transient=True)

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
