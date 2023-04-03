from contextlib import contextmanager

import traits.api as ta


@contextmanager
def matrix_edit(obj: ta.HasTraits, trait: str, capture_old=True):
    old = obj.trait_get(trait) if capture_old else ta.Undefined
    yield
    obj.trait_property_changed(trait, old, obj.trait_get(trait))
