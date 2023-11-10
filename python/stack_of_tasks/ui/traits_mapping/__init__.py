from __future__ import annotations

import typing

import traits.api as ta

from stack_of_tasks.solver import InverseJacobianSolver, OSQPSolver

SOLVER = [InverseJacobianSolver, OSQPSolver]
from typing import Any, Type

import traits.api as ta
import traits.trait_types as tt
from traits.trait_type import _read_only, _write_only


def is_editable_trait(name: str, trait: ta.CTrait) -> bool:
    # name starts with underscore?
    if name.startswith("_"):
        return False

    # is property?
    if trait.is_property:
        if trait.property_fields[1] is _read_only or trait.property_fields[0] is _write_only:
            return False
        return True

    return True


def get_editable_trait_names(obj: ta.HasTraits):
    trait_names = []
    for trait_name in obj.editable_traits():
        trait: ta.CTrait = obj.base_trait(trait_name)
        if is_editable_trait(trait_name, trait):
            trait_names.append(trait_name)

    return trait_names


def get_init_arg_trait_names(cls: type[ta.HasTraits]):
    visible_trait_names = cls.class_visible_traits()
    cls_trts = cls.class_traits()

    injected_names = cls.class_trait_names(injected=lambda x: isinstance(x, str))

    trait_names = set(injected_names)
    for name in visible_trait_names:
        if is_editable_trait(name, cls_trts[name]):
            trait_names.add(name)

    return list(trait_names)


def is_valid_value(obj: ta.HasTraits | ta.MetaHasTraits, name: str, val: Any) -> bool:
    if isinstance(obj, type):
        trt = obj.class_traits()[name]
    else:
        trt = obj.base_trait(name)

    r = True
    try:
        trt.validate(obj, name, val)
    except ta.TraitError:
        r = False
    return r
