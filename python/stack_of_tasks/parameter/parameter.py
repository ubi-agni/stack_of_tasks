import traits.api as ta

from stack_of_tasks.utils.traits import BaseSoTHasTraits


class Parameter(BaseSoTHasTraits):
    name: str = ta.Str()
    value = ta.Any()
