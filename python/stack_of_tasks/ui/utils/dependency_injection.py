from typing import Any, Generic, List, Type, TypeVar

from stack_of_tasks.ui.model_mapping import InjectionArg

T = TypeVar("T")

import traits.api as ta


class DependencyInjection:
    mapping = {}

    @staticmethod
    def inject(arg_vector):
        injected = {}

        for arg in arg_vector.keys():
            if isinstance(arg_vector[arg], InjectionArg):
                injected[arg] = DependencyInjection.mapping[arg]
            else:
                injected[arg] = arg_vector[arg]

        return injected

    @staticmethod
    def _is_injected(val):
        return isinstance(val, str)

    @staticmethod
    def create_instance(instance_cls: Type[T], kwargs: dict) -> T:
        if issubclass(instance_cls, ta.HasTraits):
            inj_t = instance_cls.class_traits(injected=DependencyInjection._is_injected)

            for k in inj_t.values():
                param_name = k.injected
                if param_name in DependencyInjection.mapping:
                    kwargs[param_name] = DependencyInjection.mapping[param_name]

        kwargs.update(DependencyInjection.inject(kwargs))
        print(kwargs)
        return instance_cls(**kwargs)
