from typing import Any, Generic, List, Type, TypeVar

from stack_of_tasks.ui.model_mapping import InjectionArg

T = TypeVar("T")


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
    def create_instance(instance_cls: Type[T], kwargs: dict) -> T:
        kwargs.update(DependencyInjection.inject(kwargs))
        return instance_cls(**kwargs)
