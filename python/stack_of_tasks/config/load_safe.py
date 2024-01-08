from stack_of_tasks.config.yaml import dump, load
from stack_of_tasks.tasks.hierarchy import TaskHierarchy


class LoadSafe:
    @staticmethod
    def load_config(yml: str):
        return load(yml)

    def save_config(stack: TaskHierarchy):
        stack_state = {}
        for i, level in enumerate(stack.levels):
            stack_state[i] = list(level)

        return dump(stack_state)
