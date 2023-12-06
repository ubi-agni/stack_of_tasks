from stack_of_tasks.config.yaml import dump, load
from stack_of_tasks.robot_model.actuators import DummyActuator
from stack_of_tasks.robot_model.robot_model import RobotModel
from stack_of_tasks.robot_model.robot_state import RobotState
from stack_of_tasks.solver.OSQPSolver import OSQPSolver
from stack_of_tasks.tasks.hierarchy import TaskHierarchy


class LoadSafe:
    def load_config(self):
        pass

    @staticmethod
    def save_config(stack: TaskHierarchy):
        stack_state = {}
        for i, level in enumerate(stack.levels):
            stack_state[i] = level

        state = {"sot": stack_state}
        return dump(state)

    # def create_empty(
    #    self,
    #    actuator_cls: DummyActuator,
    #    solver_cls: OSQPSolver,
    #    robot_model_param: str = None,
    #    robot_state_ns_prefix: str = None,
    # ):
    #    robot_model = RobotModel()
    #    robot_state = RobotState(robot_model)

    #    actruator = actuator_cls(robot_state)

    #    task_hierarchy = TaskHierarchy()
    #    solver = solver_cls(8, task_hierarchy)

    #    controller = Controller(
    #        robot_model, robot_state, actruator, solver=solver, task_hierarchy=task_hierarchy
    #    )

    #    return controller
