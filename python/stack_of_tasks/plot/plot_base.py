import traits.api as ta

from tf.transformations import euler_from_matrix

from stack_of_tasks.controller import Controller
from stack_of_tasks.ref_frame import RefFrame


class PlotBase(ta.HasTraits):
    frames = ta.List(RefFrame)

    def __init__(self, controller: Controller, prefix="SOT", **traits) -> None:
        super().__init__(**traits)

        self._prefix = prefix
        self._controller = controller

        observed = "levels, levels:items, levels:items:items"
        self._controller.task_hierarchy.observe(self.update_names, observed)
        self._controller.observe(self.update_data, "_updated")

    @ta.observe("frames, frames:items")
    def update_names(self, _=None):
        pass

    def update_data(self, _=None):
        pass

    def _names(self):
        names = []
        joint_names = [j.name for j in self._controller.robot_model.active_joints]
        names.extend([f"q/{name}" for name in joint_names])
        names.extend([f"dq/{name}" for name in joint_names])

        frame_variables = ["x", "y", "z", "rx", "ry", "rz"]
        for frame in self.frames:
            names.extend([f"frame/{frame.name}/{name}" for name in frame_variables])

        for i, level in enumerate(self._controller.task_hierarchy.levels):
            prefix = f"level {i}"
            for task in level:
                name = f"{prefix}/{task.name}"
                names.extend([f"{name}/residual[{i}]" for i in range(task.task_size)])

        return names

    def _values(self):
        values = []
        values.extend(self._controller.robot_state.joint_values)
        values.extend(self._controller.dq)

        for frame in self.frames:
            pos = list(frame.T[0:3, 3])
            rpy = list(euler_from_matrix(frame.T[0:3, 0:3]))
            values.extend(pos + rpy)

        for level in self._controller.task_hierarchy.levels:
            for task in level:
                values.extend(task.residual)

        return values
