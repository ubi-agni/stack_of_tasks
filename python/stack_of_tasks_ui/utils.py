from stack_of_tasks import tasks
from stack_of_tasks.marker.markers import OrientationMarker, PositionMarker, SixDOFMarker

TASK_MIME_FOTMAT_NAME = "application/sot-task"
TASK_INSTANCE_MIME_FOTMAT_NAME = "application/sot-task-instanstance"


# Task - class mapping

TASK_CLASSES = [
    tasks.PlaneTask,
    tasks.PositionTask,
    tasks.OrientationTask,
    tasks.ConeTask,
    tasks.ParallelTask,
    tasks.DistanceTask,
]

TASKS = {}

for task in TASK_CLASSES:
    name = task.__name__
    TASKS[name] = {"class": task, "display_name": task.name}


def mime_data_from_name(name: str):
    classname: str = None
    for k, v in TASKS.items():
        if v["display_name"] == name:
            classname = k
            break
    if classname is None:
        raise ValueError()

    return classname.encode("utf-8")


# Marker - Marker-class Mapping

MARKERS = {}
MARKERS["Orientation"] = {"class": OrientationMarker}
MARKERS["Position"] = {"class": PositionMarker}
MARKERS["6 DoF"] = {"class": SixDOFMarker}


class TypeToUi:
    @staticmethod
    def map_type_to_widget(value: type):
        if value is float:
            pass

        else:
            pass
