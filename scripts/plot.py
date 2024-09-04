import sys
from pathlib import Path

import numpy

import rospy

from stack_of_tasks.config import load
from stack_of_tasks.controller import Controller
from stack_of_tasks.marker import IAMarker, MarkerServer
from stack_of_tasks.plot import PlotCSV, PlotPublisher


def run(controller: Controller):
    dq = None
    rate = rospy.Rate(10)
    while True:
        dq = controller.control_step(dq)
        if numpy.linalg.norm(dq) < 1e-3:
            break
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("ik", disable_signals=True)
    path = Path(sys.argv[1])
    config = load(path.read_text())

    controller = Controller(config)
    print(controller.robot_model.vmaxs / controller.rate)

    marker_server = MarkerServer()
    IAMarker._default_frame_id = controller.robot_model.root_link

    frames = config.objects["frames"]
    for marker in config.objects["marker"]:
        marker_server.add_marker(marker)

    frame = [f for f in frames if f.name == "panda_hand_tcp"][0]
    marker = [m for m in config.objects["marker"] if m.name == "pose"][0]

    plot = PlotCSV(controller, frames=frames)
    plot.create_writer(f"{path.stem}.csv")

    rs = controller.robot_state
    joints = [0, -numpy.pi / 4, 0, -3 * numpy.pi / 4, 0, numpy.pi / 2, numpy.pi / 4, 0]
    joints = [0, 0.14124, 0, -1.44661, 0, 1.58987, 0.785, 0]
    rs.incoming_joint_values = joints
    rs.update()

    T = frame.T
    T[0:3, 3] = [0.2, 0.0, 0.5]  # 0.6 -> 0.2
    marker.transform = T
    run(controller)

    plot._writer.writerow([])
    T[0:3, 3] = [0.6, 0.0, 0.5]  # 0.2 -> 0.6
    marker.transform = T
    run(controller)

    numpy.set_printoptions(precision=5, suppress=True, linewidth=100)
    print(controller.robot_state.joint_values)
