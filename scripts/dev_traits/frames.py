#!/usr/bin/env python3

import numpy as np

from stack_of_tasks.ref_frame import HasJacobian, HasTransform
from stack_of_tasks.ref_frame.frames import Origin, RefFrame, ta
from stack_of_tasks.ref_frame.offset import Offset


class JacobianFrame(RefFrame):
    T = ta.Array(value=np.identity(4))
    J = ta.Array(value=np.zeros((6, 9)))


def main():
    orig = Origin()
    jf = JacobianFrame()

    print("Instance orig: ", isinstance(orig, HasTransform), isinstance(orig, HasJacobian))
    print("Instance jf: ", isinstance(jf, HasTransform), isinstance(jf, HasJacobian))

    offst_orig = Offset(orig)
    offst_jf = Offset(jf)

    print(
        "Instance offset orig: ",
        isinstance(offst_orig, HasTransform),
        isinstance(offst_orig, HasJacobian),
    )
    print(
        "Instance offset jf: ",
        isinstance(offst_jf, HasTransform),
        isinstance(offst_jf, HasJacobian),
    )

    def pf(text):
        return lambda x: print(text, "\n   ", x.new)

    jf.observe(pf("Observe jf.T"), "T")
    offst_jf.observe(pf("Observe offset_jf.T"), "T")

    jf.observe(pf("Observe jf.J"), "J")
    offst_jf.observe(pf("Observe offset_jf.J"), "J")

    print("Set jf.T")
    jf.T = np.random.random((4, 4))

    print("Set offset_jf.offset")
    offst_jf.offset = np.random.random((4, 4))

    print("Set jf.J")
    jf.J = np.random.random((6, 9))


if __name__ == "__main__":
    main()
