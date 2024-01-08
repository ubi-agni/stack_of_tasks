import traits.api as ta

from stack_of_tasks.marker.trait_marker import IAMarker

from . import RefFrame


class MarkerFrame(RefFrame):
    marker = ta.Instance(IAMarker)

    T = ta.Delegate("marker", "transform")

    def __init__(self, marker: IAMarker, **traits):
        super().__init__(**traits, marker=marker)
