import csv
from datetime import datetime

from .plot_base import PlotBase


class PlotCSV(PlotBase):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._file = None
        self._writer = None
        self._start = None

    def create_writer(self, name, start=None):
        if self._file is not None:
            self._file.close()

        self._start = start or datetime.now()
        self._file = open(name, "w", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(["t"] + self._names())

    def update_names(self, _=None):
        # close current file and trigger new file creation on next update_data()
        if self._file is not None:
            self._file.close()
            self._file = None

    def update_data(self, _=None):
        now = datetime.now()
        if self._file is None:
            self.create_writer(f"{self._prefix}-{now.strftime('%y-%m-%d %H:%M')}.csv", now)

        t = now - self._start
        self._writer.writerow([t] + self._values())
