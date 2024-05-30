from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path

from typing import List, Tuple

import platformdirs
from PyQt5.QtWidgets import QApplication, QFileDialog

from stack_of_tasks.config import Configuration, dump, load
from stack_of_tasks.ui.application.logic.project import Logic_Project
from stack_of_tasks.ui.application.widgets.home_window import HomeWindow

LATEST_PATH = platformdirs.user_cache_path("stack_of_tasks") / "latest.json"


class Logic_Main:

    latest: dict[Path, Tuple[str, datetime]]

    def __init__(self) -> None:

        self.ui = HomeWindow()
        QApplication.instance().aboutToQuit.connect(self._app_quit)

        self.latest: dict[Path, List[str, datetime]] = {}
        self._load_latest()

        self.current_project = None

        self.ui.open_project_from_file.connect(self._open_from_file)
        self.ui.open_project.connect(self._open_recent)
        self.ui.new_project.connect(self.new_project)

        if len(self.latest) == 0:
            self.new_project()

    def _load_latest(self):
        if LATEST_PATH.exists():
            data = json.loads(LATEST_PATH.read_text())
            self.latest = {
                Path(k): [v[0], datetime.fromtimestamp(v[1])] for k, v in data.items()
            }

            self.ui.set_last_items(self.latest)

    def _save_latest(self):
        data = {k.as_posix(): (v[0], v[1].timestamp()) for k, v in self.latest.items()}
        LATEST_PATH.parent.mkdir(parents=True, exist_ok=True)
        LATEST_PATH.write_text(json.dumps(data))

    def _update_latest_project_list(self, url: Path):
        name = self.latest[url][0] if url in self.latest else url.as_posix()
        self.latest[url] = (name, datetime.now())

        self._save_latest()

    def _app_quit(self):
        if self.current_project is not None:
            self.current_project.teardown()

    # Projecct management

    def _safe_file(self, url: Path):
        yaml_str = dump(self.current_project.controller)
        url.write_text(yaml_str)

        self.current_project.url = url
        self._update_latest_project_list(url)

    def _safe_as(self):
        url, _ = QFileDialog.getSaveFileName(filter="YAML - Files (*.yml)")
        if url:  # only save if a file was chosen
            self._safe_file(Path(url))

    def _safe(self):
        if self.current_project.url is None:
            self._safe_as()
        else:
            self._safe_file(self.current_project.url)

    def _create_project(self, config):
        if self.current_project is not None:
            self.current_project.teardown()

        self.current_project = Logic_Project(config)

        self.current_project.ui.new_signal.connect(self.new_project)
        self.current_project.ui.open_file_signal.connect(self._open_from_file)
        self.current_project.ui.save_signal.connect(self._safe)
        self.current_project.ui.save_as_signal.connect(self._safe_as)

        self.current_project.ui.show()

    def _load_project(self, config_location: Path):
        config = load(config_location.read_text())
        self._update_latest_project_list(config_location)
        self._create_project(config)
        self.current_project.url = config_location
        self.ui.close()

    def _open_recent(self, index: int):
        self._load_project(list(self.latest.keys())[index])

    def _open_from_file(self):
        name, _ = QFileDialog.getOpenFileName(filter="YAML - Files (*.yml)")
        if name != "":
            self._load_project(Path(name))

    def new_project(self):
        from stack_of_tasks.robot_model.actuators import JointStatePublisherActuator
        from stack_of_tasks.solver.OSQPSolver import OSQPSolver

        config = Configuration(
            actuator_cls=JointStatePublisherActuator, solver_cls=OSQPSolver
        )

        self._create_project(config)

        self.ui.close()
