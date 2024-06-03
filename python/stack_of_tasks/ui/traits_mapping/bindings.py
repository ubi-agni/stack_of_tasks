from __future__ import annotations

import logging
import weakref
from abc import ABC, abstractmethod

import typing
from typing import Any

import traits.api as ta
from PyQt5.QtCore import QModelIndex, pyqtBoundSignal
from PyQt5.QtWidgets import QWidget
from traits.observation.events import ListChangeEvent, TraitChangeEvent

from stack_of_tasks.tasks.base import Task
from stack_of_tasks.tasks.hierarchy import TaskHierarchy
from stack_of_tasks.ui import RawDataRole
from stack_of_tasks.utils.traits import Guard


def set_user_property(widget: QWidget, value: Any) -> None:
    if (prop_name := get_user_prop_name(widget)) is not None:
        widget.setProperty(prop_name, value)
    else:
        print(f"{widget} has no user property.")


def get_user_property(widget: QWidget) -> Any:
    if (prop_name := get_user_prop_name(widget)) is not None:
        return widget.property(prop_name)


def get_user_prop_signal_name(widget: QWidget) -> typing.Optional[str]:
    if (
        user_prop := widget.metaObject().userProperty()
    ) is not None and user_prop.hasNotifySignal():
        return user_prop.notifySignal().name().data().decode()


def get_user_prop_name(widget: QWidget) -> typing.Optional[str]:
    if (user_prop := widget.metaObject().userProperty()) is not None:
        return user_prop.name()


def get_user_prop_signal(widget: QWidget) -> typing.Optional[pyqtBoundSignal]:
    if name := get_user_prop_signal_name(widget):
        return getattr(widget, name)


class Binder(ABC):
    def __init__(self) -> None:
        self.finalizer = weakref.finalize(self, self._teardown)

    @abstractmethod
    def _teardown(self):
        pass


class WidgetTraitBinding(Binder):
    def __init__(
        self,
        hasTrait: ta.HasTraits,
        traitName: str,
        widget: QWidget,
        propname: str = None,
        set_post_init=False,
    ) -> None:

        super().__init__()

        self._hasTrait = weakref.ref(hasTrait)
        self._trait_name = traitName

        self._widget = weakref.ref(widget)
        widget.destroyed.connect(self.finalizer)

        self._is_array = isinstance(self._hasTrait().trait(traitName).trait_type, ta.Array)

        if propname is None:
            self.prop_name = get_user_prop_name(widget)
        else:
            self.prop_name = propname

        if self.prop_name is None:
            return

        self.widget_prop = widget.metaObject().property(
            widget.metaObject().indexOfProperty(self.prop_name)
        )

        if self.widget_prop.hasNotifySignal():
            signal: pyqtBoundSignal = getattr(
                widget, str(self.widget_prop.notifySignal().name(), "utf-8")
            )
            logging.debug(f"notify signal {signal} ")
            signal.connect(self)

            if set_post_init:
                val = widget.property(self.prop_name)
                self(val)

    def _teardown(self):
        pass

    def __call__(self, value) -> Any:
        self._hasTrait().trait_set(**{self._trait_name: value})


class TraitWidgetBinding(Binder):
    def __init__(
        self,
        hasTrait: ta.HasTraits,
        traitName: str,
        widget: QWidget,
        propname: str = None,
        set_post_init=False,
    ) -> None:
        super().__init__()

        self._hasTrait = weakref.ref(hasTrait)
        self._trait_name = traitName

        self._widget = weakref.ref(widget)
        widget.destroyed.connect(self.finalizer)

        self.prop_name = get_user_prop_name(widget) if propname is None else propname

        if self.prop_name is None:
            return

        self.widget_prop = widget.metaObject().property(
            widget.metaObject().indexOfProperty(self.prop_name)
        )

        if self.widget_prop.isWritable():
            self._hasTrait().observe(self, self._trait_name, dispatch="ui")

            if set_post_init:
                val = getattr(self._hasTrait(), self._trait_name)
                widget.setProperty(self.prop_name, val)

        print(self.prop_name, self._trait_name)

    def _teardown(self):

        if self.finalizer.alive:
            if (t := self._hasTrait()) is not None:
                t.observe(self, self._trait_name, remove=True, dispatch="ui")

    def __call__(self, val) -> Any:
        print(self.prop_name, val)
        self._widget().setProperty(self.prop_name, val.new)


class TraitObjectModelBinder(Binder):
    def __init__(
        self, obj: ta.HasTraits, trait: str, model: ObjectModel, init_data=False
    ) -> None:

        super().__init__()
        self._guard = Guard()

        self._hasTrait = weakref.ref(obj)
        self._model = weakref.ref(model)

        model.destroyed.connect(self.finalizer)

        self._trait_name = trait

        if init_data:
            model.extend(getattr(obj, self._trait_name))

        self._observe_name = f"{self._trait_name}:items"

        obj.observe(self._list_set, self._trait_name, dispatch="ui")
        obj.observe(self._list_changed, self._observe_name, dispatch="ui")

        model.rowsInserted.connect(self._model_inserted)

    def _teardown(self):
        if self.finalizer.alive:

            if (t := self._hasTrait()) is not None:
                t.observe(self._list_set, self._trait_name, remove=True, dispatch="ui")
                t.observe(self._list_changed, self._observe_name, remove=True, dispatch="ui")

    def _model_inserted(self, parent: QModelIndex, first: int, last: int):
        if (t := self._hasTrait()) is not None and "trait" not in self._guard:
            m = self._model()
            new_data = [m.item(i).data(RawDataRole) for i in range(first, last + 1)]

            with self._guard("trait"):
                getattr(t, self._trait_name).extend(new_data)

    def _list_set(self, evt: TraitChangeEvent):
        if "trait" not in self._guard:
            self._model().clear()
            self._model().extend(evt.new)

    def _list_changed(self, evt: ListChangeEvent):
        if "trait" not in self._guard:
            if len(evt.added) > 0:
                with self._guard("trait"):
                    self._model().extend(evt.added)

            if len(evt.removed) > 0:
                self._model().remove(evt.removed)


def trait_widget_binding(
    hasTrait: ta.HasTraits,
    traitName: str,
    widget: QWidget,
    propname: str = None,
    set_trait_post=False,
    set_widget_post=False,
):
    if set_trait_post and set_widget_post:
        print("!Warning: undefined behavior when setting trait and widget after binding.")

    WidgetTraitBinding(hasTrait, traitName, widget, propname, set_trait_post)
    TraitWidgetBinding(hasTrait, traitName, widget, propname, set_widget_post)


class SOT_Model_Binder(Binder):
    def __init__(self, model: SOT_Model, stack: TaskHierarchy) -> None:
        super().__init__()

        self._guard = Guard()

        self._model = weakref.ref(model, self.finalizer)
        model.destroyed.connect(self.finalizer)

        self._stack = weakref.ref(stack, self.finalizer)

        model.clear()
        model.init_model(stack)

        model.task_inserted.connect(self._task_inserted)
        model.level_inserted.connect(self._level_inserted)
        model.task_removed.connect(self._task_removed)
        model.level_removed.connect(self._level_removed)

        ## TODO LISTENERS

        # stack.observe(self._stack_set, "levels", dispatch="ui")
        # stack.observe(self._stack_changed, "levels:items", dispatch="ui")
        # stack.observe(self._level_changed, "levels:items:items", dispatch="ui")

    def _teardown(self):
        logging.debug("teardown sot binder")
        if self.finalizer.alive:
            if (stack := self._stack()) is not None:
                pass
                # stack.observe(self._stack_set, "levels", remove=True)
                # stack.observe(self._stack_changed, "levels:items", remove=True)
                # stack.observe(self._level_changed, "levels:items:items")

    # changes from model

    def _level_inserted(self, tasks: list[Task], level: int):
        with self._guard(self):
            self._stack().levels.insert(level, tasks)

    def _task_inserted(self, task: Task, level: int):
        with self._guard(self):
            self._stack().levels[level].append(task)

    def _task_removed(self, level: int, task: int):
        with self._guard(self):
            self._stack().levels[level].pop(task)

    def _level_removed(self, level: int):
        print("remove level ", level)
        with self._guard(self):
            x = self._stack().levels[level]
            self._stack().levels.pop(level)

    # changes from list

    def _stack_set(self, event: TraitChangeEvent):
        if self not in self._guard:
            logging.debug(f"stack set {event.new}")
            model = self._model()
            model.clear()
            model.insert_level_rows(levels=event.new)

    def _stack_changed(self, event: ListChangeEvent):
        """Handle changes in the task hierarchy's levels list"""
        if self not in self._guard:
            logging.debug(f"stack chagne {event}")
            self._model().removeRows(event.index, len(event.removed))
            self._model().insert_level_rows(event.added, row=event.index)

    def _level_changed(self, event: ListChangeEvent):
        """Handle changes in a level's tasks list"""

        if self not in self._guard:
            logging.debug(f"level change {event}")
            model = self._model()
            level = model.itemFromIndex(self.level_index(event.object))
            level.removeRows(event.index, len(event.removed))
            level.removeRows(event.index, len(event.removed))


from stack_of_tasks.ui.model.object_model import ObjectModel
from stack_of_tasks.ui.model.sot_model import SOT_Model
