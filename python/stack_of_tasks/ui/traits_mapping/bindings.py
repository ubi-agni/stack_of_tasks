from __future__ import annotations

import weakref

import typing
from typing import Any

import traits.api as ta
from PyQt5.QtCore import pyqtBoundSignal
from PyQt5.QtWidgets import QWidget
from traits.observation.events import ListChangeEvent, TraitChangeEvent

from stack_of_tasks.ui.model.object_model import ObjectModel

BINDERS = []  # prevent object from being gc-ed


def set_user_property(widget: QWidget, value: Any) -> None:
    if (prop_name := get_user_prop_name(widget)) is not None:
        widget.setProperty(prop_name, value)
    else:
        print(f"{widget} has no user property.")


def get_user_property(widget: QWidget) -> Any:
    if (prop_name := get_user_prop_name(widget)) is not None:
        # print(f"get_user_property {widget, prop_name}")
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


class WidgetTraitBinding:
    def __init__(
        self,
        hasTrait: ta.HasTraits,
        traitName: str,
        widget: QWidget,
        propname: str = None,
        set_post_init=False,
    ) -> None:
        self._hasTrait = hasTrait
        self._trait_name = traitName
        self._widget = widget
        self._array = isinstance(self._hasTrait.trait(traitName).trait_type, ta.Array)

        if propname is None:
            self.prop_name = get_user_prop_name(widget)

        if self.prop_name is None:
            return

        self.widget_prop = self._widget.metaObject().property(
            self._widget.metaObject().indexOfProperty(self.prop_name)
        )

        if self.widget_prop.hasNotifySignal():
            signal: pyqtBoundSignal = getattr(
                self._widget, str(self.widget_prop.notifySignal().name(), "utf-8")
            )

            signal.connect(self)

            if set_post_init:
                val = self._widget.property(self.prop_name)
                self(val)

    def __call__(self, value) -> Any:
        self._hasTrait.trait_set(**{self._trait_name: value})


class TraitWidgetBinding:
    def __init__(
        self,
        hasTrait: ta.HasTraits,
        traitName: str,
        widget: QWidget,
        propname: str = None,
        set_post_init=False,
    ) -> None:
        self._hasTrait = hasTrait
        self._trait_name = traitName
        self._widget = widget

        self.prop_name = get_user_prop_name(widget) if propname is None else propname

        if self.prop_name is None:
            return

        self.widget_prop = self._widget.metaObject().property(
            self._widget.metaObject().indexOfProperty(self.prop_name)
        )

        if self.widget_prop.isWritable():
            self._hasTrait.observe(self, self._trait_name)

            if set_post_init:
                val = getattr(self._hasTrait, self._trait_name)
                self._widget.setProperty(self.prop_name, val)

        self._widget.destroyed.connect(self._widget_removed)

    def _widget_removed(self):
        self._hasTrait.observe(self, self._trait_name, remove=True)

    def __call__(self, val) -> Any:
        print(self._trait_name, val.new)
        self._widget.setProperty(self.prop_name, val.new)


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


class TraitObjectModelBinder:
    def __init__(self, obj: ta.HasTraits, trait: str, model: ObjectModel) -> None:
        self._hasTrait = weakref.ref(obj, self._remove_listener)
        self._model = weakref.ref(model, self._remove_listener)
        self._model().destroyed.connect(self._remove_listener)

        self._trait_name = trait
        self.alive = True

        self._model()._objs = getattr(self._hasTrait(), self._trait_name)

        self._observe_name = f"{self._trait_name}:items"

        self._hasTrait().observe(self._list_set, self._trait_name)
        self._hasTrait().observe(self._data_changed, self._observe_name)
        # self._hasTrait().observe(self._item_changed, f"{self._observe_name}:display_name")

        self._finalizer_self = weakref.finalize(self, self._remove_listener)

    def _remove_listener(self, obj=None):
        if (t := self._hasTrait()) is not None and self.alive:
            t.observe(self._list_set, self._trait_name, remove=True)
            t.observe(self._data_changed, self._observe_name, remove=True)
            # t.observe(self._item_changed, f"{self._observe_name}:display_name", remove=True)

            self.alive = False

    def _list_set(self, evt: TraitChangeEvent):
        self._model.set_data_list(evt.new)

    def _data_changed(self, evt: ListChangeEvent):
        if (
            len(evt.added) > 0
        ):  # assume adding and removing from list cant happen at the same time
            self._model().items_inserted(evt.index)
        else:
            self._model().items_removed(evt.index)

    def _item_changed(self, evt: TraitChangeEvent):
        print(self._trait_name, evt.new)
        index = getattr(self._hasTrait(), self._trait_name).index(evt.object)
        self._model().item_changed(index)


class SyncTraitBinder:
    def __init__(
        self, inst_one: ta.HasTraits, trait_one: str, inst_two: ta.HasTraits, trait_two: str
    ) -> None:
        self._ta_one: ta.HasTraits = inst_one
        self._ta_two: ta.HasTraits = inst_two

        self._name_one: str = trait_one
        self._name_two: str = trait_two

        self._ta_one.sync_trait(self._name_one, self._ta_two, self._name_two)

    def remove(self):
        self._ta_one.sync_trait(self._name_one, self._ta_two, self._name_two, remove=True)
