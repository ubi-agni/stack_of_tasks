from __future__ import annotations

import typing
from typing import Any

import traits.api as ta
from PyQt5.QtCore import pyqtBoundSignal
from PyQt5.QtWidgets import QWidget


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
        self, hasTrait: ta.HasTraits, traitName: str, widget: QWidget, propname: str = None
    ) -> None:
        self._hasTrait = hasTrait
        self._trait_name = traitName
        self._widget = widget
        if propname is None:
            self.prop_name = get_user_prop_name(widget)

        self.widget_prop = self._widget.metaObject().property(
            self._widget.metaObject().indexOfProperty(self.prop_name)
        )

        if self.widget_prop.hasNotifySignal():
            signal: pyqtBoundSignal = getattr(
                self._widget, str(self.widget_prop.notifySignal().name(), "utf-8")
            )

            signal.connect(self)

    def __call__(self, value) -> Any:
        setattr(self._hasTrait, self._trait_name, value)


class TraitWidgetBinding:
    def __init__(
        self, hasTrait: ta.HasTraits, traitName: str, widget: QWidget, propname: str = None
    ) -> None:
        self._hasTrait = hasTrait
        self._trait_name = traitName
        self._widget = widget
        if propname is None:
            self.prop_name = get_user_prop_name(widget)

        self.widget_prop = self._widget.metaObject().property(
            self._widget.metaObject().indexOfProperty(self.prop_name)
        )

        if self.widget_prop.isWritable():
            self._hasTrait.observe(self, self._trait_name)

        self._widget.destroyed.connect(self._widget_removed)

    def _widget_removed(self):
        self._hasTrait.observe(self, self._trait_name, remove=True)

    def __call__(self, val) -> Any:
        self._widget.setProperty(self.prop_name, val.new)


def trait_widget_binding(
    hasTrait: ta.HasTraits, traitName: str, widget: QWidget, propname: str = None
):
    WidgetTraitBinding(hasTrait, traitName, widget, propname)
    TraitWidgetBinding(hasTrait, traitName, widget, propname)
