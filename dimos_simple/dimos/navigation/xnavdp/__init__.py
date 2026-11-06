"""Standalone XNavDP integration.

This package deliberately has no dependency on ``navdp_bridge`` or the
external NavDP checkout. The legacy ``--navigation-policy navdp`` path remains
in :mod:`dimos.navigation.navdp`.
"""

from dimos.navigation.xnavdp.navigator import (
    NavDPNavigator,
    XNavDPNavigator,
    xnavdp_navigator,
)

__all__ = ["NavDPNavigator", "XNavDPNavigator", "xnavdp_navigator"]
