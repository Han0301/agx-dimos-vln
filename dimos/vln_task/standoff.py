"""Recognize explicit stopping distances in navigation instructions."""

import math
import re

_NUMBER = r"(?P<value>[+-]?(?:[0-9]+(?:\.[0-9]+)?|\.[0-9]+))"
_EN_UNIT = r"(?P<unit>centimet(?:ers?|res?)|met(?:ers?|res?)|cm|m)\b"
_ZH_UNIT = r"(?P<unit>厘米|公分|公尺|米|cm|m)"
_EN_APPROX = r"(?:(?:at|about|around|approximately|roughly)\s+)*"
_ZH_APPROX = r"(?:大约|大概|约|差不多)?\s*"

_PATTERNS = (
    re.compile(
        rf"\b(?:stop|halt|stand|stay|remain|keep|until\s+(?:you|the robot)\s+"
        rf"(?:are|is))\s+{_EN_APPROX}{_NUMBER}\s*{_EN_UNIT}"
        rf"\s*(?:away\b|from\b|(?:of\s+)?distance\b)",
        re.IGNORECASE,
    ),
    re.compile(
        rf"\b(?:approach|move|walk)\b[^.!?;,]{{0,48}}?\b(?:to\s+)?within\s+"
        rf"{_EN_APPROX}{_NUMBER}\s*{_EN_UNIT}",
        re.IGNORECASE,
    ),
    re.compile(
        rf"(?:停(?:在|到)?|靠近到|接近至|保持)\s*"
        rf"(?:(?:距离|距|离)[^0-9+\-。！？；，,.!?;]{{0,24}}?)?"
        rf"{_ZH_APPROX}{_NUMBER}\s*{_ZH_UNIT}(?![高宽长厚])",
        re.IGNORECASE,
    ),
    re.compile(
        rf"(?:距离|距|离)[^0-9+\-。！？；，,.!?;]{{0,24}}?{_ZH_APPROX}"
        rf"{_NUMBER}\s*{_ZH_UNIT}\s*(?:时|处)?\s*(?:就|再)?\s*停",
        re.IGNORECASE,
    ),
)
_CENTIMETERS = {"centimeter", "centimeters", "centimetre", "centimetres", "cm", "厘米", "公分"}


def requested_standoff(text):
    distances = [
        float(match["value"]) / (100 if match["unit"].lower() in _CENTIMETERS else 1)
        for pattern in _PATTERNS
        for match in pattern.finditer(text)
    ]
    if not distances:
        return None
    distance = distances[0]
    if not math.isfinite(distance) or distance <= 0:
        raise ValueError("Requested stopping distance must be positive and finite")
    if any(not math.isclose(distance, other) for other in distances[1:]):
        raise ValueError("Conflicting stopping distances")
    return distance
