"""Remove credentials from diagnostic strings and configuration snapshots."""

import re

_SECRET_VALUE = re.compile(
    r"""(?i)(["']?\b(?:api[-_ ]?key|access[-_ ]?token|token|password|secret|authorization)\b["']?\s*[:=]\s*)"""
    r"""(?:"[^"]*"|'[^']*'|bearer\s+[^\s&,;]+|[^\s&,;]+)"""
)
_BEARER = re.compile(r"(?i)\bbearer\s+[^\s&,;]+")
_URL_USERINFO = re.compile(r"(://)[^/@\s]+@")
_SECRET_KEYS = {
    "api_key", "apikey", "credential", "credential_file", "credentials", "credentials_file",
    "password", "secret", "token", "authorization", "secret_access_key",
}
_SECRET_SUFFIXES = ("_api_key", "_password", "_token", "_secret", "_authorization")


def redact_text(value):
    value = _URL_USERINFO.sub(r"\1[redacted]@", value)
    value = _SECRET_VALUE.sub(r"\1[redacted]", value)
    return _BEARER.sub("Bearer [redacted]", value)


def redact_config(value):
    if isinstance(value, dict):
        result = {}
        for key, item in value.items():
            normalized = str(key).lower().replace("-", "_")
            secret = normalized in _SECRET_KEYS or normalized.endswith(_SECRET_SUFFIXES)
            result[key] = "[redacted]" if secret else redact_config(item)
        return result
    if isinstance(value, (tuple, list)):
        return [redact_config(item) for item in value]
    return redact_text(value) if isinstance(value, str) else value
