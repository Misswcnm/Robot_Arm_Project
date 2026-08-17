"""Small, dependency-free helpers for durable JSON records."""

import hashlib
import json
import os
import tempfile
from datetime import datetime, timezone


def now():
    return datetime.now(timezone.utc).isoformat()


def digest(path):
    hasher = hashlib.sha256()
    with open(os.path.expanduser(path), 'rb') as stream:
        for block in iter(lambda: stream.read(65536), b''):
            hasher.update(block)
    return hasher.hexdigest()


def atomic_json(path, value):
    """Replace one JSON file atomically after its contents reach disk."""
    path = os.path.expanduser(path)
    folder = os.path.dirname(path) or '.'
    os.makedirs(folder, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix='.tmp-', dir=folder)
    try:
        with os.fdopen(
                descriptor, 'w', encoding='utf-8') as stream:
            json.dump(
                value, stream, ensure_ascii=False, indent=2,
                sort_keys=True, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        # Persist the directory entry as well as the file contents. This is
        # important for station manifests surviving an unexpected power loss.
        try:
            directory = os.open(
                folder, os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        except OSError:
            # Some filesystems do not support directory fsync. The atomic
            # replacement itself has already completed safely.
            pass
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def load_json(path, default=None, strict=False):
    """Read JSON; strict mode distinguishes corruption from a missing file."""
    path = os.path.expanduser(path)
    try:
        with open(path, encoding='utf-8') as stream:
            return json.load(stream)
    except FileNotFoundError:
        return default
    except (OSError, ValueError) as error:
        if strict:
            raise RuntimeError(
                'cannot read valid JSON from %s: %s' % (path, error))
        return default
