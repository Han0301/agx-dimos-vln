"""Forward native stdout/stderr while file capture uses a separate disk queue."""

import sys
from pathlib import Path

from ._writer import BackgroundWriter, ConsoleFile


def relay(path):
    target = ConsoleFile(path)
    try:
        writer = BackgroundWriter(target.write, target.close, capacity=64, name="console-log")
    except RuntimeError:
        writer = None
    output = sys.stdout.buffer
    try:
        while data := sys.stdin.buffer.read1(65536):
            if output is not None:
                try:
                    output.write(data)
                    output.flush()
                except OSError:
                    output = None
            if writer is not None:
                writer.offer(data)
    finally:
        if writer is not None:
            writer.close()


if __name__ == "__main__":
    relay(Path(sys.argv[1]))
