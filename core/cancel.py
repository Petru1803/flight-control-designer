"""Cooperative cancellation of long computations (one background job at a time)."""
import threading

_flag = threading.Event()


class Cancelled(Exception):
    pass


def reset():
    _flag.clear()


def request():
    _flag.set()


def check():
    if _flag.is_set():
        raise Cancelled("Cancelled by the user")
