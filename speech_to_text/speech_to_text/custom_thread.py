"""Flexible Thread File Class"""

import threading
import ctypes
from typing import Any


class CustomThread(threading.Thread):
    """Custom Thread Class"""

    def run(self) -> None:
        """target function of the thread class"""

        self._return = None

        try:
            if self._target:
                self._return = self._target(*self._args, **self._kwargs)

        finally:
            del self._target, self._args, self._kwargs

    def get_id(self) -> int:
        """Returns id of the respective thread"""

        if hasattr(self, "_thread_id"):
            return self._thread_id

        for id, thread in threading._active.items():
            if thread is self:
                self._thread_id = id
                return id

    def __raise_exception(self) -> None:
        thread_id = self.get_id()

        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
            ctypes.c_long(thread_id), ctypes.py_object(SystemExit)
        )

        if res > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 0)
            print("Exception raise failure")

    def terminate(self) -> None:
        """Thread terminate method"""

        self.__raise_exception()

    def join(self, timeout: int = None) -> Any:
        threading.Thread.join(self, timeout=timeout)
        return self._return
