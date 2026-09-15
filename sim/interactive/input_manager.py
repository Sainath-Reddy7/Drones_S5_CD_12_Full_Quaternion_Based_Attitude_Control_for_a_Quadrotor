"""Event-driven input manager for the interactive simulator.

Why events, not polling: the previous `keyboard.is_pressed` polling missed
keys when the loop stalled (viewer sync / slow frames) and gave no way to
distinguish taps from holds. This manager subscribes once to the global
keyboard hook and maintains a live pressed-set; the sim loop only READS it
(`poll()`), so input reliability is decoupled from frame rate.

Synthetic events (`inject`) drive the same code path for the automated
control-pipeline tests -- pressing W in a test is indistinguishable to the
manager from a real keypress.
"""
from __future__ import annotations

MOVE_KEYS = ("w", "a", "s", "d", "q", "e", "r", "f", "i", "k", "j", "l",
             "up", "down", "left", "right", "space")


class InputManager:
    def __init__(self, use_hook: bool = True):
        self.pressed: set[str] = set()
        self._taps: list[str] = []          # one-shot events since last poll
        self._hook = None
        if use_hook:
            import keyboard

            self._kb = keyboard
            self._hook = keyboard.hook(self._on_event, suppress=False)

    # -- event source ------------------------------------------------------
    def _on_event(self, event) -> None:
        name = (event.name or "").lower()
        if name == "space":
            name = "space"
        if event.event_type == "down":
            if name not in self.pressed:
                self._taps.append(name)
            self.pressed.add(name)
        elif event.event_type == "up":
            self.pressed.discard(name)

    # -- sim-loop API --------------------------------------------------------
    def is_pressed(self, key: str) -> bool:
        return key in self.pressed

    def poll(self) -> tuple[set[str], list[str]]:
        """Return (held-key set, one-shot taps since last poll)."""
        taps, self._taps = self._taps, []
        return set(self.pressed), taps

    def get_axes(self) -> dict[str, float]:
        """-1..1 per axis from held keys (for the debug HUD)."""
        ax = {
            "forward": float(self.is_pressed("w")) - float(self.is_pressed("s")),
            "right": float(self.is_pressed("d")) - float(self.is_pressed("a")),
            "yaw": float(self.is_pressed("e")) - float(self.is_pressed("q")),
            "climb": (float(self.is_pressed("r")) + float(self.is_pressed("up"))
                      - float(self.is_pressed("f")) - float(self.is_pressed("down"))),
        }
        return ax

    def reset(self) -> None:
        self.pressed.clear()
        self._taps.clear()

    def unhook(self) -> None:
        if self._hook is not None:
            self._kb.unhook(self._hook)
            self._hook = None

    # -- test injection --------------------------------------------------
    def inject(self, key: str, direction: str = "down") -> None:
        """Synthetic event through the identical handler (tests only)."""
        class _E:
            pass

        e = _E()
        e.name = key
        e.event_type = "down" if direction == "down" else "up"
        self._on_event(e)
