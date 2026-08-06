# HISPEC Fiber Switcher

A Python driver for the fiber-optic port switcher PLC used by HISPEC. The
switcher routes a fiber between a 13-position "Port A" selector and a
5-position "Port B" selector and inserts/retracts the connector at
whichever position is currently selected.

The PLC acts as a TCP *server*; this driver is the TCP *client*. Commands
are fixed-format ASCII hex strings documented by the vendor. Every command
this driver sends was checked byte-for-byte against the vendor's worked
examples.

## Installation

```bash
pip install .
```

For development:

```bash
pip install -e ".[dev]"
python -m pytest
```

## Usage

```python
from fiber_switcher import FiberSwitcher

switcher = FiberSwitcher()
switcher.connect("192.168.0.183", 9001)  # first unit; second unit uses a different port
try:
    switcher.home()  # always do this once after PLC power-on
    switcher.set_target_positions(port_a=5, port_b=2)
    switcher.move_to_target()
    switcher.insert_fiber()

    status = switcher.read_status()
    print(status)
finally:
    switcher.disconnect()
```

Or as a context manager:

```python
with FiberSwitcher() as switcher:
    switcher.connect("192.168.0.183", 9001)
    switcher.home()
```
