"""Expose a config dataclass as ROS parameters.

Not an executable node. The tunable settings of terrain_map.py,
trail_detector.py and route_planner.py live in dataclasses next to the code
that uses them, each with a comment saying what it does and why it has its
default. declare_config() declares one ROS parameter per field (with the
dataclass default) and fills the dataclass with the final values, so any
setting can be overridden from a YAML file or --ros-args without the
defaults being written out a second time.

ROS 2 enforces the default's type, so overrides of float settings must be
written as floats (1.0, not 1).
"""

from dataclasses import fields


def declare_config(node, config, prefix: str = ''):
    """Declare every field of `config` as parameter `<prefix><field>`, then read
    the values back into `config` (in place) and return it."""
    for field in fields(config):
        default = getattr(config, field.name)
        setattr(config, field.name, node.declare_parameter(prefix + field.name, default).value)
    return config
