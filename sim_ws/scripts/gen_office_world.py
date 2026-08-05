#!/usr/bin/env python3
"""Generate office.sdf: 5 rooms + corridor + doorways + shelves (box composition).

Layout (x: 0..30, y: 0..8):
  corridor y 0..2 along the whole length; 5 rooms of 6x6m above it (y 2..8).
  Each room has a 1.2 m doorway in the corridor wall, plus 2 shelves.
"""
import sys
from pathlib import Path

TH = 0.2          # wall thickness
H = 2.0           # wall height
ROOM_W = 6.0      # room width (x)
ROOM_D = 6.0      # room depth (y), rooms span y=2..8
CORRIDOR_Y = 2.0  # corridor top (y=2)
DOOR = 1.2        # doorway width


def wall(model, x, y, size, ry=0.0):
    sx, sy, sz = size
    return f"""    <model name="{model}">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
      <pose>{x} {y} {sz/2} 0 0 {ry}</pose>
    </model>
"""


def shelf(model, x, y):
    return f"""    <model name="{model}">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>2.5 0.4 2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>2.5 0.4 2</size></box></geometry>
          <material><ambient>0.4 0.5 0.6 1</ambient></material>
        </visual>
      </link>
      <pose>{x} {y} 1 0 0 0</pose>
    </model>
"""


def build():
    parts = [
        '<?xml version="1.0" ?>',
        '<!-- 办公室：5 间房 + 走廊 + 门洞 + 货架（由 scripts/gen_office_world.py 生成） -->',
        '<sdf version="1.6">',
        '  <world name="office">',
        '    <include><uri>model://sun</uri></include>',
        '    <include><uri>model://ground_plane</uri></include>',
        '    <physics type="ode">',
        '      <max_step_size>0.001</max_step_size>',
        '      <real_time_factor>1</real_time_factor>',
        '      <real_time_update_rate>1000</real_time_update_rate>',
        '    </physics>',
    ]

    # south outer wall (y=0) with 1.2 m entrance at x=15
    cx = 15.0
    parts.append(wall("wall_south_w", cx - 15.4 - DOOR / 2, 0, (cx - 15.4 - DOOR / 2, TH, H)))
    parts.append(wall("wall_south_e", cx + 15.4 + DOOR / 2, 0, (30.0 - (cx + 15.4) + DOOR / 2, TH, H)))

    # north outer wall (y=8)
    parts.append(wall("wall_north", 15, 8 + TH / 2, (30, TH, H)))

    # vertical partition walls at x = 0,6,...,30 spanning y 2..8
    for i in range(6):
        x = i * ROOM_W
        if x == 0.0:
            parts.append(wall(f"wall_x{i}", x - TH / 2, 5, (TH, ROOM_D, H)))
        elif x == 30.0:
            parts.append(wall(f"wall_x{i}", x + TH / 2, 5, (TH, ROOM_D, H)))
        else:
            parts.append(wall(f"wall_x{i}", x, 5, (TH, ROOM_D + TH, H)))

    # corridor inner wall (y=2) with 5 doorways
    for i in range(5):
        x0 = i * ROOM_W
        c = x0 + ROOM_W / 2
        parts.append(wall(f"wall_corr_w{i}", x0 + (c - DOOR / 2 - x0) / 2, CORRIDOR_Y,
                          (c - DOOR / 2 - x0, TH, H)))
        parts.append(wall(f"wall_corr_e{i}", (c + DOOR / 2) + (x0 + ROOM_W - c - DOOR / 2) / 2,
                          CORRIDOR_Y, (x0 + ROOM_W - c - DOOR / 2, TH, H)))

    # shelves: 2 per room
    for i in range(5):
        x0 = i * ROOM_W
        c = x0 + ROOM_W / 2
        parts.append(shelf(f"shelf_{i}_a", c - 2.0, 6.4))
        parts.append(shelf(f"shelf_{i}_b", c + 2.0, 3.6))

    parts.append('  </world>')
    parts.append('</sdf>\n')
    return "\n".join(parts)


if __name__ == "__main__":
    out = Path(__file__).resolve().parent.parent / "src" / "m20_description" / "worlds" / "office.sdf"
    out.write_text(build())
    print(f"wrote {out}")
    sys.exit(0)
