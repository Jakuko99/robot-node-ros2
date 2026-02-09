#!/usr/bin/env python3
"""
Random Environment Generator for Robot Mapping
Generates random indoor environments and exports to Ignition Gazebo .world format
"""

import random
import os
from dataclasses import dataclass
from typing import List, Tuple
import xml.etree.ElementTree as ET
from xml.dom import minidom


@dataclass
class Room:
    x: float
    y: float
    width: float
    height: float
    num_exits: int
    exits: List[Tuple[str, float]]

    def center(self):
        return (self.x + self.width / 2, self.y + self.height / 2)


class EnvironmentGenerator:
    def __init__(
        self, width=30, height=30, num_rooms=8, wall_height=2.5, wall_thickness=0.15
    ):
        self.width = width
        self.height = height
        self.num_rooms = num_rooms
        self.wall_height = wall_height
        self.wall_thickness = wall_thickness
        self.rooms = []
        self.walls = []

    def generate(self):
        self._generate_rooms()
        self._optimize_doorways()
        self._add_room_walls()
        self._add_outer_walls()
        self._center_environment()

    def _generate_rooms(self):
        attempts = 0
        max_attempts = 10000

        while len(self.rooms) < self.num_rooms and attempts < max_attempts:
            attempts += 1
            room_width = random.uniform(2.5, 6)
            room_height = random.uniform(2.5, 6)

            if len(self.rooms) == 0:
                # Place first room at random position to allow growth in all directions
                margin = 0.5
                x = random.uniform(margin, self.width - room_width - margin)
                y = random.uniform(margin, self.height - room_height - margin)
                new_room = Room(x, y, room_width, room_height, 0, [])
                self.rooms.append(new_room)
            else:
                # Try to place room adjacent to existing rooms
                placed = False
                for attempt in range(150):
                    base_room = random.choice(self.rooms)
                    side = random.choice(["north", "south", "east", "west"])

                    # Calculate position to place room exactly adjacent (touching)
                    # Use more aggressive alignment to fill space better
                    if side == "north":
                        # Place above, choose random x alignment
                        align = random.uniform(0, 1)
                        if align < 0.25:
                            # Align left edges
                            x = base_room.x
                        elif align < 0.5:
                            # Align right edges
                            x = base_room.x + base_room.width - room_width
                        elif align < 0.75:
                            # Center alignment
                            x = base_room.x + (base_room.width - room_width) / 2
                        else:
                            # Random offset for better space filling
                            max_offset = max(
                                0.1, min(base_room.width, room_width) - 0.1
                            )
                            x = base_room.x + random.uniform(
                                0, base_room.width - max_offset
                            )
                        y = base_room.y + base_room.height
                    elif side == "south":
                        # Place below
                        align = random.uniform(0, 1)
                        if align < 0.25:
                            x = base_room.x
                        elif align < 0.5:
                            x = base_room.x + base_room.width - room_width
                        elif align < 0.75:
                            x = base_room.x + (base_room.width - room_width) / 2
                        else:
                            max_offset = max(
                                0.1, min(base_room.width, room_width) - 0.1
                            )
                            x = base_room.x + random.uniform(
                                0, base_room.width - max_offset
                            )
                        y = base_room.y - room_height
                    elif side == "east":
                        # Place to the right
                        align = random.uniform(0, 1)
                        if align < 0.25:
                            y = base_room.y
                        elif align < 0.5:
                            y = base_room.y + base_room.height - room_height
                        elif align < 0.75:
                            y = base_room.y + (base_room.height - room_height) / 2
                        else:
                            max_offset = max(
                                0.1, min(base_room.height, room_height) - 0.1
                            )
                            y = base_room.y + random.uniform(
                                0, base_room.height - max_offset
                            )
                        x = base_room.x + base_room.width
                    else:  # west
                        # Place to the left
                        align = random.uniform(0, 1)
                        if align < 0.25:
                            y = base_room.y
                        elif align < 0.5:
                            y = base_room.y + base_room.height - room_height
                        elif align < 0.75:
                            y = base_room.y + (base_room.height - room_height) / 2
                        else:
                            max_offset = max(
                                0.1, min(base_room.height, room_height) - 0.1
                            )
                            y = base_room.y + random.uniform(
                                0, base_room.height - max_offset
                            )
                        x = base_room.x - room_width

                    # Check if within bounds - use minimal margin to maximize space usage
                    margin = 0.1
                    if (
                        x < margin
                        or y < margin
                        or x + room_width > self.width - margin
                        or y + room_height > self.height - margin
                    ):
                        continue

                    new_room = Room(x, y, room_width, room_height, 0, [])
                    if not self._overlaps_existing_room(new_room):
                        # Add doorways based on adjacent rooms
                        self._find_adjacent_rooms(new_room)
                        self.rooms.append(new_room)
                        placed = True
                        break

                # If we couldn't place adjacent, skip this attempt
                if not placed:
                    continue

        print(f"Generated {len(self.rooms)} rooms")

    def _overlaps_existing_room(self, new_room, min_spacing=0.0):
        """Check if new_room overlaps with existing rooms interior (not just touching edges)."""
        for room in self.rooms:
            # Rooms are separate if any gap exists
            if not (
                new_room.x + new_room.width + min_spacing <= room.x
                or new_room.x >= room.x + room.width + min_spacing
                or new_room.y + new_room.height + min_spacing <= room.y
                or new_room.y >= room.y + room.height + min_spacing
            ):
                return True

            # Additional check: prevent one room from being completely inside another
            # Check if new_room is inside existing room
            if (
                new_room.x > room.x
                and new_room.x + new_room.width < room.x + room.width
                and new_room.y > room.y
                and new_room.y + new_room.height < room.y + room.height
            ):
                return True

            # Check if existing room is inside new_room
            if (
                room.x > new_room.x
                and room.x + room.width < new_room.x + new_room.width
                and room.y > new_room.y
                and room.y + room.height < new_room.y + new_room.height
            ):
                return True
        return False

    def _find_adjacent_rooms(self, new_room):
        """Find rooms adjacent to this new room and potentially create doorways"""
        threshold = 0.5  # How close rooms need to be to have a doorway
        potential_connections = []

        for room in self.rooms:
            # Check if rooms are adjacent on each side and store potential connections
            # North side of new_room touches south side of room
            if abs((new_room.y + new_room.height) - room.y) < threshold and not (
                new_room.x + new_room.width < room.x or new_room.x > room.x + room.width
            ):
                overlap_start = max(new_room.x, room.x)
                overlap_end = min(new_room.x + new_room.width, room.x + room.width)
                door_pos = (overlap_start + overlap_end) / 2 - new_room.x
                room_door_pos = (overlap_start + overlap_end) / 2 - room.x
                potential_connections.append(
                    ("north", door_pos, room, "south", room_door_pos)
                )

            # South side of new_room touches north side of room
            elif abs(new_room.y - (room.y + room.height)) < threshold and not (
                new_room.x + new_room.width < room.x or new_room.x > room.x + room.width
            ):
                overlap_start = max(new_room.x, room.x)
                overlap_end = min(new_room.x + new_room.width, room.x + room.width)
                door_pos = (overlap_start + overlap_end) / 2 - new_room.x
                room_door_pos = (overlap_start + overlap_end) / 2 - room.x
                potential_connections.append(
                    ("south", door_pos, room, "north", room_door_pos)
                )

            # East side of new_room touches west side of room
            elif abs((new_room.x + new_room.width) - room.x) < threshold and not (
                new_room.y + new_room.height < room.y
                or new_room.y > room.y + room.height
            ):
                overlap_start = max(new_room.y, room.y)
                overlap_end = min(new_room.y + new_room.height, room.y + room.height)
                door_pos = (overlap_start + overlap_end) / 2 - new_room.y
                room_door_pos = (overlap_start + overlap_end) / 2 - room.y
                potential_connections.append(
                    ("east", door_pos, room, "west", room_door_pos)
                )

            # West side of new_room touches east side of room
            elif abs(new_room.x - (room.x + room.width)) < threshold and not (
                new_room.y + new_room.height < room.y
                or new_room.y > room.y + room.height
            ):
                overlap_start = max(new_room.y, room.y)
                overlap_end = min(new_room.y + new_room.height, room.y + room.height)
                door_pos = (overlap_start + overlap_end) / 2 - new_room.y
                room_door_pos = (overlap_start + overlap_end) / 2 - room.y
                potential_connections.append(
                    ("west", door_pos, room, "east", room_door_pos)
                )

        # Limit number of connections to avoid too many passthrough rooms
        # Always create at least 1 connection, up to 3 max (avoid 4-way intersections)
        if potential_connections:
            max_connections = min(3, len(potential_connections))
            # If room already has some connections or there are many potential ones, limit it
            if len(new_room.exits) > 0 or len(potential_connections) > 2:
                max_connections = random.randint(1, min(2, len(potential_connections)))

            # Randomly select which connections to create
            selected = random.sample(
                potential_connections, min(max_connections, len(potential_connections))
            )
            for new_side, new_pos, other_room, other_side, other_pos in selected:
                new_room.exits.append((new_side, new_pos))
                other_room.exits.append((other_side, other_pos))

    def _optimize_doorways(self):
        """Ensure every room has at least one entrance and fix isolated rooms"""
        threshold = 0.5

        for room in self.rooms:
            if len(room.exits) == 0:
                # Room has no exits, find closest adjacent room and force a connection
                best_connection = None

                for other_room in self.rooms:
                    if other_room == room:
                        continue

                    # Check all four sides for adjacency
                    # North side
                    if abs((room.y + room.height) - other_room.y) < threshold and not (
                        room.x + room.width < other_room.x
                        or room.x > other_room.x + other_room.width
                    ):
                        overlap_start = max(room.x, other_room.x)
                        overlap_end = min(
                            room.x + room.width, other_room.x + other_room.width
                        )
                        if overlap_end > overlap_start:
                            door_pos = (overlap_start + overlap_end) / 2 - room.x
                            other_door_pos = (
                                overlap_start + overlap_end
                            ) / 2 - other_room.x
                            best_connection = (
                                "north",
                                door_pos,
                                other_room,
                                "south",
                                other_door_pos,
                            )
                            break

                    # South side
                    elif abs(
                        room.y - (other_room.y + other_room.height)
                    ) < threshold and not (
                        room.x + room.width < other_room.x
                        or room.x > other_room.x + other_room.width
                    ):
                        overlap_start = max(room.x, other_room.x)
                        overlap_end = min(
                            room.x + room.width, other_room.x + other_room.width
                        )
                        if overlap_end > overlap_start:
                            door_pos = (overlap_start + overlap_end) / 2 - room.x
                            other_door_pos = (
                                overlap_start + overlap_end
                            ) / 2 - other_room.x
                            best_connection = (
                                "south",
                                door_pos,
                                other_room,
                                "north",
                                other_door_pos,
                            )
                            break

                    # East side
                    elif abs((room.x + room.width) - other_room.x) < threshold and not (
                        room.y + room.height < other_room.y
                        or room.y > other_room.y + other_room.height
                    ):
                        overlap_start = max(room.y, other_room.y)
                        overlap_end = min(
                            room.y + room.height, other_room.y + other_room.height
                        )
                        if overlap_end > overlap_start:
                            door_pos = (overlap_start + overlap_end) / 2 - room.y
                            other_door_pos = (
                                overlap_start + overlap_end
                            ) / 2 - other_room.y
                            best_connection = (
                                "east",
                                door_pos,
                                other_room,
                                "west",
                                other_door_pos,
                            )
                            break

                    # West side
                    elif abs(
                        room.x - (other_room.x + other_room.width)
                    ) < threshold and not (
                        room.y + room.height < other_room.y
                        or room.y > other_room.y + other_room.height
                    ):
                        overlap_start = max(room.y, other_room.y)
                        overlap_end = min(
                            room.y + room.height, other_room.y + other_room.height
                        )
                        if overlap_end > overlap_start:
                            door_pos = (overlap_start + overlap_end) / 2 - room.y
                            other_door_pos = (
                                overlap_start + overlap_end
                            ) / 2 - other_room.y
                            best_connection = (
                                "west",
                                door_pos,
                                other_room,
                                "east",
                                other_door_pos,
                            )
                            break

                # Create the connection if one was found
                if best_connection:
                    side, pos, other_room, other_side, other_pos = best_connection
                    room.exits.append((side, pos))
                    other_room.exits.append((other_side, other_pos))
                    print(f"  Added forced doorway to isolated room")

    def _add_room_walls(self):
        """Add walls for all rooms with doorways where they connect"""
        for room in self.rooms:
            self._add_single_room_walls(room)

    def _add_single_room_walls(self, room):
        door_width = 1.0
        sides = {
            "north": (room.x, room.y + room.height, room.width, 0),
            "south": (room.x, room.y, room.width, 0),
            "east": (room.x + room.width, room.y, 0, room.height),
            "west": (room.x, room.y, 0, room.height),
        }

        for side, (base_x, base_y, length_x, length_y) in sides.items():
            exits_on_side = sorted([pos for dir, pos in room.exits if dir == side])

            if not exits_on_side:
                if length_x > 0:
                    self.walls.append((base_x, base_y, length_x, self.wall_thickness))
                else:
                    self.walls.append((base_x, base_y, self.wall_thickness, length_y))
            else:
                is_horizontal = length_x > 0
                total_length = length_x if is_horizontal else length_y

                if exits_on_side[0] - door_width / 2 > 0.1:
                    if is_horizontal:
                        wall_len = exits_on_side[0] - door_width / 2
                        self.walls.append(
                            (base_x, base_y, wall_len, self.wall_thickness)
                        )
                    else:
                        wall_len = exits_on_side[0] - door_width / 2
                        self.walls.append(
                            (base_x, base_y, self.wall_thickness, wall_len)
                        )

                for i in range(len(exits_on_side) - 1):
                    start = exits_on_side[i] + door_width / 2
                    end = exits_on_side[i + 1] - door_width / 2
                    if end - start > 0.1:
                        if is_horizontal:
                            self.walls.append(
                                (
                                    base_x + start,
                                    base_y,
                                    end - start,
                                    self.wall_thickness,
                                )
                            )
                        else:
                            self.walls.append(
                                (
                                    base_x,
                                    base_y + start,
                                    self.wall_thickness,
                                    end - start,
                                )
                            )

                if total_length - (exits_on_side[-1] + door_width / 2) > 0.1:
                    start = exits_on_side[-1] + door_width / 2
                    if is_horizontal:
                        self.walls.append(
                            (
                                base_x + start,
                                base_y,
                                total_length - start,
                                self.wall_thickness,
                            )
                        )
                    else:
                        self.walls.append(
                            (
                                base_x,
                                base_y + start,
                                self.wall_thickness,
                                total_length - start,
                            )
                        )

    def _add_outer_walls(self):
        # Add outer boundary walls at the edges
        margin = 0.0
        self.walls.append(
            (margin, self.height - margin, self.width - 2 * margin, self.wall_thickness)
        )
        self.walls.append(
            (margin, margin, self.width - 2 * margin, self.wall_thickness)
        )
        self.walls.append(
            (self.width - margin, margin, self.wall_thickness, self.height - 2 * margin)
        )
        self.walls.append(
            (margin, margin, self.wall_thickness, self.height - 2 * margin)
        )

    def _center_environment(self):
        """Shift all coordinates so environment is centered at (0, 0)"""
        offset_x = self.width / 2
        offset_y = self.height / 2

        # Center rooms
        for room in self.rooms:
            room.x -= offset_x
            room.y -= offset_y

        # Center walls
        centered_walls = []
        for x, y, w, h in self.walls:
            centered_walls.append((x - offset_x, y - offset_y, w, h))
        self.walls = centered_walls

    def export_to_world(self, filename, include_robots=True):
        sdf = ET.Element("sdf", version="1.9")
        world = ET.SubElement(sdf, "world", name="random_environment")

        # Physics configuration
        physics = ET.SubElement(world, "physics", name="default_physics", type="ode")
        physics.set("default", "false")
        ET.SubElement(physics, "max_step_size").text = "0.001"
        ET.SubElement(physics, "real_time_factor").text = "1"
        ET.SubElement(physics, "real_time_update_rate").text = "1000"

        # Plugins
        ET.SubElement(
            world,
            "plugin",
            name="ignition::gazebo::systems::Physics",
            filename="ignition-gazebo-physics-system",
        )
        ET.SubElement(
            world,
            "plugin",
            name="ignition::gazebo::systems::SceneBroadcaster",
            filename="ignition-gazebo-scene-broadcaster-system",
        )
        sensors_plugin = ET.SubElement(
            world,
            "plugin",
            name="gz::sim::systems::Sensors",
            filename="ignition-gazebo-sensors-system",
        )
        ET.SubElement(sensors_plugin, "render_engine").text = "ogre1"

        # GUI configuration
        gui = ET.SubElement(world, "gui", fullscreen="false")
        self._add_gui_plugins(gui)

        # Gravity and other world properties
        ET.SubElement(world, "gravity").text = "0 0 -9.8"
        ET.SubElement(world, "magnetic_field").text = "6e-06 2.3e-05 -4.2e-05"
        atmosphere = ET.SubElement(world, "atmosphere", type="adiabatic")

        # Scene
        scene = ET.SubElement(world, "scene")
        ET.SubElement(scene, "ambient").text = "0.4 0.4 0.4 1"
        ET.SubElement(scene, "background").text = "0.7 0.7 0.7 1"
        ET.SubElement(scene, "shadows").text = "true"

        self._add_ground_plane(world)

        for i, (x, y, width, height) in enumerate(self.walls):
            self._add_wall_model(world, f"wall_{i}", x, y, width, height)

        # Add robot includes (optional)
        if include_robots:
            self._add_robot_includes(world)

        # Add directional light
        sun = ET.SubElement(world, "light", type="directional", name="sun")
        ET.SubElement(sun, "pose").text = "0 0 10 0 -0 0"
        ET.SubElement(sun, "cast_shadows").text = "true"
        ET.SubElement(sun, "intensity").text = "1"
        ET.SubElement(sun, "direction").text = "-0.5 0.1 -0.9"
        ET.SubElement(sun, "diffuse").text = "0.8 0.8 0.8 1"
        ET.SubElement(sun, "specular").text = "0.2 0.2 0.2 1"
        attenuation = ET.SubElement(sun, "attenuation")
        ET.SubElement(attenuation, "range").text = "1000"
        ET.SubElement(attenuation, "linear").text = "0.01"
        ET.SubElement(attenuation, "constant").text = "0.90000000000000002"
        ET.SubElement(attenuation, "quadratic").text = "0.001"
        spot = ET.SubElement(sun, "spot")
        ET.SubElement(spot, "inner_angle").text = "0"
        ET.SubElement(spot, "outer_angle").text = "0"
        ET.SubElement(spot, "falloff").text = "0"

        xml_str = ET.tostring(sdf, encoding="unicode")
        dom = minidom.parseString(xml_str)
        pretty_xml = dom.toprettyxml(indent="  ")
        lines = [line for line in pretty_xml.split("\n") if line.strip()]

        with open(filename, "w") as f:
            f.write("\n".join(lines))

    def _add_gui_plugins(self, gui):
        """Add GUI plugins configuration"""
        # 3D View plugin
        view_plugin = ET.SubElement(
            gui, "plugin", name="3D View", filename="MinimalScene"
        )
        view_gui = ET.SubElement(view_plugin, "ignition-gui")
        ET.SubElement(view_gui, "title").text = "3D View"
        prop = ET.SubElement(view_gui, "property", type="bool", key="showTitleBar")
        prop.text = "false"
        prop = ET.SubElement(view_gui, "property", type="string", key="state")
        prop.text = "docked"
        ET.SubElement(view_plugin, "engine").text = "ogre2"
        ET.SubElement(view_plugin, "scene").text = "scene"
        ET.SubElement(view_plugin, "ambient_light").text = "0.4 0.4 0.4"
        ET.SubElement(view_plugin, "background_color").text = "0.8 0.8 0.8"
        ET.SubElement(view_plugin, "camera_pose").text = "-6 0 6 0 0.5 0"

        # Entity context menu
        context_plugin = ET.SubElement(
            gui,
            "plugin",
            name="Entity context menu",
            filename="EntityContextMenuPlugin",
        )
        context_gui = ET.SubElement(context_plugin, "ignition-gui")
        ET.SubElement(context_gui, "property", key="state", type="string").text = (
            "floating"
        )
        ET.SubElement(context_gui, "property", key="width", type="double").text = "5"
        ET.SubElement(context_gui, "property", key="height", type="double").text = "5"
        ET.SubElement(context_gui, "property", key="showTitleBar", type="bool").text = (
            "false"
        )

        # Scene Manager
        scene_plugin = ET.SubElement(
            gui, "plugin", name="Scene Manager", filename="GzSceneManager"
        )
        scene_gui = ET.SubElement(scene_plugin, "ignition-gui")
        ET.SubElement(scene_gui, "property", key="resizable", type="bool").text = (
            "false"
        )
        ET.SubElement(scene_gui, "property", key="width", type="double").text = "5"
        ET.SubElement(scene_gui, "property", key="height", type="double").text = "5"
        ET.SubElement(scene_gui, "property", key="state", type="string").text = (
            "floating"
        )
        ET.SubElement(scene_gui, "property", key="showTitleBar", type="bool").text = (
            "false"
        )

        # Interactive view control
        interactive_plugin = ET.SubElement(
            gui,
            "plugin",
            name="Interactive view control",
            filename="InteractiveViewControl",
        )
        interactive_gui = ET.SubElement(interactive_plugin, "ignition-gui")
        ET.SubElement(
            interactive_gui, "property", key="resizable", type="bool"
        ).text = "false"
        ET.SubElement(interactive_gui, "property", key="width", type="double").text = (
            "5"
        )
        ET.SubElement(interactive_gui, "property", key="height", type="double").text = (
            "5"
        )
        ET.SubElement(interactive_gui, "property", key="state", type="string").text = (
            "floating"
        )
        ET.SubElement(
            interactive_gui, "property", key="showTitleBar", type="bool"
        ).text = "false"

        # Camera Tracking
        camera_plugin = ET.SubElement(
            gui, "plugin", name="Camera Tracking", filename="CameraTracking"
        )
        camera_gui = ET.SubElement(camera_plugin, "ignition-gui")
        ET.SubElement(camera_gui, "property", key="resizable", type="bool").text = (
            "false"
        )
        ET.SubElement(camera_gui, "property", key="width", type="double").text = "5"
        ET.SubElement(camera_gui, "property", key="height", type="double").text = "5"
        ET.SubElement(camera_gui, "property", key="state", type="string").text = (
            "floating"
        )
        ET.SubElement(camera_gui, "property", key="showTitleBar", type="bool").text = (
            "false"
        )

        # World control
        control_plugin = ET.SubElement(
            gui, "plugin", name="World control", filename="WorldControl"
        )
        control_gui = ET.SubElement(control_plugin, "ignition-gui")
        ET.SubElement(control_gui, "title").text = "World control"
        ET.SubElement(control_gui, "property", type="bool", key="showTitleBar").text = (
            "false"
        )
        ET.SubElement(control_gui, "property", type="bool", key="resizable").text = (
            "false"
        )
        ET.SubElement(control_gui, "property", type="double", key="height").text = "72"
        ET.SubElement(control_gui, "property", type="double", key="width").text = "121"
        ET.SubElement(control_gui, "property", type="double", key="z").text = "1"
        ET.SubElement(control_gui, "property", type="string", key="state").text = (
            "floating"
        )
        anchors = ET.SubElement(control_gui, "anchors", target="3D View")
        ET.SubElement(anchors, "line", own="left", target="left")
        ET.SubElement(anchors, "line", own="bottom", target="bottom")
        ET.SubElement(control_plugin, "play_pause").text = "true"
        ET.SubElement(control_plugin, "step").text = "true"
        ET.SubElement(control_plugin, "start_paused").text = "true"
        ET.SubElement(control_plugin, "use_event").text = "true"

        # World stats
        stats_plugin = ET.SubElement(
            gui, "plugin", name="World stats", filename="WorldStats"
        )
        stats_gui = ET.SubElement(stats_plugin, "ignition-gui")
        ET.SubElement(stats_gui, "title").text = "World stats"
        ET.SubElement(stats_gui, "property", type="bool", key="showTitleBar").text = (
            "false"
        )
        ET.SubElement(stats_gui, "property", type="bool", key="resizable").text = (
            "false"
        )
        ET.SubElement(stats_gui, "property", type="double", key="height").text = "110"
        ET.SubElement(stats_gui, "property", type="double", key="width").text = "290"
        ET.SubElement(stats_gui, "property", type="double", key="z").text = "1"
        ET.SubElement(stats_gui, "property", type="string", key="state").text = (
            "floating"
        )
        anchors = ET.SubElement(stats_gui, "anchors", target="3D View")
        ET.SubElement(anchors, "line", own="right", target="right")
        ET.SubElement(anchors, "line", own="bottom", target="bottom")
        ET.SubElement(stats_plugin, "sim_time").text = "true"
        ET.SubElement(stats_plugin, "real_time").text = "true"
        ET.SubElement(stats_plugin, "real_time_factor").text = "true"
        ET.SubElement(stats_plugin, "iterations").text = "true"

        # Visualize Lidar plugins
        ET.SubElement(gui, "plugin", name="Visualize Lidar", filename="VisualizeLidar")
        # ET.SubElement(gui, "plugin", name="Visualize Lidar", filename="VisualizeLidar")

        # Component inspector
        inspector_plugin = ET.SubElement(
            gui, "plugin", name="Component inspector", filename="ComponentInspector"
        )
        inspector_gui = ET.SubElement(inspector_plugin, "ignition-gui")
        ET.SubElement(inspector_gui, "property", type="string", key="state").text = (
            "docked"
        )

        # Entity tree
        tree_plugin = ET.SubElement(
            gui, "plugin", name="Entity tree", filename="EntityTree"
        )
        tree_gui = ET.SubElement(tree_plugin, "ignition-gui")
        ET.SubElement(tree_gui, "property", type="string", key="state").text = "docked"

    def _add_robot_includes(self, world):
        """Add robot model includes"""
        # First robot
        include1 = ET.SubElement(world, "include")
        ET.SubElement(include1, "uri").text = "file://src/robot_sim/gazebo/kris_robot"
        ET.SubElement(include1, "name").text = "kris_robot1"
        ET.SubElement(include1, "pose").text = "0.0 0.0 0.5 0.0 0.0 3.14"

    def _add_ground_plane(self, world):
        model = ET.SubElement(world, "model", name="ground_plane")
        ET.SubElement(model, "static").text = "true"
        link = ET.SubElement(model, "link", name="link")
        collision = ET.SubElement(link, "collision", name="collision")
        geometry = ET.SubElement(collision, "geometry")
        plane = ET.SubElement(geometry, "plane")
        ET.SubElement(plane, "normal").text = "0 0 1"
        visual = ET.SubElement(link, "visual", name="visual")
        geometry = ET.SubElement(visual, "geometry")
        plane = ET.SubElement(geometry, "plane")
        ET.SubElement(plane, "normal").text = "0 0 1"
        ET.SubElement(plane, "size").text = f"{self.width * 2} {self.height * 2}"
        material = ET.SubElement(visual, "material")
        ET.SubElement(material, "ambient").text = "0.8 0.8 0.8 1"
        ET.SubElement(material, "diffuse").text = "0.8 0.8 0.8 1"
        ET.SubElement(material, "specular").text = "0.8 0.8 0.8 1"

    def _add_wall_model(self, world, name, x, y, width, height):
        center_x = x + width / 2
        center_y = y + height / 2
        center_z = self.wall_height / 2

        model = ET.SubElement(world, "model", name=name)
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = f"{center_x} {center_y} {center_z} 0 0 0"

        link = ET.SubElement(model, "link", name="link")
        collision = ET.SubElement(link, "collision", name="collision")
        geometry = ET.SubElement(collision, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = f"{width} {height} {self.wall_height}"

        visual = ET.SubElement(link, "visual", name="visual")
        geometry = ET.SubElement(visual, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = f"{width} {height} {self.wall_height}"

        material = ET.SubElement(visual, "material")
        ET.SubElement(material, "ambient").text = "0.7 0.7 0.7 1"
        ET.SubElement(material, "diffuse").text = "0.7 0.7 0.7 1"
        ET.SubElement(material, "specular").text = "0.1 0.1 0.1 1"


if __name__ == "__main__":
    # Remove seed to get different environments each run
    # random.seed(42)
    # np.random.seed(42)

    generator = EnvironmentGenerator(width=20, height=20, num_rooms=20)
    generator.generate()
    generator.export_to_world(
        f"{os.path.dirname(__file__)}/../gazebo/random_environment.sdf",
        include_robots=True,
    )
