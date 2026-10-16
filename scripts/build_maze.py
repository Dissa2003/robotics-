"""Add the assignment maze walls to the existing Gazebo world.

Coordinates are in metres, with the origin at the bottom-left
of Figure 1. Thin measurement-grid lines are not physical walls.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent.parent
WORLD_PATH = ROOT / "worlds" / "my_maze.world"

WALL_HEIGHT = 0.5
WALL_THICKNESS = 0.02

# Each segment is (x1, y1, x2, y2).
WALLS = [
    # Outer boundary.
    (0.0, 0.0, 3.0, 0.0),
    (0.0, 3.0, 3.0, 3.0),
    (0.0, 0.0, 0.0, 3.0),
    (3.0, 0.0, 3.0, 3.0),

    # Internal vertical walls.
    (0.5, 1.0, 0.5, 2.5),
    (1.0, 1.5, 1.0, 2.0),
    (1.0, 2.5, 1.0, 3.0),
    (1.5, 1.0, 1.5, 1.5),
    (1.5, 2.0, 1.5, 2.5),
    (2.0, 1.5, 2.0, 2.0),
    (2.0, 2.5, 2.0, 3.0),
    (2.5, 0.5, 2.5, 1.5),
    (2.5, 2.0, 2.5, 2.5),

    # Internal horizontal walls.
    (0.0, 2.0, 0.5, 2.0),
    (1.0, 2.0, 2.5, 2.0),
    (2.0, 1.5, 3.0, 1.5),
    (0.5, 1.0, 2.0, 1.0),
    (0.0, 0.5, 0.5, 0.5),
    (1.0, 0.5, 2.5, 0.5),
]


def add_wall(world, index, segment):
    x1, y1, x2, y2 = segment
    length = math.hypot(x2 - x1, y2 - y1)
    yaw = math.atan2(y2 - y1, x2 - x1)

    model = ET.SubElement(
        world, "model", name=f"assignment_wall_{index:02d}"
    )
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = (
        f"{(x1 + x2) / 2} {(y1 + y2) / 2} "
        f"{WALL_HEIGHT / 2} 0 0 {yaw}"
    )
    link = ET.SubElement(model, "link", name="wall_link")

    for kind in ("collision", "visual"):
        element = ET.SubElement(link, kind, name=f"wall_{kind}")
        geometry = ET.SubElement(element, "geometry")
        box = ET.SubElement(geometry, "box")
        # Small endpoint overlap prevents gaps at wall junctions.
        ET.SubElement(box, "size").text = (
            f"{length + WALL_THICKNESS} "
            f"{WALL_THICKNESS} {WALL_HEIGHT}"
        )

        if kind == "visual":
            material = ET.SubElement(element, "material")
            ET.SubElement(material, "ambient").text = "0.2 0.25 0.3 1"
            ET.SubElement(material, "diffuse").text = "0.3 0.35 0.4 1"


def main():
    tree = ET.parse(WORLD_PATH)
    world = tree.getroot().find("world")
    if world is None:
        raise RuntimeError("World element is missing.")

    # Re-running this script replaces only its generated walls.
    for model in list(world.findall("model")):
        if model.get("name", "").startswith("assignment_wall_"):
            world.remove(model)

    for index, segment in enumerate(WALLS, start=1):
        add_wall(world, index, segment)

    ET.indent(tree, space="  ")
    tree.write(WORLD_PATH, encoding="utf-8", xml_declaration=True)
    print(f"Added {len(WALLS)} wall models to {WORLD_PATH}")


if __name__ == "__main__":
    main()
