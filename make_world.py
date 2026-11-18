"""Generate a self-contained Gazebo world for the parking demo."""
import argparse
import math
from pathlib import Path
from xml.etree import ElementTree as ET


ULTRASONIC_MOUNTS = {
    "front": (.33, 0.0, 0.0),
    "rear": (-.33, 0.0, math.pi),
    "left": (0.0, .31, math.pi / 2),
    "right": (0.0, -.31, -math.pi / 2),
}
VISION_MARKER_WIDTH = .9
CAMERA_FOV = .9
CAMERA_X = .32
TRAFFIC_COLORS = (
    "0.85 0.49 0.10 1", "0.12 0.26 0.63 1", "0.74 0.76 0.79 1",
    "0.52 0.10 0.12 1", "0.29 0.42 0.29 1", "0.20 0.20 0.22 1",
)
AISLE_Y = 10
ENTRY_X = -28
ACCESS_CURB_X = ENTRY_X - .7
IDLE_POINT = (ENTRY_X + 2.0, -2.3)
IDLE_YAW = math.pi
WHEEL_RADIUS = .12
# X increases toward the diagonal end; Y increases toward the right of the aerial.
# Measured from the driveway mouth to the far curb on each aisle centerline.
RIGHT_CURB_X = ENTRY_X + 150 * .3048 + 130 * .0254
LEFT_CURB_X = ENTRY_X + 197 * .3048
curb_slope = (LEFT_CURB_X - RIGHT_CURB_X) / (2 * AISLE_Y)
# Widen the diagonal connector to match the 7.6 m driving aisles.
CONNECTOR_HALF_WIDTH = 3.8
CONNECTOR_CURB_X = 33.0
CONNECTOR_SCALE = math.hypot(1.0, curb_slope)


def connector_x(y, curb_inset=0.0):
    """X on a track parallel to the far curb, inset by the given meters."""
    return CONNECTOR_CURB_X + curb_slope * y - curb_inset * CONNECTOR_SCALE


# A continuous diagonal back wall, then one clipped corner into the outer row.
FOOTPRINT = ((ENTRY_X, -18.5), (connector_x(-18.5), -18.5),
             (connector_x(12), 12),
             (36, 18.5), (ENTRY_X, 18.5))
STALL_WIDTH = 106 * .0254
STALL_DEPTH = 16 * .3048
LOADING_WIDTH, LOADING_LENGTH = 80 * .0254, 285 * .0254
MOTO_WIDTH, MOTO_LENGTH = 39.5 * .0254, 103 * .0254

# Aisle, row, first and last car centers, yaw, car count, loading areas, motorcycles.
ROWS = (
    (10, 16, -23, 26, 1.99, 13, 0, 0),       # Green, left outer
    (-10, -16, -25, 13, -1.15, 19, 0, 2),    # Red, right outer
    (-10, -3.8, -23, 14, 1.15, 10, 4, 0),    # Cyan, right middle
    (10, 3.8, -23, 16, -1.99, 12, 4, 2),    # Purple, left middle
)
# Ordered from the entrance down each row. Empty stalls remain between cars.
PLATES = (
    ("7ABC853", "8CFA427", "7EDF619", "9FAB372", "6BAE935"),
    ("7CDE482", "6DEA715", "8EAF329", "7FBC946", "8DFE504"),
    ("9DAC518", "8ABC271", "6EBD237", "8FAD654"),
    ("7BCE183", "9ACE562", "9BDF603", "6DBF801"),
)


def stalls(first, last, yaw, count, loading):
    # Place access aisles between pairs near the driveway, as in the aerial.
    # Distribute any extra row length across the gaps.
    access_after = {2 * i for i in range(loading)}
    loading_pitch = LOADING_WIDTH / abs(math.sin(yaw))
    pitch = (last - first - loading * loading_pitch) / (count - 1)
    centers = [first]
    for i in range(count - 1):
        centers.append(centers[-1] + pitch + (loading_pitch if i in access_after else 0))
    return centers, access_after


PARKED_CARS = []
for (_, y, first, last, yaw, count, loading, _), plates in zip(ROWS, PLATES):
    centers, _ = stalls(first, last, yaw, count, loading)
    for i, number in enumerate(plates):
        slot = round(i * (len(centers) - 1) / (len(plates) - 1))
        PARKED_CARS.append((number, centers[slot], y, yaw))


def add(parent, tag, value=None, **attributes):
    element = ET.SubElement(parent, tag, attributes)
    if value is not None:
        element.text = str(value)
    return element


def plugin(parent, library, system, settings=()):
    node = add(parent, "plugin", filename=f"gz-sim-{library}-system",
               name=f"gz::sim::systems::{system}")
    for key, value in settings:
        add(node, key, value)


def shape(link, name, size, color, pose="0 0 0 0 0 0", collide=True):
    visual = add(link, "visual", name=name)
    for element in [visual] + ([add(link, "collision", name=f"{name}_collision")] if collide else []):
        add(element, "pose", pose)
        geometry = add(element, "geometry")
        if isinstance(size, str):
            add(add(geometry, "box"), "size", size)
        else:
            cylinder = add(geometry, "cylinder")
            add(cylinder, "radius", size[0])
            add(cylinder, "length", size[1])
    material = add(visual, "material")
    add(material, "ambient", color)
    add(material, "diffuse", color)
    return visual


def static_model(world, name, pose):
    model = add(world, "model", name=name)
    add(model, "static", "true")
    add(model, "pose", pose)
    return add(model, "link", name="body")


def inertia(link, mass, diagonal):
    node = add(link, "inertial")
    add(node, "mass", mass)
    tensor = add(node, "inertia")
    for axis, value in zip(("ixx", "iyy", "izz"), diagonal):
        add(tensor, axis, value)


def rover(world):
    model = add(world, "model", name="rover")
    x, y = IDLE_POINT
    add(model, "pose", f"{x} {y} 0.23 0 0 {IDLE_YAW}")
    base = add(model, "link", name="base_link")
    inertia(base, 8, (0.12, 0.22, 0.32))
    shape(base, "chassis", "0.62 0.40 0.18", "0.1 0.45 0.16 1")
    joints = []
    for side, y in (("left", 0.26), ("right", -0.26)):
        for axle, x in (("front", 0.22), ("rear", -0.22)):
            name = f"{axle}_{side}"
            wheel = add(model, "link", name=name)
            add(wheel, "pose", f"{x} {y} -0.10 0 0 0")
            inertia(wheel, 0.4, (0.002, 0.003, 0.002))
            shape(wheel, "tire", (WHEEL_RADIUS, 0.075), "0.05 0.05 0.05 1",
                  "0 0 0 1.570796 0 0")
            joint = add(model, "joint", name=f"{name}_joint", type="revolute")
            add(joint, "parent", "base_link")
            add(joint, "child", name)
            add(add(joint, "axis"), "xyz", "0 1 0")
            joints.append((f"{side}_joint", f"{name}_joint"))
    plugin(model, "diff-drive", "DiffDrive", joints + [
        ("wheel_separation", 0.52), ("wheel_radius", WHEEL_RADIUS),
        ("topic", "/cmd_vel"), ("odom_topic", "/odom"),
        ("frame_id", "odom"), ("child_frame_id", "base_link"),
    ])
    plugin(model, "joint-state-publisher", "JointStatePublisher", [
        ("topic", "/wheel_encoders"),
        *(("joint_name", name) for _, name in joints),
    ])
    imu = add(base, "sensor", name="rover_imu", type="imu")
    add(imu, "topic", "/imu/data")
    add(imu, "always_on", "true")
    add(imu, "update_rate", 50)
    lidar = add(base, "sensor", name="traffic_lidar", type="gpu_lidar")
    add(lidar, "pose", "0 0 0.32 0 0 0")
    add(lidar, "topic", "/scan")
    add(lidar, "always_on", "true")
    add(lidar, "update_rate", 15)
    settings = add(lidar, "lidar")
    horizontal = add(add(settings, "scan"), "horizontal")
    add(horizontal, "samples", 720)
    add(horizontal, "min_angle", -math.pi)
    add(horizontal, "max_angle", math.pi)
    limits = add(settings, "range")
    add(limits, "min", .35)
    add(limits, "max", 25)
    add(limits, "resolution", .01)
    # Short, narrow ray fans approximate ultrasonic cones in Gazebo.
    for side, (x, y, yaw) in ULTRASONIC_MOUNTS.items():
        sensor = add(base, "sensor", name=f"ultrasonic_{side}", type="gpu_lidar")
        add(sensor, "pose", f"{x} {y} 0.20 0 0 {yaw}")
        add(sensor, "topic", f"/ultrasonic/{side}")
        add(sensor, "always_on", "true")
        add(sensor, "update_rate", 10)
        settings = add(sensor, "lidar")
        horizontal = add(add(settings, "scan"), "horizontal")
        add(horizontal, "samples", 9)
        add(horizontal, "min_angle", -.2)
        add(horizontal, "max_angle", .2)
        limits = add(settings, "range")
        add(limits, "min", .03)
        add(limits, "max", 1.5)
        add(limits, "resolution", .01)
    sensor = add(base, "sensor", name="plate_camera", type="camera")
    add(sensor, "pose", f"{CAMERA_X} 0 0.24 0 0 0")
    add(sensor, "topic", "/plate_camera/image")
    add(sensor, "always_on", "true")
    add(sensor, "update_rate", 15)
    camera = add(sensor, "camera")
    add(camera, "horizontal_fov", CAMERA_FOV)
    image = add(camera, "image")
    add(image, "width", 640)
    add(image, "height", 360)
    clip = add(camera, "clip")
    add(clip, "near", 0.05)
    add(clip, "far", 50)


def plate(link, number):
    shape(link, "plate", "0.025 0.32 0.16", "0.96 0.96 0.96 1",
          "-2.27 0 0.47 0 0 0", False)
    # Each hexadecimal value encodes one row of a five-pixel-wide glyph.
    glyphs = {
        "0": "0e 11 13 15 19 11 0e", "1": "04 0c 04 04 04 04 0e",
        "2": "0e 11 01 02 04 08 1f", "3": "1e 01 01 0e 01 01 1e",
        "4": "02 06 0a 12 1f 02 02", "5": "1f 10 10 1e 01 01 1e",
        "6": "0e 10 10 1e 11 11 0e", "7": "1f 01 02 04 08 08 08",
        "8": "0e 11 11 0e 11 11 0e", "9": "0e 11 11 0f 01 01 0e",
        "A": "0e 11 11 1f 11 11 11", "B": "1e 11 11 1e 11 11 1e",
        "C": "0f 10 10 10 10 10 0f", "D": "1e 11 11 11 11 11 1e",
        "E": "1f 10 10 1e 10 10 1f", "F": "1f 10 10 1e 10 10 10",
    }
    for index, character in enumerate(number):
        for row, mask in enumerate(glyphs[character].split()):
            for column in range(5):
                if int(mask, 16) & (16 >> column):
                    y = (3 - index) * .042 + (2 - column) * .007
                    z = .47 + (3 - row) * .014
                    shape(link, f"glyph_{index}_{row}_{column}", "0.005 0.007 0.014",
                          "0.02 0.04 0.16 1", f"-2.286 {y} {z} 0 0 0", False)


def car_body(car, color):
    shape(car, "body", "4.5 1.85 0.70", color, "0 0 0.51 0 0 0")
    shape(car, "cabin", "2.2 1.55 0.50", "0.09 0.13 0.17 1",
          "0.20 0 1.08 0 0 0")


def line(link, name, start, end, width, height, color, collide=False):
    x, y = start
    dx, dy = end[0] - x, end[1] - y
    shape(link, name, f"{math.hypot(dx, dy)} {width} {height}", color,
          f"{x + dx/2} {y + dy/2} {height/2} 0 0 {math.atan2(dy, dx)}", collide)


def pavement(world):
    ground = static_model(world, "surrounding_ground", "0 0 -0.1 0 0 0")
    shape(ground, "ground", "100 85 0.2", "0.33 0.37 0.23")
    asphalt = static_model(world, "lot10_pavement", "0 0 0.002 0 0 0")
    visual = add(asphalt, "visual", name="traced_footprint")
    polyline = add(add(visual, "geometry"), "polyline")
    add(polyline, "height", .004)
    for x, y in FOOTPRINT:
        add(polyline, "point", f"{x} {y}")
    material = add(visual, "material")
    add(material, "ambient", "0.20 0.21 0.23 1")
    add(material, "diffuse", "0.20 0.21 0.23 1")
    shape(asphalt, "botanic_gardens_drive", "8 62 0.008", "0.24 0.25 0.27 1",
          "-32 0 0 0 0 0", False)
    curbs = static_model(world, "perimeter_curbs", "0 0 0 0 0 0")
    concrete = "0.68 0.65 0.58 1"
    for i, (start, end) in enumerate(zip(FOOTPRINT, FOOTPRINT[1:])):
        line(curbs, f"edge_{i}", start, end, .20, .14, concrete, True)
    island = static_model(world, "center_median", "0 0 0 0 0 0")
    shape(island, "walkway", "34 1.6 .12", concrete, "-10 0 .06 0 0 0")
    shape(island, "planting_bed", "16 3.2 .18", "0.43 0.34 0.23 1",
          "14 0 .09 0 0 0")
    shape(island, "rounded_tip", (1.6, .18), "0.43 0.34 0.23 1",
          "22 0 .09 0 0 0")
    markings = static_model(world, "markings", "0 0 .009 0 0 0")
    blue, white = "0.22 0.55 0.92 1", "0.94 0.94 0.88 1"
    for row, (_, y, first, last, yaw, count, loading, motorcycles) in enumerate(ROWS):
        centers, access_after = stalls(first, last, yaw, count, loading)
        dx = math.cos(yaw)
        dy = math.sin(yaw)
        pitch = (last - first - loading * LOADING_WIDTH / abs(dy)) / (count - 1)
        # The red row's count and nominal width cannot both fit its measured run.
        width = min(STALL_WIDTH / abs(dy), pitch)
        stripe_positions = set()
        for i, x in enumerate(centers):
            for edge in (x - width / 2, x + width / 2):
                stripe_positions.add(round(edge, 5))
            if i in access_after:
                center = (x + centers[i + 1]) / 2
                half = LOADING_WIDTH / 2
                for k in range(7):
                    along = (k - 3) * LOADING_LENGTH / 7
                    start = (center + dx * along + dy * half, y + dy * along - dx * half)
                    end = (center + dx * along - dy * half, y + dy * along + dx * half)
                    line(markings, f"loading_{row}_{i}_{k}", start, end, .07, .008, blue)
        for i, x in enumerate(sorted(stripe_positions)):
            line(markings, f"stall_{row}_{i}",
                 (x - dx * STALL_DEPTH/2, y - dy * STALL_DEPTH/2),
                 (x + dx * STALL_DEPTH/2, y + dy * STALL_DEPTH/2),
                 .09, .008, white)
        for i in range(motorcycles + 1 if motorcycles else 0):
            x = last + width / 2 + .2 + i * MOTO_WIDTH / abs(dy)
            line(markings, f"motorcycle_{row}_{i}",
                 (x - dx * MOTO_LENGTH/2, y - dy * MOTO_LENGTH/2),
                 (x + dx * MOTO_LENGTH/2, y + dy * MOTO_LENGTH/2), .09, .008, white)
    for i in range(7):
        shape(markings, f"crosswalk_{i}", ".33 3 .008", "0.94 0.94 0.88 1",
              f"{-34.5 + i*.77} -6 .004 0 0 0", False)


def build_world():
    sdf = ET.Element("sdf", version="1.9")
    world = add(sdf, "world", name="lot10_prototype")
    physics = add(world, "physics", name="real_time", type="ode")
    add(physics, "max_step_size", .001)
    add(physics, "real_time_factor", 1.0)
    plugin(world, "physics", "Physics")
    plugin(world, "imu", "Imu")
    plugin(world, "user-commands", "UserCommands")
    plugin(world, "scene-broadcaster", "SceneBroadcaster")
    plugin(world, "sensors", "Sensors", [("render_engine", "ogre2")])
    sun = add(world, "light", name="sun", type="directional")
    add(sun, "pose", "0 0 10 0 0 0")
    add(sun, "diffuse", "0.9 0.9 0.9 1")
    add(sun, "direction", "-0.3 0.2 -1")
    scene = add(world, "scene")
    add(scene, "ambient", "0.6 0.6 0.6 1")
    pavement(world)
    colors = ("0.1 0.2 0.65 1", "0.6 0.1 0.1 1", "0.4 0.45 0.48 1",
              "0.8 0.8 0.76 1", "0.28 0.35 0.25 1")
    for index, (number, x, y, yaw) in enumerate(PARKED_CARS):
        color = colors[index % len(colors)]
        car = static_model(world, f"car_{number}", f"{x} {y} 0 0 0 {yaw}")
        car_body(car, color)
        plate(car, number)
    for slot, color in enumerate(TRAFFIC_COLORS):
        traffic = static_model(world, f"passing_car_{slot}", f"0 0 {-5 - slot * 2} 0 0 0")
        car_body(traffic, color)
        # High-contrast target for the simulated camera detector.
        for face, x in (("front", 2.26), ("rear", -2.26)):
            marker = shape(traffic, f"vision_marker_{face}", f".015 {VISION_MARKER_WIDTH} .24",
                           "1 0.015 1 1", f"{x} 0 .54 0 0 0", False)
            add(marker.find("material"), "emissive", ".9 .01 .9 1")
    rover(world)
    return ET.ElementTree(sdf)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, nargs="?", default=Path("lot10_prototype.sdf"))
    args = parser.parse_args()
    tree = build_world()
    ET.indent(tree)
    tree.write(args.output, encoding="utf-8", xml_declaration=True)
