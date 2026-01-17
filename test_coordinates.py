#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import os

map_file = os.path.expanduser(
    "~/Desktop/opentcs-6.3.0-bin/opentcs-modeleditor/data/Demo-01.xml"
)

tree = ET.parse(map_file)
root = tree.getroot()

print("=" * 70)
print("OpenTCS COORDINATE ANALYSIS")
print("=" * 70)

points = {}
for point in root.findall(".//point"):
    name = point.get("name")
    x_raw = float(point.get("positionX", "0"))
    y_raw = float(point.get("positionY", "0"))
    points[name] = (x_raw, y_raw)

# Get min/max to understand scale
x_values = [p[0] for p in points.values()]
y_values = [p[1] for p in points.values()]

print(f"\nRAW OpenTCS values (likely in millimeters):")
print(f"  X range: {min(x_values):.0f} to {max(x_values):.0f}")
print(f"  Y range: {min(y_values):.0f} to {max(y_values):.0f}")
print(f"  Map width: {max(x_values) - min(x_values):.0f} mm")
print(f"  Map height: {max(y_values) - min(y_values):.0f} mm")

# Show a few points with different scalings
print("\nSample points with different scalings:")
print("-" * 70)
print(f"{'Point':<15} {'Raw (mm)':<20} {'÷1000 (m)':<20} {'÷10000 (m)':<20}")
print("-" * 70)

for name in list(points.keys())[:10]:
    x, y = points[name]
    print(
        f"{name:<15} ({x:>7.0f}, {y:>7.0f})   ({x*0.001:>6.3f}, {y*0.001:>6.3f})   ({x*0.0001:>6.3f}, {y*0.0001:>6.3f})"
    )

print("\n" + "=" * 70)
print("RECOMMENDATION:")
map_width_m = (max(x_values) - min(x_values)) * 0.001
map_height_m = (max(y_values) - min(y_values)) * 0.001

if map_width_m > 100 or map_height_m > 100:
    print(f"⚠ Map is HUGE ({map_width_m:.1f}m x {map_height_m:.1f}m)!")
    print(f"  Try using scale = 0.0001 instead of 0.001")
    print(f"  OR your OpenTCS coordinates might be in a different unit")
elif map_width_m > 20 or map_height_m > 20:
    print(f"⚠ Map is large ({map_width_m:.1f}m x {map_height_m:.1f}m)")
    print(f"  This might work but Gazebo world is typically ~10m x 10m")
else:
    print(f"✓ Map size looks good: {map_width_m:.1f}m x {map_height_m:.1f}m")

print("=" * 70)
