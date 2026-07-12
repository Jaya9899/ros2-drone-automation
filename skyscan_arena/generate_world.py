# Generate mission2_arena.sdf to scale from the Mission 2 brief.
# Coordinate frame (SDF ENU, +X = forward/east, +Y = left/north, +Z = up):
#   takeoff pad centered at origin; corridor along +X; arena beyond.
import os
TEX_DIR = os.path.expanduser("~/ros2_ws/src/skyscan_arena/materials/textures")
def box_visual_only(name, x, y, z, sx, sy, sz, rgba, static=True):
    r,g,b,a = rgba
    return f'''    <model name="{name}">
      <static>{"true" if static else "false"}</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <visual name="v">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
'''

def box_solid(name, x, y, z, sx, sy, sz, rgba):
    # physical: has collision (drone can hit walls / obstacles)
    r,g,b,a = rgba
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="c">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>
        </visual>
      </link>
    </model>
'''
def qr_plane(name, x, y, texture, size=1.0, z=0.016, thickness=0.002):
    path = os.path.join(TEX_DIR, texture)
    if not os.path.exists(path):
        raise FileNotFoundError(f"{name}: {path}")
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><box><size>{size} {size} {thickness}</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>0 0 0 1</specular>
            <pbr>
              <metal>
                <albedo_map>{path}</albedo_map>
                <metalness>0</metalness>
                <roughness>1</roughness>
              </metal>
            </pbr>
          </material>
        </visual>
      </link>
    </model>
'''

parts = []

# ---- takeoff / landing zone (blue marker) centered at origin, 4m x 6m ----
parts.append(box_visual_only("takeoff_landing_zone", 0, 0, 0.02, 4, 6, 0.02, (0.15,0.35,0.85,1)))


# ---- corridor: 10 m long (x=4..14), two 3.5 m lanes, 10 ft (3.048 m) walls ----
H = 10.0          # wall height
LANE_WIDTH = 3.5
GAP = 2.0         # space between corridors
WALL = 0.10
GROUND_TOP = 0.001
ARENA_TOP = 0.016
# --------------------------------------------------------
# Two independent corridors
# --------------------------------------------------------

forward_y = 4.75
return_y = -4.75

# coloured floors
parts.append(box_visual_only(
    "lane_forward_green",
    9,
    forward_y,
    0.011,
    10,
    LANE_WIDTH,
    0.02,
    (0.15,0.6,0.2,1)
))

parts.append(box_visual_only(
    "lane_return_orange",
    9,
    return_y,
    0.011,
    10,
    LANE_WIDTH,
    0.02,
    (0.9,0.5,0.1,1)
))

wall_rgba=(0.55,0.55,0.55,1)

# forward corridor walls
parts.append(box_solid(
    "forward_outer_wall",
    9,
    forward_y + LANE_WIDTH/2,
    H/2,
    10,
    WALL,
    H,
    wall_rgba
))

parts.append(box_solid(
    "forward_inner_wall",
    9,
    forward_y - LANE_WIDTH/2,
    H/2,
    10,
    WALL,
    H,
    wall_rgba
))

# return corridor walls
parts.append(box_solid(
    "return_inner_wall",
    9,
    return_y + LANE_WIDTH/2,
    H/2,
    10,
    WALL,
    H,
    wall_rgba
))

parts.append(box_solid(
    "return_outer_wall",
    9,
    return_y - LANE_WIDTH/2,
    H/2,
    10,
    WALL,
    H,
    wall_rgba
))
# ---- static obstacles in the RETURN (orange) lane ----
obs = (0.05,0.05,0.05,1)
parts.append(box_solid("obstacle_1", 7.0,  -4.0, 1.0, 0.3, 0.3, 2.0, obs))
parts.append(box_solid("obstacle_2", 9.5,  -5.8, 1.0, 0.3, 0.3, 2.0, obs))
parts.append(box_solid("obstacle_3", 11.5, -4.8, 1.25, 0.4, 0.4, 2.5, obs))

# ---- arena floor: 40 m (x=14..54) x 30 m (y=-15..15), green field ----
parts.append(box_visual_only("arena_floor", 34, 0, 0.008, 40, 30, 0.01, (0.20,0.55,0.25,1)))
# thin white boundary strips (low markers, not walls)
bnd = (0.95,0.95,0.95,1)
parts.append(box_visual_only("arena_bnd_w", 14, 0, 0.1, 0.15, 30, 0.2, bnd))
parts.append(box_visual_only("arena_bnd_e", 54, 0, 0.1, 0.15, 30, 0.2, bnd))
parts.append(box_visual_only("arena_bnd_n", 34, 15, 0.1, 40, 0.15, 0.2, bnd))
parts.append(box_visual_only("arena_bnd_s", 34,-15, 0.1, 40, 0.15, 0.2, bnd))

# ---- Red Zone (no-fly), right-center of arena, 8m x 6m ----
# Main
parts.append(box_visual_only(
    "red_zone_main",
    43,
    0,
    0.02,
    8,
    6,
    0.02,
    (0.85,0.1,0.1,1)
))

# Small
parts.append(box_visual_only(
    "red_zone_small",
    24,
    -10,
    0.02,
    5,
    4,
    0.02,
    (0.85,0.1,0.1,1)
))

# Large
parts.append(box_visual_only(
    "red_zone_large",
    37,
    11,
    0.02,
    10,
    8,
    0.02,
    (0.85,0.1,0.1,1)
))
# ---- QR targets scattered in arena; TARGET_A is the correct match ----
# ---- reference QR (on ground before corridor) ----
parts.append(
    qr_plane(
        "qr_reference",
        2.5,
        0,
        "qr_ref_target_A.png",
        z=GROUND_TOP
    )
)

# ---- QR targets in arena (placed above arena floor) ----
parts.append(
    qr_plane(
        "qr_target_A",
        30,
        5,
        "qr_target_A.png",
        z=ARENA_TOP
    )
)

parts.append(
    qr_plane(
        "qr_target_B",
        20,
        8,
        "qr_target_b.png",
        z=ARENA_TOP
    )
)

parts.append(
    qr_plane(
        "qr_target_C",
        22,
        -9,
        "qr_target_c.png",
        z=ARENA_TOP
    )
)

parts.append(
    qr_plane(
        "qr_target_D",
        33,
        -7,
        "qr_target_d.png",
        z=ARENA_TOP
    )
)

parts.append(
    qr_plane(
        "qr_target_E",
        48,
        8,
        "qr_target_e.png",
        z=ARENA_TOP
    )
)

parts.append(
    qr_plane(
        "qr_target_F",
        48,
        -9,
        "qr_target_f.png",
        z=ARENA_TOP
    )
)

models = "\n".join(parts)

gui = '''    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui><title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <engine>ogre2</engine><scene>scene</scene>
        <ambient_light>0.5 0.5 0.5</ambient_light>
        <background_color>0.75 0.82 0.9</background_color>
        <camera_pose>25 -48 34 0 0.62 1.5708</camera_pose>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager"/>
      <plugin filename="InteractiveViewControl" name="Interactive view control"/>
      <plugin filename="CameraTracking" name="Camera Tracking"/>
      <plugin filename="MarkerManager" name="Marker manager"/>
      <plugin filename="SelectEntities" name="Select Entities"/>
      <plugin filename="Spawn" name="Spawn Entities"/>
      <plugin filename="EntityContextMenuPlugin" name="Entity context menu"/>
      <plugin filename="WorldControl" name="World control">
        <gz-gui>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">72</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="left" target="left"/><line own="bottom" target="bottom"/></anchors>
        </gz-gui>
        <play_pause>true</play_pause><step>true</step><start_paused>true</start_paused>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <gz-gui>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">110</property><property type="double" key="width">290</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="right" target="right"/><line own="bottom" target="bottom"/></anchors>
        </gz-gui>
        <sim_time>true</sim_time><real_time>true</real_time><real_time_factor>true</real_time_factor>
      </plugin>
      <plugin filename="TransformControl" name="Transform control"/>
      <plugin filename="Shapes" name="Shapes"/>
      <plugin filename="ComponentInspector" name="Component inspector"/>
      <plugin filename="EntityTree" name="Entity tree"/>
    </gui>
'''

sdf = f'''<?xml version="1.0" ?>
<!--
  SkyScan Mission 2 arena - built to the competition brief.
  Standalone world: open now with `gz sim mission2_arena.sdf` to build/eyeball.
  Add the ArduPilot iris model later (see README) to fly in it.
-->
<sdf version="1.9">
  <world name="mission2_arena">
    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

{gui}
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

{models}
  </world>
</sdf>
'''

with open("mission2_arena.sdf","w") as f:
    f.write(sdf)
