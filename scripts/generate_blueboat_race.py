#!/usr/bin/env python3
"""Generate the non-colliding Stonefish race circuit from one JSON track file.

Run on the organiser's computer after changing the course configuration.
Dependencies for *generation only*: numpy, Pillow, matplotlib and opencv-contrib
with aruco. All PNGs/XML are checked into the workshop distribution, so
participants do not need those packages to launch the simulation.
"""
import json
import math
import sys
from pathlib import Path
from xml.sax.saxutils import escape

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from roscon_stonefish_workshop.race_geometry import (  # noqa: E402
    load_course, length, sample_course, gates,
)

CONFIG = ROOT / 'config' / 'blueboat_race.json'
SCENE = ROOT / 'data' / 'scenarios' / 'blueboat_race.scn'
TEXTURES = ROOT / 'data' / 'textures'


def animated_sphere(name, xy, z, radius, look):
    return f'''  <animated name="{name}" type="sphere" collisions="false">
    <dimensions radius="{radius:.3f}"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <material name="MarkerMaterial"/>
    <look name="{look}"/>
    <trajectory type="manual">
      <keypoint time="0.0" xyz="{xy[0]:.3f} {xy[1]:.3f} {z:.3f}" rpy="0 0 0"/>
    </trajectory>
  </animated>'''


def animated_sign(name, xy, z, yaw, look, dimensions='0.035 1.60 1.60'):
    # Narrow local X is the normal of the sign face. Y/Z form its visible face.
    # uv_mode=1 uses the full texture on each box face (Stonefish 1.6).
    return f'''  <animated name="{name}" type="box" collisions="false">
    <dimensions xyz="{dimensions}"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <material name="MarkerMaterial"/>
    <look name="{look}" uv_mode="1"/>
    <trajectory type="manual">
      <keypoint time="0.0" xyz="{xy[0]:.3f} {xy[1]:.3f} {z:.3f}" rpy="0 0 {yaw:.6f}"/>
    </trajectory>
  </animated>'''


def make_sign_image(marker_id, turn, filename):
    """Large, genuine DICT_4X4_50 fiducial; arrows deliberately separate."""
    import cv2
    import numpy as np
    from PIL import Image

    if not hasattr(cv2, 'aruco'):
        raise RuntimeError('OpenCV ArUco support needed for course generation')
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    # WORKSHOP GPU DISPLAY FIX: Stonefish box UVs appear to expose only the
    # central ~512 x 512 pixels of the original 1024 texture on the broad face.
    # The old 768 px code was cropped. Put the whole 384px code inside the
    # middle 512px instead (64px white quiet zone on each edge of that window).
    # This preserves the original 1.60m boxes and both yellow direction headers.
    size = 384   # 6x6 ArUco cells, 64px each; no antialiasing
    if hasattr(cv2.aruco, 'generateImageMarker'):
        tag = cv2.aruco.generateImageMarker(dictionary, marker_id, size)
    else:
        tag = cv2.aruco.drawMarker(dictionary, marker_id, size)
    image = Image.new('L', (1024,1024), 255)
    image.paste(Image.fromarray(tag), ((1024-size)//2,(1024-size)//2))
    # Offline verify the centre texture crop too: this is the visible region
    # suggested by the screenshots of the procedural Stonefish box signs.
    rgb=image.convert('RGB')
    rgb.save(filename)
    gray=np.array(image)
    if hasattr(cv2.aruco,'ArucoDetector'):
        _corners,ids,_reject=cv2.aruco.ArucoDetector(dictionary).detectMarkers(gray)
    else:
        _corners,ids,_reject=cv2.aruco.detectMarkers(gray,dictionary)
    if ids is None or ids.flatten().tolist()!=[marker_id]:
        raise RuntimeError(f'Invalid ArUco ID {marker_id}: {ids}')
    window=gray[256:768,256:768]
    if hasattr(cv2.aruco, 'ArucoDetector'):
        _wc, wid, _wr=cv2.aruco.ArucoDetector(dictionary).detectMarkers(window)
    else:
        _wc, wid, _wr=cv2.aruco.detectMarkers(window,dictionary)
    if wid is None or wid.flatten().tolist()!=[marker_id]:
        raise RuntimeError(f'Invalid ArUco ID {marker_id} in expected visible UV crop: {wid}')


def make_arrow_image(turn, cp, filename):
    """Human-readable course instructions, separate from machine-readable tag."""
    from PIL import Image,ImageDraw,ImageFont
    panel=Image.new('RGB',(1024,384),(245,220,20))
    dr=ImageDraw.Draw(panel)
    dr.rectangle((6,6,1018,378),outline='black',width=20)
    try:
        big=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',215)
        small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',80)
    except OSError:
        big=small=ImageFont.load_default()
    arr={'LEFT':'←','RIGHT':'→','STRAIGHT':'↑'}[turn]
    dr.text((215,178),arr,font=big,anchor='mm',fill='black')
    dr.text((720,180),f'{cp}  {turn}',font=small,anchor='mm',fill='black',stroke_width=0)
    panel.save(filename)


def turn_after(course, s):
    # sign points to the next turn, not the instantaneous heading at the sign
    points = course['centerline_xy_m']
    before = sample_course(course, max(0, s-1.0))[1]
    after = sample_course(course, min(length(course), s+10))[1]
    signed = math.atan2(before[0]*after[1]-before[1]*after[0],
                        before[0]*after[0]+before[1]*after[1])
    if signed > 0.15:
        return 'RIGHT'
    if signed < -0.15:
        return 'LEFT'
    return 'STRAIGHT'


def make_preview(course, filename):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    import numpy as np

    d = length(course)
    pts = np.array([sample_course(course, s)[0] for s in np.linspace(0, d, 500)])
    width = course['half_width_m']
    fig, ax = plt.subplots(figsize=(14, 6.2))
    ax.set_facecolor('#f0f8fd')
    for side, color in ((-1, '#d1ad00'), (1, '#d1ad00')):
        p = np.array([tuple(c[k]+side*width*n[k] for k in (0, 1))
                      for s in np.linspace(0, d, 300)
                      for c, _t, n in [sample_course(course, s)]])
        ax.plot(p[:, 0], p[:, 1], '-', linewidth=1.2, color=color)
        for s in np.arange(0, d+1e-6, course['buoy_spacing_m']):
            c, _t, n = sample_course(course, s)
            ax.add_patch(Circle((c[0]+side*width*n[0],c[1]+side*width*n[1]),
                                0.27, color='#eaba11'))
    ax.plot(pts[:, 0], pts[:, 1], '--', linewidth=1, color='#5f7f97')
    # Two roadside ArUco signs at each gate (outside the yellow buoy lines).
    for name, distance, centre, tangent, normal in gates(course):
        if name.startswith('CP'):
            for side in (-1, 1):
                marker_xy = (centre[0] + side*(width+1.35)*normal[0],
                             centre[1] + side*(width+1.35)*normal[1])
                ax.plot(*marker_xy, marker='s', markersize=5.5,
                        color='#202020', markeredgecolor='white', markeredgewidth=.5)
    for name, s, c, tangent, normal in gates(course):
        half = width - course['boat_boundary_margin_m']
        a, b = (c[0]+half*normal[0], c[1]+half*normal[1]), (c[0]-half*normal[0], c[1]-half*normal[1])
        color = '#008753' if name == 'START' else '#c31e43' if name == 'FINISH' else '#3862a8'
        ax.plot((a[0],b[0]),(a[1],b[1]),color=color,linewidth=1.9)
        ax.annotate(name, c, xytext=(0, 10), textcoords='offset points', fontsize=8,
                    ha='center', color=color, fontweight='bold')
    ax.plot([course['boat_start_xyz_m'][0]], [course['boat_start_xyz_m'][1]], 'bo', ms=7)
    ax.annotate('BLUEBOAT START',course['boat_start_xyz_m'][:2],xytext=(-10,-20),textcoords='offset points',ha='center',fontsize=9)
    ax.axis('equal')
    ax.set_xlabel('Local NED X / North (m)')
    ax.set_ylabel('Local NED Y / East (m)')
    ax.set_title('ROSCon UK BlueBoat Grand Prix – Course & Ordered Timing Gates')
    ax.grid(alpha=.2)
    fig.tight_layout()
    fig.savefig(filename,dpi=160)
    plt.close(fig)


def update_robot_start(course):
    # Keep the robot spawn synchronized with the same configuration as the
    # visual gates and official race judge; fail loudly if robot source changed.
    import re
    robot = ROOT / 'data' / 'robots' / 'blueboat.scn'
    content = robot.read_text(encoding='utf-8')
    xyz = ' '.join(str(v) for v in course['boat_start_xyz_m'])
    rpy = ' '.join(str(v) for v in course['boat_start_rpy_rad'])
    pattern = r'(<world_transform rpy=")[^"]+(" xyz=")[^"]+("\s*/>)'
    updated, hits = re.subn(pattern,lambda m: m.group(1)+rpy+m.group(2)+xyz+m.group(3),content)
    if hits != 1:
        raise ValueError('Expected exactly one robot world_transform in blueboat.scn')
    robot.write_text(updated,encoding='utf-8')


def main():
    course = load_course(CONFIG)
    update_robot_start(course)
    TEXTURES.mkdir(parents=True, exist_ok=True)
    d = length(course)
    xml = ['<?xml version="1.0"?>', '<scenario>',
           '  <!-- GENERATED from config/blueboat_race.json. Do not hand edit. -->',
           '  <environment>',
           '    <ned latitude="41.7777" longitude="3.0333"/>',
           '    <ocean/>',
           '    <atmosphere><sun azimuth="20.0" elevation="50.0"/></atmosphere>',
           '  </environment>',
           '  <materials>',
           '    <material name="Neutral" density="1000.0" restitution="0.5"/>',
           '    <material name="Rock" density="3000.0" restitution="0.8"/>',
           '    <material name="Fiberglass" density="1500.0" restitution="0.3"/>',
           '    <material name="Aluminium" density="2710.0" restitution="0.7"/>',
           '    <material name="Polyamid" density="1500.0" restitution="0.1"/>',
           '    <material name="MarkerMaterial" density="1000.0" restitution="0.1"/>',
           '    <friction_table>',
           '      <friction material1="Neutral" material2="Neutral" static="0.5" dynamic="0.2"/>',
           '      <friction material1="Polyamid" material2="Polyamid" static="0.1" dynamic="0.05"/>',
           '    </friction_table>',
           '  </materials>',
           '  <looks>',
           '    <look name="RaceYellow" rgb="1.0 0.87 0.02" roughness="0.30"/>',
           '    <look name="RaceGreen" rgb="0.02 0.9 0.12" roughness="0.35"/>',
           '    <look name="RaceRed" rgb="0.95 0.02 0.08" roughness="0.35"/>',
           '    <look name="White" rgb="0.96 0.96 0.96" roughness="0.5"/>']
    for i, (name, *_rest) in enumerate(gates(course)):
        if name.startswith('CP'):
            filename = TEXTURES / f'sign_{name.lower()}.png'
            turn = turn_after(course, _rest[0])
            # Right marker uses IDs 11-22 and its already-validated UV-fit texture.
            make_sign_image(10+i, turn, filename)
            # Left marker unique ID 31-42, right marker ID 11-22.
            # Both are valid DICT_4X4_50 values (0..49).
            left_filename = TEXTURES / f'sign_{name.lower()}_left.png'
            make_sign_image(30+i, turn, left_filename)
            arrow_file = TEXTURES / f'arrow_{name.lower()}.png'
            make_arrow_image(turn, name, arrow_file)
            xml.append(f'    <look name="Sign_{name}" gray="1.0" roughness="0.7" texture="$(find roscon_stonefish_workshop)/data/textures/{filename.name}"/>')
            xml.append(f'    <look name="Sign_{name}_Left" gray="1.0" roughness="0.7" texture="$(find roscon_stonefish_workshop)/data/textures/{left_filename.name}"/>')
            xml.append(f'    <look name="Arrow_{name}" gray="1.0" roughness="0.7" texture="$(find roscon_stonefish_workshop)/data/textures/{arrow_file.name}"/>')
    xml.extend(['  </looks>', '  <include file="$(find roscon_stonefish_workshop)/data/robots/blueboat.scn"/>',
                '  <!-- Buoys at fixed positions. Animated/manual = fixed; collisions=false = intangible. -->'])
    for i in range(math.ceil(d/course['buoy_spacing_m'])+1):
        s = min(d, i*course['buoy_spacing_m'])
        c, _t, n = sample_course(course,s)
        for side, tag in ((1,'L'),(-1,'R')):
            xy = (c[0]+side*course['half_width_m']*n[0],
                  c[1]+side*course['half_width_m']*n[1])
            xml.append(animated_sphere(f'Buoy_{tag}_{i:03d}', xy,
                                       course['buoy_center_z_m'],
                                       course['buoy_radius_m'], 'RaceYellow'))
    for name, s, c, tangent, normal in gates(course):
        if name in ('START', 'FINISH'):
            for side, label in ((1, 'L'), (-1, 'R')):
                xy = (c[0]+side*course['half_width_m']*normal[0],
                      c[1]+side*course['half_width_m']*normal[1])
                xml.append(animated_sphere(f'{name}_Ball_{label}',xy,
                                           course['buoy_center_z_m'],0.47,
                                           'RaceGreen' if name == 'START' else 'RaceRed'))
        if name.startswith('CP'):
            # Identical physical panels on BOTH sides of the lane.  All are
            # outside the yellow buoy boundary, have collisions disabled and
            # face approaching boats. Right IDs 11..22, left IDs 31..42.
            # Use the previously WORKING Stonefish box UV mode and 384px tags:
            # no custom OBJ planes (those previously rendered as artifacts).
            yaw = math.atan2(tangent[1], tangent[0])+math.pi
            for side, tag in ((-1,'R'), (1,'L')):
                off = side*(course['half_width_m']+1.35)
                sx = c[0]+off*normal[0]
                sy = c[1]+off*normal[1]
                sign_look = f'Sign_{name}_Left' if tag == 'L' else f'Sign_{name}'
                xml.append(animated_sign(f'Aruco_Direction_{name}_{tag}',(sx,sy),
                                         -1.25,yaw,sign_look))
                xml.append(animated_sign(f'Arrow_Direction_{name}_{tag}',(sx,sy),
                                         -2.32,yaw,f'Arrow_{name}',
                                         dimensions='0.035 1.60 0.48'))
    xml.append('</scenario>')
    SCENE.write_text('\n'.join(xml)+'\n', encoding='utf-8')
    make_preview(course, ROOT/'data'/'blueboat_race_map.png')
    print(f'Wrote {SCENE}, {2*len(course["checkpoint_s_m"])} verified ArUco DICT_4X4_50 markers on paired gates. Track length {d:.1f}m.')


if __name__ == '__main__':
    main()
