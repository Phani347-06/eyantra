#!/usr/bin/env python3
"""Check alternate marker layouts using still renders, without launching a flight."""

import hashlib
import json
import os
from pathlib import Path
import runpy

os.environ.setdefault('MUJOCO_GL', 'egl')
import mujoco
import cv2 as cv
import numpy as np


def main():
    root = Path(__file__).resolve().parents[1]
    checks = runpy.run_path(str(root / 'checks/check_task2c_planner.py'))
    node, functions = checks['node'], checks['ns']
    arena = root / 'src/swift_pico_description/models/swift_pico/arena.xml'
    original_hash = hashlib.sha256(arena.read_bytes()).hexdigest()
    model = mujoco.MjModel.from_xml_path(str(arena))
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=960, width=960)
    output = root / 'verification/rendered_layouts'
    output.mkdir(parents=True, exist_ok=True)
    layouts = [
        ['D7', 'E4', 'H5', 'H7'],
        ['D4', 'H4', 'D8', 'H8'],
        ['E5', 'G5', 'E7', 'G7'],
        ['B2', 'J2', 'B10', 'J10'],
        ['F4', 'F5', 'F7', 'F8'],
    ]
    reports = []
    try:
        for index, survivors in enumerate(layouts):
            # Change only the in-memory floor markers; leave arena files intact.
            for number, label in enumerate(survivors, 1):
                geom = model.geom(f'obstacle_triangle_{number}')
                col, row = ord(label[0]) - ord('A'), int(label[1:])
                geom.pos[:] = [(col - 5) * 1.203414, (6 - row) * 1.203414, 0.005]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera='top_cam')
            image = cv.cvtColor(renderer.render(), cv.COLOR_RGB2BGR)
            corners, ids = functions['get_markers'](cv.cvtColor(image, cv.COLOR_BGR2GRAY))
            assert ids is not None, survivors
            points = functions['get_inner_corners'](corners, ids)
            matrix = cv.getPerspectiveTransform(points, np.float32([[0, 0], [899, 0], [899, 899], [0, 899]]))
            rectified = cv.warpPerspective(image, matrix, (900, 900))
            objects = functions['detect_objects'](rectified, functions['get_grid_points']())
            found = sorted(item['grid_location'] for item in objects if item['shape'] == 'Red Triangle')
            assert found == sorted(survivors), (survivors, found)
            order, route, distance = node.plan_route(found)
            navigation = node.get_navigation_route(route, found)
            goals = navigation if 'F6' in found else navigation[1:]
            filename = output / f'layout_{index}.png'
            assert cv.imwrite(str(filename), image)
            reports.append({
                'survivors': found, 'visit_order': order, 'navigation_goals': goals,
                'distance': distance, 'image': str(filename.relative_to(root)),
                'flight_tested': False,
            })
            print(f'PASS: rendered {found}; {len(goals)} navigation goals', flush=True)
    finally:
        renderer.close()
    assert hashlib.sha256(arena.read_bytes()).hexdigest() == original_hash
    (output / 'results.json').write_text(json.dumps(reports, indent=2) + '\n')
    print('PASS: five still-render layouts; arena file unchanged. No flights run.')


if __name__ == '__main__':
    main()
