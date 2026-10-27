#!/usr/bin/env python3
"""Offline route and detection checks; optionally pass a rendered arena image."""
import ast, heapq, math, json, random, time, sys
from pathlib import Path
from itertools import permutations
import cv2 as cv
import numpy as np

source=Path(__file__).resolve().parents[1] / 'src/swift_pico/src/waypoint_service.py'
tree=ast.parse(source.read_text())
cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='WayPoints')
methods=['shortest_grid_path','plan_route','get_navigation_route','get_survivor_coordinates']
cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in methods]
funcs=[n for n in tree.body if isinstance(n,ast.FunctionDef)]
ns=dict(Node=object, math=math, heapq=heapq, permutations=permutations, cv=cv, np=np)
exec(compile(ast.fix_missing_locations(ast.Module(body=funcs+[cls],type_ignores=[])),str(source),'exec'),ns)
node=ns['WayPoints']()
node.grid_coordinates=json.loads(source.with_name('grid_whycode_coordinates.json').read_text())
node.turn_penalty=.15
node.max_route_extra=.02
labels=sorted(node.grid_coordinates)
indices={label:i for i,label in enumerate(labels)}
# Independent all-pairs shortest paths over measured, neighboring grid edges.
dist=np.full((len(labels),len(labels)),np.inf)
np.fill_diagonal(dist,0.)
for a in labels:
    for b in labels:
        dc=abs(ord(a[0])-ord(b[0])); dr=abs(int(a[1:])-int(b[1:]))
        if max(dc,dr)==1:
            p=node.grid_coordinates[a];q=node.grid_coordinates[b]
            dist[indices[a],indices[b]]=math.hypot(p['x']-q['x'],p['y']-q['y'])
for k in range(len(labels)):
    dist=np.minimum(dist,dist[:,k,None]+dist[None,k,:])
def expand(route):
    result=route[:1]
    for a,b in zip(route,route[1:]):
        dc=ord(b[0])-ord(a[0]);dr=int(b[1:])-int(a[1:]);count=max(abs(dc),abs(dr))
        assert count and (dc==0 or dr==0 or abs(dc)==abs(dr))
        for i in range(1,count+1):
            result.append(f'{chr(ord(a[0])+i*(dc//count))}{int(a[1:])+i*(dr//count)}')
    return result
rng=random.Random(347)
layouts=[['D7','E4','H5','H7'],['F6','F7','F8','F9'],['A1','A11','K1','K11']]
layouts.extend(rng.sample(labels,4) for _ in range(100))
started=time.monotonic()
for survivors in layouts:
    order,route,distance=node.plan_route(survivors)
    baseline=min(sum(dist[indices[a],indices[b]] for a,b in zip(('F6',)+order,order)) for order in permutations(survivors))
    assert distance <= baseline*1.02+1e-8, (survivors,distance,baseline)
    assert route[0]=='F6' and set(survivors)<=set(route)
    seen=set()
    for label in route[:-1]:
        seen.add(label)
        assert not set(survivors)<=seen
    for a,b in zip(route,route[1:]):
        assert max(abs(ord(a[0])-ord(b[0])),abs(int(a[1:])-int(b[1:])))==1
    nav=node.get_navigation_route(route,survivors)
    assert expand(nav)==route
    assert [x for x in route if x in survivors]==[x for x in nav if x in survivors]
    assert order==list(dict.fromkeys(x for x in route if x in survivors))
    visited=set();hover=[]
    for x in (nav if 'F6' in survivors else nav[1:]):
        if x in survivors and x not in visited:
            hover.append(x);visited.add(x)
    assert len(hover)==4
print('PASS: 103 layouts, independent Floyd-Warshall/24-order distance bound, grid edges, compression, exactly four first-visit holds',flush=True)
order,route,distance=node.plan_route(layouts[0])
print('Current arena:',order,route,node.get_navigation_route(route,layouts[0]),'distance',distance,flush=True)
assert len(node.get_navigation_route(route,layouts[0]))-1==7
for bad in [[],['D7']*4,['A1','B1','C1','Z99']]:
    try: node.plan_route(bad)
    except ValueError: pass
    else: raise AssertionError('Invalid layout accepted')
print('PASS: invalid/duplicate/unknown survivor labels rejected',flush=True)
# Detection across different synthetic survivor locations, including boundaries.
grid=ns['get_grid_points']()
for survivors in layouts[:23]:
    image=np.full((900,900,3),180,np.uint8)
    for label in survivors:
        x,y=grid[label]
        cv.fillPoly(image,[np.array([[x,y-15],[x-15,y+12],[x+15,y+12]])],(0,0,255))
    detected=ns['detect_objects'](image,grid)
    assert sorted(x['grid_location'] for x in detected if x['shape']=='Red Triangle')==sorted(survivors)
print('PASS: detection at 23 synthetic survivor layouts',flush=True)
if len(sys.argv) > 1:
    frame=cv.imread(sys.argv[1])
    assert frame is not None
    for size in [640,960,1920]:
        for brightness in [.65,1.,1.25]:
            scaled=cv.resize(frame,(size,size))
            scaled=np.clip(scaled.astype(np.float32)*brightness,0,255).astype(np.uint8)
            corners,ids=ns['get_markers'](cv.cvtColor(scaled,cv.COLOR_BGR2GRAY))
            assert ids is not None
            src_pts=ns['get_inner_corners'](corners,ids)
            matrix=cv.getPerspectiveTransform(src_pts,np.float32([[0,0],[899,0],[899,899],[0,899]]))
            warped=cv.warpPerspective(scaled,matrix,(900,900))
            detected=ns['detect_objects'](warped,grid)
            critical=sorted(x['grid_location'] for x in detected if x['shape']=='Red Triangle')
            assert critical==sorted(layouts[0]), (size,brightness,critical)
    print('PASS: rendered arena detection at 3 resolutions x 3 brightness factors',flush=True)
print('Elapsed',round(time.monotonic()-started,2),'seconds',flush=True)
