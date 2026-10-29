#!/usr/bin/env python3
"""Replay saved base clouds through radial grasp geometry; no ROS or motion."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src/rim_locator'), str(ROOT / 'src/grasp_executor')]
from rim_locator.config import load_config
from rim_locator.rim_detection import locate_grasp_candidates
from grasp_executor.cartesian import rotate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('clouds', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    config = load_config(ROOT / 'src/rim_locator/config/place_config.json')
    rows = []
    for path in args.clouds:
        data = np.load(path, allow_pickle=False)
        if str(data['frame_id']) != config.target_frame:
            raise ValueError('cloud_frame_mismatch')
        for oid in sorted(set(data['object_ids'].tolist())):
            row = {'cloud': str(path), 'object_id': int(oid)}
            points = data['points'][data['object_ids'] == oid]
            try:
                candidates = locate_grasp_candidates(points, config)
                offsets = []
                for candidate in candidates:
                    if 'circle_center_xy' not in candidate.diagnostics:
                        continue  # Wall fallback has no trusted fitted centre.
                    center = np.array(candidate.diagnostics['circle_center_xy'])
                    delta = np.r_[center - candidate.point[:2], 0.]
                    inverse = tuple(-v for v in candidate.orientation[:3]) + (candidate.orientation[3],)
                    offsets.append(rotate(inverse, delta))
                row.update(accepted=True, candidate_count=len(candidates),
                           # XY geometry only: no claim about slip/height or actual placement error.
                           predicted_center_offset_xy_spread_m=(np.ptp(np.array(offsets)[:,:2],axis=0).tolist()
                                                               if len(offsets) == len(candidates) else None),
                           candidates=[{'position': c.point.tolist(), 'rpy': list(c.rpy),
                                        'geometry': c.diagnostics} for c in candidates])
                if not any(r.get('accepted') for r in rows):
                    import matplotlib
                    matplotlib.use('Agg')
                    import matplotlib.pyplot as plt
                    fig, ax = plt.subplots(figsize=(7,7))
                    ax.scatter(points[::3,0],points[::3,1],s=2,color='gray',alpha=.3,label='observed cloud')
                    c = candidates[0]
                    if 'circle_center_xy' in c.diagnostics:
                        center=np.array(c.diagnostics['circle_center_xy']);radius=c.diagnostics['circle_radius']
                        a=np.linspace(0,2*np.pi,200)
                        ax.plot(center[0]+radius*np.cos(a),center[1]+radius*np.sin(a),'k--',label='fitted circle')
                        ax.plot(*center,'rx',ms=10,label='estimated centre')
                    for i,c in enumerate(candidates):
                        v=np.array(rotate(c.orientation,(0,1,0)))[:2]
                        ax.arrow(*c.point[:2],*(v*.025),width=.0005,color='tab:blue')
                    ax.set(xlabel='base_link X (m)',ylabel='base_link Y (m)',
                           title='Tool +Y direction at each observed grasp contact',aspect='equal')
                    ax.legend();fig.tight_layout();fig.savefig(args.output/'radial_grasps.svg');plt.close(fig)
            except ValueError as exc:
                row.update(accepted=False,reason=str(exc),geometry=getattr(exc,'diagnostics',{}))
            rows.append(row)
    report={'mode':config.orientation_mode,'outside_finger_axis':config.outside_finger_axis,
            'motion_commands_sent':False,'accepted':sum(r['accepted'] for r in rows),
            'rejected':sum(not r['accepted'] for r in rows),'objects':rows}
    (args.output/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='objects'},ensure_ascii=False))
    for r in rows:
        print(Path(r['cloud']).parent.parent.name,r['object_id'],r.get('reason',r.get('predicted_center_offset_xy_spread_m')))


if __name__ == '__main__':
    main()
