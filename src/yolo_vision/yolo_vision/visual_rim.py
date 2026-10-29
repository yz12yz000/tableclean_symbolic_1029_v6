"""Visual occupancy geometry only; the original rim grasp is unchanged."""
from dataclasses import replace
import numpy as np

from rim_locator.config import RimConfig
from rim_locator.rim_detection import locate_rim
from rim_locator.circle_fit import fit_rim_circle


def estimate_visual_rim(world, config):
    # Reuse the supported circle fit. A second height band may recover a rim
    # fragmented by reflection; it must pass the SAME support/error gates.
    base = replace(RimConfig(), circle_min_radius=config.get('visual_rim_min_radius', .04),
                   circle_max_radius=config.get('visual_rim_max_radius', .15),
                   circle_min_inlier_ratio=.40)
    fits = []
    for percentile in (98., 95.):
        try:
            rim = locate_rim(world, replace(base, top_percentile=percentile))
            center, fit = fit_rim_circle(rim.candidates, base)
            if (fit['circle_inlier_ratio'] < .55 and not
                    (fit['circle_inlier_count'] >= 100 and fit['circle_coverage_deg'] >= 180
                     and fit['circle_sector_count'] >= 5)):
                continue
            fits.append((center, fit, rim.z_top, rim.candidates))
        except ValueError:
            continue
    if not fits:
        raise ValueError("visual_rim_no_supported_circle")
    center, fit, z_top, rim_points = max(fits, key=lambda item: item[1]['circle_coverage_deg'])
    if any(np.linalg.norm(other - center) > .008 or
           abs(meta['circle_radius'] - fit['circle_radius']) > .008
           for other, meta, _, _ in fits):
        raise ValueError("visual_rim_inconsistent_height_bands")
    # The circle can follow the inner lip. Include the observed outer body and
    # a configurable lip allowance, never replace it with a known bowl size.
    # Only the supported rim-height band contributes outer extent. Table
    # pixels leaked into a mask lie lower and must not inflate the bowl.
    radial = np.linalg.norm(rim_points[:, :2] - center, axis=1)
    outer = float(np.percentile(radial, 90))
    if outer > fit['circle_radius'] + .025:
        raise ValueError("visual_rim_inconsistent_footprint")
    physical = max(outer, fit['circle_radius']) + config['rim_outer_margin']
    uncertainty = max(.003, 2 * fit['circle_residual'])
    if physical > .17 or uncertainty > .006:
        raise ValueError("visual_rim_uncertain_size")
    # Current occupancy needs measurement allowance. Future placement error
    # is added by the grid search, not to bowls already observed in the tray.
    return dict(center_xy=center.tolist(), center_z=float(z_top),
                estimated_radius=physical, estimated_diameter=2 * physical,
                observed_rim_radius=fit['circle_radius'], size_uncertainty=uncertainty,
                radius=physical + uncertainty, rim_fit_residual=fit['circle_residual'],
                rim_fit_coverage_deg=fit['circle_coverage_deg'],
                rim_fit_inlier_ratio=fit['circle_inlier_ratio'])
