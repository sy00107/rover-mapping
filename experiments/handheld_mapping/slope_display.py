"""Repair short flat-colour interruptions inside an observed slope, for display only."""
import math

import numpy as np


def continuous_slope_mask(terrain, threshold_deg=5., maximum_gap_m=.15,
                          minimum_support_m=.15, neighbourhood_m=.20,
                          minimum_slope_fraction=.65, plane_tolerance_m=.02):
    """Bridge <=15 cm green runs bracketed by sustained, height-consistent slopes.

    Only the returned colour mask changes. Heights, gradients, semantic ground and
    navigation costs are never changed. A genuine flat strip shorter than the gap
    limit can be indistinguishable from noise; this is an explicit display prior.
    """
    layers = terrain['layers']
    gx = np.asarray(layers['reference_ground_gradient_x'])
    gy = np.asarray(layers['reference_ground_gradient_y'])
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    pink = np.isfinite(slope) & (slope >= threshold_deg)
    stats = dict(method='short_bracketed_slope_colour_gaps_v1', corrected_cells=0,
                 maximum_gap_m=maximum_gap_m, minimum_support_each_side_m=minimum_support_m,
                 neighbourhood_radius_m=neighbourhood_m,
                 minimum_slope_fraction=minimum_slope_fraction,
                 plane_tolerance_m=plane_tolerance_m,
                 display_only=True)
    required = ('ground', 'surface_observed', 'reference_ground_height_m')
    if not all(key in layers for key in required):
        return pink, stats
    res = float(terrain['resolution'])
    if not math.isfinite(res) or res <= 0:
        raise ValueError('invalid slope colour grid resolution')
    height = np.asarray(layers['reference_ground_height_m'])
    known = layers['ground'] & layers['surface_observed'] & np.isfinite(height) & np.isfinite(slope)
    pink &= known
    # All decisions use original seeds, so corrections cannot spread recursively.
    seeds = pink.copy()
    radius = int(math.ceil(neighbourhood_m/res))
    if res > maximum_gap_m+1e-9:
        return pink, stats
    rows, cols = height.shape
    directions = ((0, 1), (1, 0), (1, 1), (1, -1))
    candidates = np.argwhere(known & ~seeds)
    for y, x in candidates:
        bracketed = False
        for dy, dx in directions:
            step = res*math.hypot(dx, dy)
            max_steps = int(math.floor(maximum_gap_m/step+1e-8))+1
            distances = []
            for sign in (-1, 1):
                found = None
                for distance in range(1, max_steps+1):
                    yy, xx = y+sign*dy*distance, x+sign*dx*distance
                    if not (0 <= yy < rows and 0 <= xx < cols) or not known[yy, xx]:
                        break
                    if seeds[yy, xx]:
                        support = max(1, int(math.ceil(minimum_support_m/step-1e-8)))
                        if all(0 <= y+sign*dy*(distance+j) < rows
                               and 0 <= x+sign*dx*(distance+j) < cols
                               and seeds[y+sign*dy*(distance+j), x+sign*dx*(distance+j)]
                               for j in range(support)):
                            found = distance
                        break
                distances.append(found)
            if (None not in distances
                    and (sum(distances)-1)*step <= maximum_gap_m+1e-9):
                bracketed = True
                break
        if not bracketed:
            continue
        ya, yb = max(0, y-radius), min(rows, y+radius+1)
        xa, xb = max(0, x-radius), min(cols, x+radius+1)
        yy, xx = np.mgrid[ya:yb, xa:xb]
        circle = ((yy-y)**2+(xx-x)**2)*res**2 <= neighbourhood_m**2+1e-9
        local_known = known[ya:yb, xa:xb] & circle
        local_seeds = seeds[ya:yb, xa:xb] & circle
        if local_known.sum() < 8 or local_seeds.sum() < 8:
            continue
        if local_seeds.sum()/local_known.sum() < minimum_slope_fraction:
            continue
        coords = np.c_[(xx[local_seeds]-x)*res, (yy[local_seeds]-y)*res,
                       np.ones(local_seeds.sum())]
        # Require a two-dimensional neighbourhood, not a line with undefined plane.
        if np.linalg.matrix_rank(coords) < 3:
            continue
        values = height[ya:yb, xa:xb][local_seeds]
        fit = np.linalg.lstsq(coords, values, rcond=None)[0]
        fitted_slope = np.degrees(np.arctan(np.linalg.norm(fit[:2])))
        residual = values-coords@fit
        if (fitted_slope < 7. or np.sqrt(np.mean(residual**2)) > .015
                or abs(height[y, x]-fit[2]) > plane_tolerance_m):
            continue
        pink[y, x] = True
    stats['corrected_cells'] = int(np.count_nonzero(pink & ~seeds))
    return pink, stats
