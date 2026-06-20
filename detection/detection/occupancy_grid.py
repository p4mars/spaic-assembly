"""
Standalone grid analysis test — no ROS needed.
Detects the ArUco marker, projects a 5x5 grid, checks occupancy.

Usage:
    python3 test_grid.py --image path/to/image.jpg

Tuning:
    - MARKER_SIZE_M  : measure your physical marker
    - GRID_ORIGIN_X/Y/Z: shift the grid until lines align with real blocks
    - CELL_SIZE_M    : measure your physical blocks
"""

import argparse
import cv2
import numpy as np

# ── Configuration ─────────────────────────────────────────────────────────────

MARKER_SIZE_M  = 0.03
ARUCO_DICT     = cv2.aruco.DICT_4X4_1000

CELL_SIZE_M    = 0.03
GRID_ROWS      = 5
GRID_COLS      = 5

GRID_ORIGIN_X  = 0.03
GRID_ORIGIN_Y  = -0.06
GRID_ORIGIN_Z  = -0.03

CELL_SAMPLE_PX = 15

BLUE_HSV_LOW   = (90,  50,  50)
BLUE_HSV_HIGH  = (140, 255, 255)
OCCUPANCY_THRESHOLD = 0.30

CAMERA_K = np.array([
    827.01,   0.0,    306.50,
      0.0,  817.84,   221.24,
      0.0,    0.0,      1.0,
], dtype=np.float64).reshape(3, 3)
#CAMERA_D = np.array(
#    [-1.147382, 21.242253, 0.013422, 0.000006, -147.716365],
#    dtype=np.float64
#)
# seems to work better without the calibrated D params and undistoring the image using opencv
CAMERA_D = np.zeros(5, dtype=np.float64)


# -- Helpers --
def get_rvec_tvec(corners):
    half = MARKER_SIZE_M / 2
    marker_3d = np.array([
        [-half,  half, 0],
        [ half,  half, 0],
        [ half, -half, 0],
        [-half, -half, 0],
    ], dtype=np.float32)
    success, rvec, tvec = cv2.solvePnP(
        marker_3d, corners.reshape(4, 2), CAMERA_K, CAMERA_D)
    return (rvec, tvec) if success else (None, None)


def project(pt, rvec, tvec):
    px, _ = cv2.projectPoints(
        np.array(pt, dtype=np.float32).reshape(1, 1, 3),
        rvec, tvec, CAMERA_K, CAMERA_D)
    return tuple(px[0][0].astype(int))


def build_occupancy_grid(image, rvec, tvec):
    h, w  = image.shape[:2]
    hsv   = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    blue  = cv2.inRange(hsv, BLUE_HSV_LOW, BLUE_HSV_HIGH)
    result = {}
    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            cx, cy = project(
                [GRID_ORIGIN_X + c * CELL_SIZE_M,
                 GRID_ORIGIN_Y + r * CELL_SIZE_M,
                 GRID_ORIGIN_Z],
                rvec, tvec)
            x0, x1 = max(cx - CELL_SAMPLE_PX, 0), min(cx + CELL_SAMPLE_PX, w)
            y0, y1 = max(cy - CELL_SAMPLE_PX, 0), min(cy + CELL_SAMPLE_PX, h)
            if x1 <= x0 or y1 <= y0:
                result[(r, c)] = False
                continue
            roi = blue[y0:y1, x0:x1]
            result[(r, c)] = float((roi > 0).mean()) > OCCUPANCY_THRESHOLD
    return result


def draw_debug(image, rvec, tvec, occupancy):
    out = image.copy()

    # Grid lines
    for r in range(GRID_ROWS + 1):
        y  = GRID_ORIGIN_Y + r * CELL_SIZE_M - CELL_SIZE_M / 2
        p1 = project([GRID_ORIGIN_X - CELL_SIZE_M / 2, y, GRID_ORIGIN_Z], rvec, tvec)
        p2 = project([GRID_ORIGIN_X + GRID_COLS * CELL_SIZE_M - CELL_SIZE_M / 2, y, GRID_ORIGIN_Z], rvec, tvec)
        cv2.line(out, p1, p2, (255, 255, 0), 1)
    for c in range(GRID_COLS + 1):
        x  = GRID_ORIGIN_X + c * CELL_SIZE_M - CELL_SIZE_M / 2
        p1 = project([x, GRID_ORIGIN_Y - CELL_SIZE_M / 2, GRID_ORIGIN_Z], rvec, tvec)
        p2 = project([x, GRID_ORIGIN_Y + GRID_ROWS * CELL_SIZE_M - CELL_SIZE_M / 2, GRID_ORIGIN_Z], rvec, tvec)
        cv2.line(out, p1, p2, (255, 255, 0), 1)

    # Cell centres — green if occupied, red if empty
    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            cx, cy = project(
                [GRID_ORIGIN_X + c * CELL_SIZE_M,
                 GRID_ORIGIN_Y + r * CELL_SIZE_M,
                 GRID_ORIGIN_Z],
                rvec, tvec)
            color = (0, 255, 0) if occupancy.get((r, c)) else (0, 0, 255)
            cv2.circle(out, (cx, cy), 4, color, -1)
            cv2.putText(out, f'{r},{c}', (cx - 10, cy - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.28, color, 1)

    # Closest cell + magenta connection line
    cell_centers  = {
        (r, c): np.array([GRID_ORIGIN_X + c * CELL_SIZE_M,
                          GRID_ORIGIN_Y + r * CELL_SIZE_M,
                          GRID_ORIGIN_Z])
        for r in range(GRID_ROWS) for c in range(GRID_COLS)
    }
    marker_origin = np.array([0.0, 0.0, 0.0])
    closest_cell  = min(cell_centers, key=lambda k: np.linalg.norm(cell_centers[k] - marker_origin))
    closest_pt    = cell_centers[closest_cell]
    dist          = np.linalg.norm(closest_pt - marker_origin)

    for i in range(20):
        t0, t1 = i / 20, (i + 1) / 20
        pa = project(marker_origin * (1 - t0) + closest_pt * t0, rvec, tvec)
        pb = project(marker_origin * (1 - t1) + closest_pt * t1, rvec, tvec)
        cv2.line(out, pa, pb, (255, 0, 255), 2)

    marker_px   = project([0, 0, 0], rvec, tvec)
    closest_px  = project(closest_pt, rvec, tvec)
    cv2.circle(out, marker_px, 7, (0, 0, 255), -1)
    cv2.putText(out, 'marker', (marker_px[0] + 6, marker_px[1] - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
    cv2.putText(out, f'closest: {closest_cell} ({dist*100:.1f}cm)',
                (closest_px[0] + 6, closest_px[1] + 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 128, 255), 1)

    return out


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', required=True)
    parser.add_argument('--save',  default='occupancy_grid_debug.jpg')
    args = parser.parse_args()

    image = cv2.imread(args.image)
    if image is None:
        print(f'ERROR: could not load {args.image}')
        return

    image = cv2.undistort(image, CAMERA_K, CAMERA_D)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)      
    detector   = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is None:
        print('ERROR: no ArUco marker detected.')
        return
    print(f'Detected marker IDs: {ids.flatten()}')

    # We assume there is only one marker in the image
    rvec, tvec = get_rvec_tvec(corners[0])
    if rvec is None:
        print('ERROR: solvePnP failed.')
        return
    print(f'tvec: {tvec.flatten()}')

    occupancy = build_occupancy_grid(image, rvec, tvec)

    print('\nOccupancy (O=occupied, .=empty):')
    for c in range(GRID_COLS - 1, -1, -1):   # high c first → physical top first
        print('  ' + '  '.join(
            'O' if occupancy.get((r, c)) else '.' for r in range(GRID_ROWS - 1, -1, -1)))  # high r first → physical left first

    missing = [cell for cell, v in occupancy.items() if not v]
    print(f'\nMissing cells: {missing}')

    debug = draw_debug(image, rvec, tvec, occupancy)
    cv2.imwrite(args.save, debug)
    print(f'\nSaved: {args.save}')

    try:
        cv2.imshow('Grid Debug', debug)
        print('Press any key to close.')
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except Exception:
        pass


if __name__ == '__main__':
    main()