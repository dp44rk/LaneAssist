#!/usr/bin/env python3
# calibrate_camera.py
import glob, json, os, sys, cv2
import numpy as np

# ────────────────────────────────
# 0. 사용자 설정
# ────────────────────────────────
IMG_GLOB       = "./frame/*.jpg"     # 입력 이미지
PATTERN_SIZE   = (10, 7)              # 체커보드 내부 코너 수 (col, row)
SQUARE_SIZE_MM = 25.0                # 한 칸 한 변 길이[mm]

OUT_JSON       = "calib_params.json"
VIS_DIR        = "calib_visual"      # 코너 검출 결과 저장 (선택)

# ────────────────────────────────
# 1. 코너 검출
# ────────────────────────────────
objp = np.zeros((PATTERN_SIZE[0]*PATTERN_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:PATTERN_SIZE[0], 0:PATTERN_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE_MM          # 실제 단위로 스케일

obj_points, img_points, img_size = [], [], None
os.makedirs(VIS_DIR, exist_ok=True)

for fname in sorted(glob.glob(IMG_GLOB)):
    img  = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if img_size is None:
        img_size = gray.shape[::-1]   # (width, height)

    ok, corners = cv2.findChessboardCorners(gray, PATTERN_SIZE,
                                            cv2.CALIB_CB_ADAPTIVE_THRESH +
                                            cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        print(f"[skip] corners not found: {fname}")
        continue

    # 서브픽셀 정제
    corners = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001))

    obj_points.append(objp)
    img_points.append(corners)

    # (선택) 시각화 저장
    vis = cv2.drawChessboardCorners(img.copy(), PATTERN_SIZE, corners, ok)
    cv2.imwrite(os.path.join(VIS_DIR, os.path.basename(fname)), vis)

print(f"⮕ usable images: {len(obj_points)}")
if len(obj_points) < 5:
    sys.exit("❗ 체커보드가 5장 이상 필요합니다")

# ────────────────────────────────
# 2. 카메라 파라미터 추정
# ────────────────────────────────
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, img_size, None, None)

# ────────────────────────────────
# 3. 결과 저장 (JSON)
# ────────────────────────────────
fx, fy = K[0, 0], K[1, 1]
cx, cy = K[0, 2], K[1, 2]
k1, k2, p1, p2, k3 = dist.ravel()[:5]

out = {
    "image_width" : img_size[0],
    "image_height": img_size[1],
    "fx": fx, "fy": fy, "cx": cx, "cy": cy,
    "k1": k1, "k2": k2, "p1": p1, "p2": p2, "k3": k3,
    "rms_reprojection_error": ret
}
with open(OUT_JSON, "w") as f:
    json.dump(out, f, indent=4)
print(f"✅ saved {OUT_JSON}")
