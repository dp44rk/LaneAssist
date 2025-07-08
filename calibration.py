# calibration.py
import json, math, numpy as np, cv2

def _Rx(r): c,s=math.cos(r),math.sin(r);return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def _Ry(p): c,s=math.cos(p),math.sin(p);return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def _Rz(y): c,s=math.cos(y),math.sin(y);return np.array([[c,-s,0],[s,c,0],[0,0,1]])

class Calibration:
    def __init__(self, json_path="./calibration.json"):
        with open(json_path) as f:
            self.C = json.load(f)
        self._build_mats()

    # ---------- public API ----------
    def H(self):          # Homography (road→image)
        return self._H.copy()

    def invH(self):       # Inverse (image→road)
        return self._H_inv.copy()

    def undistort(self, img):
        if not hasattr(self, "_map1"):
            self._map1, self._map2 = cv2.initUndistortRectifyMap(
                self._K, self._dist, None, self._K,
                (img.shape[1], img.shape[0]), cv2.CV_16SC2)
        return cv2.remap(img, self._map1, self._map2, cv2.INTER_LINEAR)

    # ---------- internal ----------
    def _build_mats(self):
        C = self.C
        self._K = np.array([[C["fx"],0,C["u0"]],
                            [0,C["fy"],C["v0"]],
                            [0,0,1.0]])
        self._dist = np.array([[C.get(k,0) for k in ("k1","k2","p1","p2","k3")]], np.float64)

        R = _Rz(C["yaw"]) @ _Ry(C["pitch"]) @ _Rx(C["roll"])
        t = np.array([[C["x"]],[C["y"]],[C["z"]]])
        self._H = self._K @ np.hstack([R[:,:2], t])
        self._H /= self._H[2,2]
        self._H_inv = np.linalg.inv(self._H)
