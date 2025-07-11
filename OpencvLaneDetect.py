import cv2
import numpy as np
import logging
import math
from PIDSteering import PIDSteering

# ---------- for debugge ----------
SHOW_IMAGE = True          # True면 모든 중간 이미지를 띄움
WIN_SIZE   = (420, 320)    # 각 창 크기 (width, height)
WIN_COLS   = 3             # 한 줄 window count
# ----------------------------------

class OpencvLaneDetect(object):
    def __init__(self):
        self.steer_ctl = PIDSteering(kp=0.55, ki=0.0005, kd=1.1,
                                ema_alpha=0.10, rate_limit=3)
        self.curr_steering_angle = 90
        self.filtered_angle      = 90 

    def get_lane(self, frame):
        show_image("orignal", frame)
        lane_lines, frame = detect_lane(frame)
        return lane_lines, frame

    def get_steering_angle(self, img_lane, lane_lines):
        # ② 차선 없으면 이전 각도 유지
        if len(lane_lines) == 0:
            heading = display_heading_line(img_lane, self.curr_steering_angle)
            return self.curr_steering_angle, heading

        # ③ x-offset 계산 → PIDSteering 업데이트
        x_off = compute_x_offset(img_lane, lane_lines)
        self.curr_steering_angle = self.steer_ctl.update(x_off)

        ALPHA = 0.2                    # 0.0(부드럽다) ↔ 1.0(즉각 반응)
        self.filtered_angle = (ALPHA * self.curr_steering_angle +
                            (1 - ALPHA) * self.filtered_angle)

        # ④ 헤딩 라인 한 번만 그려서 반환
        heading = display_heading_line(img_lane, self.filtered_angle)
        show_image("heading", heading)
        return self.filtered_angle, heading

############################
# Frame processing steps
############################
def detect_lane(frame):
    logging.debug("detecting lane lines (ROI → Edge)...")

    # 1) ---------------- ROI 마스크 & 시각화 ----------------
    roi_frame, polygon, mask = region_of_interest(frame, return_polygon=True)

    # (디버깅) ROI를 반투명하게 오버레이
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    overlay  = cv2.addWeighted(frame, 0.7, mask_bgr, 0.3, 0)
    show_image("ROI overlay", overlay, True)

    # 2) ---------------- Edge 검출 --------------------------
    edges = detect_edges(roi_frame)             # ROI 내부 픽셀만 사용

    # 3) ---------------- 이후 파이프라인 ---------------------
    line_segments = detect_line_segments(edges)
    line_segment_image = display_lines(frame, line_segments)
    show_image("line segments", line_segment_image)

    lane_lines = average_slope_intercept(frame, line_segments)
    lane_lines_image = display_lines(frame, lane_lines)
    show_image("lane lines", lane_lines_image)

    return lane_lines, lane_lines_image



"""
1) BGR → HSV, HLS 변환
2) 흰색 차선 마스크      : S < 60, V > 170
3) 글레어(반사) 마스크   : S < 25, V/L > 230
4) (차선 마스크) − (글레어 마스크)
5) 모폴로지로 정제 → Canny Edge
"""
def detect_edges(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)

    # ---------- (1) 흰색 차선 마스크 ----------
    white = (hsv[...,1] < 60) & (hsv[...,2] > 170)
    white = white.astype(np.uint8) * 255

    # ── (2) 글레어 마스크 (동적) ────────────────────
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
    top_hat = cv2.morphologyEx(hsv[...,2], cv2.MORPH_TOPHAT, kernel)
    white[top_hat < 30] = 0        # 국부 대비가 낮으면 제거

    # 프레임마다 상위 1 % 밝기를 자동 임계값으로 설정
    v95 = np.percentile(hsv[...,2], 99)
    glare = (hsv[...,2] > v95) & (hsv[...,1] < 30) & (hls[...,1] > v95)
    glare = glare.astype(np.uint8) * 255

    mask = cv2.subtract(white, glare)
    show_image("white mask (raw)", mask, True)

    # 노이즈 억제
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)  # 작은 구멍 메우기
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN , k, iterations=1)  # 점·실선 제거
    show_image("noise mask", mask, True)

    # ---------- (5) Gaussian Blur → Canny ----------
    # 3×3 블러로 렌즈플레어처럼 날카로운 잔여 에지 완화  ⬅︎ 추가
    blurred = cv2.GaussianBlur(mask, (3, 3), 0)   # sigmaX=0 (자동)

    # ---------- (5) Canny ----------
    edges = cv2.Canny(blurred, 50, 120)
    show_image("white edge", edges)
    return edges


def region_of_interest(img, return_polygon=False):
    """
    img  : BGR(3채널) 또는 GRAY(1채널) 이미지
    return_polygon=True → (masked_img, polygon, mask) 반환
    """
    h, w = img.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)

    # ▸ ROI 비율 (필요하면 실험으로 조정)
    top_y   = int(h * 0.60)
    left_x  = int(w * 0.15)
    right_x = int(w * 0.95)

    polygon = np.array([[
        (left_x,  top_y),
        (right_x, top_y),
        (w,       h),
        (0,       h)
    ]], dtype=np.int32)

    cv2.fillPoly(mask, polygon, 255)

    # 채널 수(1 or 3)에 맞춰 bitwise_and
    if img.ndim == 3:
        masked = cv2.bitwise_and(img, img, mask=mask)
    else:
        masked = cv2.bitwise_and(img, mask)

    if return_polygon:
        return masked, polygon, mask
    return masked

def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # degree in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=15, maxLineGap=4)

    if line_segments is not None:
        for line_segment in line_segments:
            logging.debug('detected line_segment:')
            logging.debug("%s of length %s" % (line_segment, length_of_line_segment(line_segment[0])))

    return line_segments


def average_slope_intercept(frame, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        logging.info('No line_segment segments detected')
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen
    
    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    if slope < -0.75:
                        left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    if slope > 0.75:
                        right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

    return lane_lines

def compute_steering_angle(frame, lane_lines):
    """ Find the steering angle based on lane line coordinate
        We assume that camera is calibrated to point to dead center
    """
    if len(lane_lines) == 0:
        logging.info('No lane lines detected, do nothing')
        return -90

    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        logging.debug('Only detected one lane line, just follow it. %s' % lane_lines[0])
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        camera_mid_offset_percent = 0.02 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid

    # find the steering angle, which is angle between navigation direction to end of center line
    y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel

    logging.debug('new steering angle: %s' % steering_angle)
    return steering_angle


def stabilize_steering_angle(curr_steering_angle, new_steering_angle, num_of_lane_lines, max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=1):
    """
    Using last steering angle to stabilize the steering angle
    This can be improved to use last N angles, etc
    if new angle is too different from current angle, only turn by max_angle_deviation degrees
    """
    if num_of_lane_lines == 2 :
        # if both lane lines detected, then we can deviate more
        max_angle_deviation = max_angle_deviation_two_lines
    else :
        # if only one lane detected, don't deviate too much
        max_angle_deviation = max_angle_deviation_one_lane
    
    angle_deviation = new_steering_angle - curr_steering_angle
    if abs(angle_deviation) > max_angle_deviation:
        stabilized_steering_angle = int(curr_steering_angle
                                        + max_angle_deviation * angle_deviation / abs(angle_deviation))
    else:
        stabilized_steering_angle = new_steering_angle
    logging.info('Proposed angle: %s, stabilized angle: %s' % (new_steering_angle, stabilized_steering_angle))
    return stabilized_steering_angle


"""
  Utility Functions
"""
def display_lines(frame, lines, line_color=(0, 255, 0), line_width=10):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image


def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5, ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image


def length_of_line_segment(line):
    x1, y1, x2, y2 = line
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


_window_idx = {}           # 창 순서를 기억하는 사전

def show_image(title, frame, show=SHOW_IMAGE):
    """
    디버그 이미지 시각화 도우미
      • 처음 부를 때만 namedWindow → 창 재활용
      • grid 방식으로 위치 자동 배치
    """
    if not show:
        return

    # 1) 창이 이미 만들어졌는지 확인
    if title not in _window_idx:
        _window_idx[title] = len(_window_idx)      # 새 창 번호 부여
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)  # 크기·이동 가능한 창
        cv2.resizeWindow(title, *WIN_SIZE)

        # 2) 창 위치 계산 (그리드 배치)
        idx   = _window_idx[title]
        col   = idx % WIN_COLS
        row   = idx // WIN_COLS
        x_pos = col * WIN_SIZE[0]
        y_pos = row * WIN_SIZE[1]
        cv2.moveWindow(title, x_pos, y_pos)        # 실제 위치 이동

    # 3) 이미지 갱신
    cv2.imshow(title, frame)



def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]

def compute_x_offset(frame, lane_lines):
    height, width, _ = frame.shape
    if len(lane_lines) == 0:
        return 0                         # fallback → 그대로 직진
    if len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        return (x2 + x1) / 2 - width/2
    _, _, lx2, _ = lane_lines[0][0]
    _, _, rx2, _ = lane_lines[1][0]
    return (lx2 + rx2)/2 - width/2
