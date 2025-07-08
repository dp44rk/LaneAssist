import cv2
import glob
import os
from OpencvLaneDetect import OpencvLaneDetect

def main():
    detector     = OpencvLaneDetect()
    image_paths  = sorted(glob.glob("./frame/*.jpg"))

    if not image_paths:
        raise FileNotFoundError("./frame 폴더에 JPG 이미지가 없습니다.")

    paused = False          # 재생/ 일시정지 상태 표시

    for idx, img_path in enumerate(image_paths, 1):
        frame = cv2.imread(img_path)
        if frame is None:
            print(f"[{idx}] {os.path.basename(img_path)}: 읽기 실패 → 건너뜀")
            continue

        lane_lines, lane_img = detector.get_lane(frame)
        angle, heading_img   = detector.get_steering_angle(frame, lane_lines)
        if heading_img is None or heading_img.size == 0:
            heading_img = frame.copy() 

        cv2.putText(heading_img, f"{angle:.1f} deg", (30,50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)
        cv2.imshow("Heading", heading_img)

        # ----------------- 키 입력 처리 ----------------- #
        key = cv2.waitKey(0 if paused else 30) & 0xFF   # paused=True면 무한 대기

        if key == ord('q'):         # q → 종료
            break
        elif key == ord(' '):       # 스페이스 → 토글
            paused = not paused
            print("❚❚ Paused" if paused else "▶ Resumed")

        print(f"[{idx}/{len(image_paths)}] {os.path.basename(img_path)} → angle = {angle:.1f}°")

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
