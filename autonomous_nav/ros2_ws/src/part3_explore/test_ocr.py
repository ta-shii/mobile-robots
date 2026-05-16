#!/usr/bin/env python3
"""
Standalone test for Greek letter detection logic.
Run this directly — no ROS needed.

Usage:
  # Test with live camera
  python3 test_ocr.py

  # Test with a saved image file
  python3 /workspace/autonomous_nav/ros2_ws/src/part3_explore/test_ocr.py \
  --image /workspace/autonomous_nav/ros2_ws/src/part3_explore/photo.png"""

import sys
import argparse
import cv2
import numpy as np
import pytesseract

# ── OCR config ─────────────────────────────────────────────────────────────
# NOTE: LSTM engine (oem 1/3) ignores tessedit_char_whitelist — don't use it.
# Instead we post-process results to fix digit/letter lookalikes.
TESS_CONFIGS = [
    '--psm 8 -l eng --oem 1',   # single word — best for large handwritten single char
    '--psm 7 -l eng --oem 1',   # single line — fallback
]
CONF_THRESH    = 30   # lowered for testing — raise to 50 in production
WHITE_MIN_AREA = 3000

# Post-processing: fix digit/letter lookalikes that Tesseract commonly confuses
_DIGIT_FIX = {'8': 'B', '0': 'O', '1': 'I', '5': 'S', '2': 'Z', '6': 'G'}


def preprocess(img):
    """
    1. Convert to grayscale and threshold to find dark ink on white paper.
    2. Find the largest dark contour cluster (the letter itself).
    3. Crop tight to just the letter + padding — removes tape/edge noise.
    4. Resize to fixed height and add border for Tesseract.
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Threshold: find dark ink pixels
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # Clean small noise
    k      = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, k)

    # Find all dark contours
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        # Fallback: return full image
        return _resize_and_pad(255 - binary)

    # Keep only contours away from all edges (excludes tape, floor, background)
    h, w    = binary.shape
    x_margin = int(w * 0.15)
    y_margin = int(h * 0.10)
    central_contours = [
        c for c in contours
        if cv2.contourArea(c) > 80
        and cv2.boundingRect(c)[0] > x_margin                          # left edge
        and cv2.boundingRect(c)[0] + cv2.boundingRect(c)[2] < w - x_margin  # right edge
        and cv2.boundingRect(c)[1] > y_margin                          # top edge
        and cv2.boundingRect(c)[1] + cv2.boundingRect(c)[3] < h - y_margin  # bottom edge
    ]

    if not central_contours:
        central_contours = contours  # fallback: use all

    # Get bounding box covering all central contours (the full letter)
    all_pts = np.vstack([c.reshape(-1, 2) for c in central_contours])
    lx, ly, lw, lh = cv2.boundingRect(all_pts)

    # Crop to letter + 20% padding
    pad = int(max(lw, lh) * 0.20)
    x1  = max(0, lx - pad)
    y1  = max(0, ly - pad)
    x2  = min(w, lx + lw + pad)
    y2  = min(h, ly + lh + pad)

    letter_crop = gray[y1:y2, x1:x2]

    # Re-threshold the clean crop
    _, clean = cv2.threshold(letter_crop, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    return _resize_and_pad(clean)


def _resize_and_pad(img):
    h, w  = img.shape
    scale = 300 / max(h, w)   # fit into 300x300
    img   = cv2.resize(img, (int(w * scale), int(h * scale)),
                       interpolation=cv2.INTER_CUBIC)
    # Add 60px white border
    return cv2.copyMakeBorder(img, 60, 60, 60, 60,
                              cv2.BORDER_CONSTANT, value=255)


def find_white_paper(frame):
    """
    Find the white A4 paper and return a perspective-corrected flat crop.
    This handles tilted/angled papers by warping the 4 detected corners.
    """
    h, w   = frame.shape[:2]
    roi_y0 = int(h * 0.20)
    roi    = frame[roi_y0:, :]

    # HSV mask: true white = high brightness, low saturation
    hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, 0, 180]), np.array([180, 60, 255]))

    k    = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_contour, best_area = None, 0
    for c in contours:
        area = cv2.contourArea(c)
        if area < WHITE_MIN_AREA:
            continue
        x, y, cw, ch = cv2.boundingRect(c)
        aspect = cw / float(ch) if ch > 0 else 0
        if 0.25 < aspect < 4.0 and area > best_area:
            best_area    = area
            best_contour = c

    if best_contour is None:
        return None, None

    # Try to get 4 corners for perspective warp
    x, y, cw, ch = cv2.boundingRect(best_contour)
    fy = y + roi_y0
    bbox = (x, fy, cw, ch)

    # Approximate contour to polygon — if we get 4 points, do perspective warp
    peri   = cv2.arcLength(best_contour, True)
    approx = cv2.approxPolyDP(best_contour, 0.02 * peri, True)

    if len(approx) == 4:
        pts = approx.reshape(4, 2).astype(np.float32)
        # Shift pts back to full-frame coordinates
        pts[:, 1] += roi_y0
        warped = _four_point_transform(frame, pts)
        return warped, bbox

    # Fallback: plain bounding-box crop
    cropped = frame[fy:fy + ch, x:x + cw]
    return cropped, bbox


def _four_point_transform(img, pts):
    """Warp a quadrilateral region to a flat rectangle."""
    # Order points: top-left, top-right, bottom-right, bottom-left
    rect = _order_points(pts)
    tl, tr, br, bl = rect

    w = int(max(np.linalg.norm(br - bl), np.linalg.norm(tr - tl)))
    h = int(max(np.linalg.norm(tr - br), np.linalg.norm(tl - bl)))
    w, h = max(w, 1), max(h, 1)

    dst = np.array([[0, 0], [w-1, 0], [w-1, h-1], [0, h-1]], dtype=np.float32)
    M   = cv2.getPerspectiveTransform(rect, dst)
    return cv2.warpPerspective(img, M, (w, h))


def _order_points(pts):
    rect = np.zeros((4, 2), dtype=np.float32)
    s    = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)
    rect[0] = pts[np.argmin(s)]     # top-left
    rect[2] = pts[np.argmax(s)]     # bottom-right
    rect[1] = pts[np.argmin(diff)]  # top-right
    rect[3] = pts[np.argmax(diff)]  # bottom-left
    return rect


def run_tesseract(img):
    best_text, best_conf = None, 0

    for config in TESS_CONFIGS:
        try:
            data = pytesseract.image_to_data(
                img, config=config, output_type=pytesseract.Output.DICT
            )
        except Exception as e:
            print(f'  [WARN] Tesseract error with config "{config}": {e}')
            continue

        for text, conf in zip(data['text'], data['conf']):
            text = text.strip()
            conf = int(conf)
            if text and conf > best_conf:
                best_conf = conf
                fixed = _DIGIT_FIX.get(text, text)
                if fixed != text:
                    print(f'  Post-process: "{text}" → "{fixed}"')
                best_text = fixed
                print(f'  [{config}] → "{best_text}"  conf={conf}')

    return best_text, best_conf


def process_frame(frame, show=True):
    display = frame.copy()

    paper, bbox = find_white_paper(frame)
    if paper is None:
        print('  No white paper detected in frame')
        if show:
            cv2.putText(display, 'No paper found', (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    else:
        bx, by, bw, bh = bbox
        cv2.rectangle(display, (bx, by), (bx + bw, by + bh), (0, 255, 0), 3)
        print(f'  White paper found: bbox=({bx},{by},{bw},{bh})')

        preprocessed = preprocess(paper)
        text, conf = run_tesseract(preprocessed)

        if text:
            status = 'ACCEPTED' if conf >= CONF_THRESH else 'LOW CONF'
            print(f'  Tesseract result: "{text}"  conf={conf}/100  [{status}]')
            colour = (0, 255, 0) if conf >= CONF_THRESH else (0, 165, 255)
            label  = f'{text}  conf={conf}'
        else:
            print('  Tesseract: no text detected')
            colour = (0, 0, 255)
            label  = 'No text'

        cv2.putText(display, label, (bx, by - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0),   4)
        cv2.putText(display, label, (bx, by - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, colour,       2)

        # Save preprocessed crop so we can inspect what Tesseract sees
        cv2.imwrite('/workspace/autonomous_nav/debug_paper.jpg', preprocessed)
        print('  Saved preprocessed crop: debug_paper.jpg')

    # Save annotated frame
    cv2.imwrite('/workspace/autonomous_nav/debug_frame.jpg', display)
    print('  Saved annotated frame:    debug_frame.jpg')
    return display


def test_image(path):
    frame = cv2.imread(path)
    if frame is None:
        print(f'Cannot read image: {path}')
        sys.exit(1)
    print(f'\nTesting image: {path}')
    process_frame(frame, show=False)
    print('\nCheck debug_frame.jpg and debug_paper.jpg for visual output.')


def test_live():
    print('\nOpening camera (index 0) ...')
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print('Cannot open camera. Try --image <file> instead.')
        sys.exit(1)

    print('Camera open. Hold the A4 paper with the letter in front.')
    print('Press Q to quit, S to save current frame.\n')

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        print('---')
        process_frame(frame, show=True)

        key = cv2.waitKey(500) & 0xFF   # process at ~2 Hz
        if key == ord('q'):
            break
        if key == ord('s'):
            cv2.imwrite('test_capture.jpg', frame)
            print('  Saved: test_capture.jpg')

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', help='Path to image file to test')
    args = parser.parse_args()

    if args.image:
        test_image(args.image)
    else:
        test_live()
