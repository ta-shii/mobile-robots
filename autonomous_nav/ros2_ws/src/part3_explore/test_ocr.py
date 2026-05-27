#!/usr/bin/env python3
"""
Standalone test for Greek letter detection logic.
Run this directly — no ROS needed.

Usage:
  # Test with live camera
  python3 test_ocr.py

  # Test with a saved image file
  python3 /home/team9/Desktop/group9/mobile-robots/autonomous_nav/ros2_ws/src/part3_explore/test_ocr.py \
  --image /home/team9/Desktop/group9/mobile-robots/autonomous_nav/ros2_ws/src/part3_explore/photo.png
  
  # With debug visualization (shows intermediate steps)
  python3 test_ocr.py --image photo.png --debug
  
  # Interactive threshold tuning
  python3 test_ocr.py --image photo.png --tune"""

import sys
import argparse
import json
import os
import cv2
import numpy as np

try:
    import torch
    import torch.nn as nn
    import torchvision.models as tv_models
    import torchvision.transforms as T
    from PIL import Image as PILImage

    _CLF_DIR = os.path.join(os.path.dirname(__file__), 'part3_explore', 'greek_ocr', 'classifier')

    with open(os.path.join(_CLF_DIR, 'classes.json')) as _f:
        _idx_to_class = {int(k): v for k, v in json.load(_f).items()}

    _clf_model = tv_models.mobilenet_v2(weights=None)
    _clf_model.classifier[1] = nn.Linear(_clf_model.last_channel, len(_idx_to_class))
    _clf_model.load_state_dict(
        torch.load(os.path.join(_CLF_DIR, 'mobilenet_best.pth'), map_location='cpu')
    )
    _clf_model.eval()
    _clf_device = 'cuda' if torch.cuda.is_available() else 'cpu'
    _clf_model.to(_clf_device)
    _clf_transform = T.Compose([
        T.Resize((224, 224)),
        T.ToTensor(),
        T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])
    _CLASSIFIER_OK = True
    print(f'[Classifier] MobileNetV2 loaded  ({len(_idx_to_class)} classes, device={_clf_device})')
except Exception as e:
    _CLASSIFIER_OK = False
    print(f'[Classifier] not available ({e})')

CONF_THRESH = 60   # minimum classifier confidence (0-100) to accept a detection

# Debug output directory — save next to this script so /workspace isn't needed
_DEBUG_DIR = os.path.dirname(os.path.abspath(__file__))

# ── White paper detection thresholds ───────────────────────────────────────
WHITE_MIN_AREA    = 5000
WHITE_MAX_AREA    = 300000
HSV_MIN           = np.array([0, 0, 170])
HSV_MAX           = np.array([180, 60, 255])
ASPECT_MIN        = 0.5
ASPECT_MAX        = 2.0
RECT_MIN          = 0.7
MORPH_KERNEL_SIZE = 5



def find_white_paper(frame, debug=False):
    """
    Find the white A4 paper and return a perspective-corrected flat crop.
    This handles tilted/angled papers by warping the 4 detected corners.
    
    Args:
        frame: Input image (BGR)
        debug: If True, save intermediate visualization steps
    """
    h, w   = frame.shape[:2]
    roi_y0 = int(h * 0.20)
    roi    = frame[roi_y0:, :]

    # HSV mask: IMPROVED — require very white (high V, very low S)
    hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_MIN, HSV_MAX)
    
    if debug:
        cv2.imwrite(os.path.join(_DEBUG_DIR, 'debug_01_hsv_mask.jpg'), mask)
        print('  [DEBUG] Saved: debug_01_hsv_mask.jpg')

    # Morphology with smaller kernel
    k    = cv2.getStructuringElement(cv2.MORPH_RECT, (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    
    if debug:
        cv2.imwrite(os.path.join(_DEBUG_DIR, 'debug_02_morphology.jpg'), mask)
        print('  [DEBUG] Saved: debug_02_morphology.jpg')

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if debug:
        debug_contours = roi.copy()
        cv2.drawContours(debug_contours, contours, -1, (0, 255, 0), 2)
        cv2.imwrite(os.path.join(_DEBUG_DIR, 'debug_03_all_contours.jpg'), debug_contours)
        print(f'  [DEBUG] Found {len(contours)} contours. Saved: debug_03_all_contours.jpg')

    best_contour, best_area = None, 0
    for i, c in enumerate(contours):
        area = cv2.contourArea(c)
        
        # IMPROVED: Check both min AND max area
        if area < WHITE_MIN_AREA or area > WHITE_MAX_AREA:
            if debug:
                print(f'    Contour {i}: area={area} - rejected (out of range [{WHITE_MIN_AREA}, {WHITE_MAX_AREA}])')
            continue
        
        x, y, cw, ch = cv2.boundingRect(c)
        aspect = cw / float(ch) if ch > 0 else 0
        
        # IMPROVED: Stricter aspect ratio for A4 paper
        if not (ASPECT_MIN < aspect < ASPECT_MAX):
            if debug:
                print(f'    Contour {i}: area={area}, aspect={aspect:.2f} - rejected (aspect out of [{ASPECT_MIN}, {ASPECT_MAX}])')
            continue
        
        # IMPROVED: Check rectangularity (contour should fill ~70%+ of bounding box)
        rect = cv2.minAreaRect(c)
        rect_area = rect[1][0] * rect[1][1]
        rectangularity = area / rect_area if rect_area > 0 else 0
        if rectangularity < RECT_MIN:
            if debug:
                print(f'    Contour {i}: area={area}, aspect={aspect:.2f}, rect={rectangularity:.2f} - rejected (not rectangular)')
            continue
        
        if area > best_area:
            best_area    = area
            best_contour = c
            if debug:
                print(f'    Contour {i}: area={area}, aspect={aspect:.2f}, rect={rectangularity:.2f} ✓ CANDIDATE')

    if best_contour is None:
        if debug:
            print('  [DEBUG] No contours passed filtering!')
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
        if debug:
            print(f'  [DEBUG] 4-corner perspective warp applied')
        return warped, bbox

    # Fallback: plain bounding-box crop
    cropped = frame[fy:fy + ch, x:x + cw]
    if debug:
        print(f'  [DEBUG] Using bounding box crop (no 4-point detection)')
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


def run_classifier(img_bgr):
    """MobileNetV2 classifier on a BGR image crop. Returns (word, conf 0-100)."""
    try:
        h, w  = img_bgr.shape[:2]
        img   = cv2.resize(img_bgr, (256, int(h * 256 / w)), interpolation=cv2.INTER_AREA)
        gray  = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, bw = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        blurred = cv2.GaussianBlur(bw, (3, 3), 0)
        _, binary = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)
        black = np.where(binary == 0)
        if len(black[0]) > 0:
            p = 6
            binary = binary[
                max(0, black[0].min() - p):min(binary.shape[0], black[0].max() + p),
                max(0, black[1].min() - p):min(binary.shape[1], black[1].max() + p),
            ]
        rgb = cv2.cvtColor(binary, cv2.COLOR_GRAY2RGB)
        pil = PILImage.fromarray(rgb)
        px  = _clf_transform(pil).unsqueeze(0).to(_clf_device)
        with torch.no_grad():
            logits = _clf_model(px)
        probs          = torch.softmax(logits, dim=-1)[0]
        conf_val, pred = probs.max(0)
        word = _idx_to_class[pred.item()]
        conf = round(conf_val.item() * 100, 1)
        print(f'  [Classifier] → "{word}"  conf={conf}')
        return word, conf
    except Exception as e:
        print(f'  [WARN] Classifier error: {e}')
        return None, 0


def process_frame(frame, show=True, debug=False, skip_detection=False):
    display = frame.copy()

    if skip_detection:
        # Bypass paper finder — treat the whole image as the paper crop.
        # Useful for og_photos which are already tight crops of the paper.
        print('  [skip-detection] Using full image as paper crop')
        h, w = frame.shape[:2]
        paper = frame
        bbox  = (0, 0, w, h)
    else:
        paper, bbox = find_white_paper(frame, debug=debug)

    if paper is None:
        print('  No white paper detected in frame')
        if show:
            cv2.putText(display, 'No paper found', (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    else:
        bx, by, bw, bh = bbox
        cv2.rectangle(display, (bx, by), (bx + bw, by + bh), (0, 255, 0), 3)
        print(f'  White paper found: bbox=({bx},{by},{bw},{bh})')

        if _CLASSIFIER_OK:
            text, conf = run_classifier(paper)
            engine = 'mobilenet'
        else:
            print('  No classifier available.')
            text, conf, engine = None, 0, 'none'

        if text:
            status = 'ACCEPTED' if conf >= CONF_THRESH else 'LOW CONF'
            print(f'  [{engine}] result: "{text}"  conf={conf}/100  [{status}]')
            colour = (0, 255, 0) if conf >= CONF_THRESH else (0, 165, 255)
            label  = f'{text}  conf={conf}  [{engine}]'
        else:
            print(f'  [{engine}] no text detected')
            colour = (0, 0, 255)
            label  = 'No text'

        cv2.putText(display, label, (bx, by - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0),   4)
        cv2.putText(display, label, (bx, by - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, colour,       2)


    # Save annotated frame
    cv2.imwrite(os.path.join(_DEBUG_DIR, 'debug_frame.jpg'), display)
    print('  Saved annotated frame:    debug_frame.jpg')
    return display


def test_image(path, debug=False, skip_detection=False):
    frame = cv2.imread(path)
    if frame is None:
        print(f'Cannot read image: {path}')
        sys.exit(1)
    print(f'\nTesting image: {path}')
    if debug:
        print('  [DEBUG MODE ON] — Will save intermediate steps')
    process_frame(frame, show=False, debug=debug, skip_detection=skip_detection)
    print('\nCheck debug_frame.jpg for visual output.')
    if debug:
        print('Also check:')
        print('  • debug_01_hsv_mask.jpg     — Raw HSV mask')
        print('  • debug_02_morphology.jpg   — After morphological ops')
        print('  • debug_03_all_contours.jpg — All detected contours')


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
    parser.add_argument('--debug', action='store_true', help='Show detailed debug info and intermediate steps')
    parser.add_argument('--skip-detection', action='store_true',
                        help='Skip white paper detection — use for og_photos which are already cropped')
    args = parser.parse_args()

    if args.image:
        test_image(args.image, debug=args.debug, skip_detection=args.skip_detection)
    else:
        test_live()
