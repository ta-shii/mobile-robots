#!/usr/bin/env python3
"""
test_classifier.py  –  Batch test the MobileNetV2 Greek letter classifier
                        against the og_photos reference images.

No ROS or camera needed.  Run directly from the host or inside the container.

Usage:
  python3 test_classifier.py
  python3 test_classifier.py --photos path/to/folder   # custom photo dir
  python3 test_classifier.py --image  path/to/single.jpg
  python3 test_classifier.py --save-debug               # write preprocessed crops
"""

import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np
import torch
import torch.nn as nn
import torchvision.models as tv_models
import torchvision.transforms as T
from PIL import Image as PILImage

# ---------------------------------------------------------------------------
# Paths — same fallback logic as marker_detector.py
# ---------------------------------------------------------------------------

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

_CLF_DIR = os.path.join(_SCRIPT_DIR, 'part3_explore', 'greek_ocr', 'classifier')
if not os.path.isdir(_CLF_DIR):
    _CLF_DIR = '/workspace/autonomous_nav/ros2_ws/src/part3_explore/part3_explore/greek_ocr/classifier'

_DEFAULT_PHOTOS = os.path.join(_SCRIPT_DIR, 'part3_explore', 'greek_ocr', 'og_photos')
if not os.path.isdir(_DEFAULT_PHOTOS):
    _DEFAULT_PHOTOS = '/workspace/autonomous_nav/ros2_ws/src/part3_explore/part3_explore/greek_ocr/og_photos'

# ---------------------------------------------------------------------------
# Load model
# ---------------------------------------------------------------------------

def load_model():
    classes_path = os.path.join(_CLF_DIR, 'classes.json')
    weights_path = os.path.join(_CLF_DIR, 'mobilenet_best.pth')

    if not os.path.isfile(classes_path):
        sys.exit(f'ERROR: classes.json not found at {classes_path}')
    if not os.path.isfile(weights_path):
        sys.exit(f'ERROR: mobilenet_best.pth not found at {weights_path}')

    with open(classes_path) as f:
        idx_to_class = {int(k): v for k, v in json.load(f).items()}

    model = tv_models.mobilenet_v2(weights=None)
    model.classifier[1] = nn.Linear(model.last_channel, len(idx_to_class))
    model.load_state_dict(torch.load(weights_path, map_location='cpu'))
    model.eval()

    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model.to(device)

    transform = T.Compose([
        T.Resize((224, 224)),
        T.ToTensor(),
        T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])

    print(f'Model loaded  ({len(idx_to_class)} classes, device={device})')
    print(f'Classes: {sorted(idx_to_class.values())}\n')
    return model, idx_to_class, transform, device


# ---------------------------------------------------------------------------
# Preprocessing — mirrors the training script's preprocess_for_inference
# ---------------------------------------------------------------------------

def preprocess(img_bgr: np.ndarray, save_debug_path: str = None) -> PILImage.Image:
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
    if save_debug_path:
        cv2.imwrite(save_debug_path, binary)
    rgb = cv2.cvtColor(binary, cv2.COLOR_GRAY2RGB)
    return PILImage.fromarray(rgb)


# ---------------------------------------------------------------------------
# Inference
# ---------------------------------------------------------------------------

def predict(img_bgr, model, idx_to_class, transform, device, save_debug_path=None):
    pil = preprocess(img_bgr, save_debug_path=save_debug_path)
    px  = transform(pil).unsqueeze(0).to(device)
    with torch.no_grad():
        logits = model(px)
    probs          = torch.softmax(logits, dim=-1)[0]
    conf_val, pred = probs.max(0)
    word = idx_to_class[pred.item()]
    conf = round(conf_val.item() * 100, 1)

    # Top-3 predictions for debugging
    top3_vals, top3_idx = probs.topk(min(3, len(idx_to_class)))
    top3 = [(idx_to_class[i.item()], round(v.item() * 100, 1))
            for v, i in zip(top3_vals, top3_idx)]
    return word, conf, top3


# ---------------------------------------------------------------------------
# Batch test against og_photos
# ---------------------------------------------------------------------------

def run_batch(photos_dir, model, idx_to_class, transform, device, save_debug):
    extensions = ['*.jpg', '*.JPG', '*.jpeg', '*.JPEG', '*.png', '*.PNG']
    files = []
    for ext in extensions:
        files.extend(glob.glob(os.path.join(photos_dir, ext)))
    files = sorted(set(files))

    if not files:
        sys.exit(f'No images found in {photos_dir}')

    print(f'Found {len(files)} images in {photos_dir}\n')
    print(f'{"File":<25} {"Expected":<12} {"Predicted":<12} {"Conf":>6}  {"Top-3"}')
    print('-' * 80)

    correct = 0
    results = []

    for path in files:
        fname    = os.path.basename(path)
        expected = fname.split('_')[0].lower()

        img = cv2.imread(path)
        if img is None:
            print(f'  SKIP  {fname} — could not read')
            continue

        debug_path = None
        if save_debug:
            debug_dir = os.path.join(_SCRIPT_DIR, 'debug_crops')
            os.makedirs(debug_dir, exist_ok=True)
            debug_path = os.path.join(debug_dir, fname)

        word, conf, top3 = predict(img, model, idx_to_class, transform, device,
                                   save_debug_path=debug_path)

        match  = '✓' if word.lower() == expected else '✗'
        top3_s = '  '.join(f'{n}({c:.0f}%)' for n, c in top3)
        print(f'{match} {fname:<24} {expected:<12} {word:<12} {conf:>5.1f}%  {top3_s}')

        if word.lower() == expected:
            correct += 1
        results.append({'file': fname, 'expected': expected,
                        'predicted': word, 'conf': conf, 'correct': word.lower() == expected})

    total = len(results)
    print('-' * 80)
    print(f'\nAccuracy: {correct}/{total} = {correct/total*100:.0f}%' if total else 'No results.')

    # Per-class breakdown
    classes = sorted(set(r['expected'] for r in results))
    if len(classes) > 1:
        print('\nPer-class accuracy:')
        for cls in classes:
            cls_results = [r for r in results if r['expected'] == cls]
            cls_correct = sum(1 for r in cls_results if r['correct'])
            print(f'  {cls:<12} {cls_correct}/{len(cls_results)}')


# ---------------------------------------------------------------------------
# Single image test
# ---------------------------------------------------------------------------

def run_single(path, model, idx_to_class, transform, device, save_debug):
    img = cv2.imread(path)
    if img is None:
        sys.exit(f'Cannot read: {path}')

    debug_path = os.path.join(_SCRIPT_DIR, 'debug_single_crop.jpg') if save_debug else None
    word, conf, top3 = predict(img, model, idx_to_class, transform, device,
                               save_debug_path=debug_path)

    fname    = os.path.basename(path)
    expected = fname.split('_')[0].lower()
    match    = '✓' if word.lower() == expected else '✗'

    print(f'Image:     {path}')
    print(f'Expected:  {expected}')
    print(f'Predicted: {word}  ({conf:.1f}%)  {match}')
    print(f'Top-3:     {", ".join(f"{n} ({c:.1f}%)" for n, c in top3)}')
    if save_debug:
        print(f'Debug crop saved: {debug_path}')


# ---------------------------------------------------------------------------

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Test MobileNetV2 Greek letter classifier')
    parser.add_argument('--photos',     default=None,
                        help='Directory of test images (default: og_photos)')
    parser.add_argument('--image',      default=None,
                        help='Single image to test')
    parser.add_argument('--save-debug', action='store_true',
                        help='Save preprocessed crops to debug_crops/')
    args = parser.parse_args()

    model, idx_to_class, transform, device = load_model()

    if args.image:
        run_single(args.image, model, idx_to_class, transform, device, args.save_debug)
    else:
        photos_dir = args.photos or _DEFAULT_PHOTOS
        run_batch(photos_dir, model, idx_to_class, transform, device, args.save_debug)
