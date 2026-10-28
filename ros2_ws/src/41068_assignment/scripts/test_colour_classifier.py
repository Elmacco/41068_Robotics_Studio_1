#!/usr/bin/env python3
"""Offline test for colour_classifier.py (no ROS, no Gazebo needed).

    python3 scripts/test_colour_classifier.py

Part 1 uses synthetic images. Part 2 (if tools/textures/ is present) samples
the real red/green Rescue Randy textures to check that red is flagged and
green is not.
"""

import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import colour_classifier as cc  # noqa: E402

P = cc.ColourParams()
FAILS = []


def check(name, got, want):
    ok = got == want
    print(f'  [{"ok" if ok else "FAIL"}] {name}: {got}' + ('' if ok else f' (wanted {want})'))
    if not ok:
        FAILS.append(name)


def hsv_img(h, s, v, shape=(480, 640)):
    img = np.zeros((*shape, 3), np.uint8)
    img[:] = (h, s, v)
    return cv2.cvtColor(img, cv2.COLOR_HSV2BGR)


def scene(torso_bgr, bg_hsv=(55, 100, 120)):
    """Grass-coloured background with a person box (cx=320, cy=240, 100x240)
    whose torso slice is filled with torso_bgr."""
    img = hsv_img(*bg_hsv)
    box = (320, 240, 100, 240)
    x1, y1, x2, y2 = cc.torso_region(box, 640, 480, P)
    img[y1:y2, x1:x2] = torso_bgr
    return img, box


def px(h, s, v):
    return cv2.cvtColor(np.uint8([[[h, s, v]]]), cv2.COLOR_HSV2BGR)[0, 0]


def main():
    print('Synthetic scenes:')
    img, box = scene(px(0, 230, 200))
    check('solid red torso', cc.classify_person(img, box, P)[0], cc.RED)
    img, box = scene(px(176, 230, 200))
    check('red on the high side of hue wrap', cc.classify_person(img, box, P)[0], cc.RED)
    img, box = scene(px(60, 230, 200))
    check('solid green torso is NOT flagged', cc.classify_person(img, box, P)[0], cc.NONE)
    img, box = scene(px(15, 230, 220))
    check('orange torso is NOT flagged', cc.classify_person(img, box, P)[0], cc.NONE)
    img, box = scene(px(0, 0, 130))
    check('grey torso', cc.classify_person(img, box, P)[0], cc.NONE)
    img, box = scene(px(5, 70, 200))
    check('pale pink / skin-tone torso', cc.classify_person(img, box, P)[0], cc.NONE)
    img, box = scene(px(0, 230, 25))
    check('very dark red (below min_val)', cc.classify_person(img, box, P)[0], cc.NONE)

    # Only ~15% of the torso is red: below min_fraction.
    img, box = scene(px(0, 0, 130))
    x1, y1, x2, y2 = cc.torso_region(box, 640, 480, P)
    img[y1:y2, x1:x1 + int((x2 - x1) * 0.15)] = px(0, 230, 200)
    check('15% red is not enough', cc.classify_person(img, box, P)[0], cc.NONE)
    img[y1:y2, x1:x1 + int((x2 - x1) * 0.40)] = px(0, 230, 200)
    check('40% red is enough', cc.classify_person(img, box, P)[0], cc.RED)

    # Colour outside the torso slice (e.g. red legs) must not count.
    img, box = scene(px(0, 0, 130))
    img[int(240 + 0.30 * 240):480, :] = px(0, 230, 200)
    check('red legs only, grey torso', cc.classify_person(img, box, P)[0], cc.NONE)

    # Box partly off the image edge must not crash.
    img = hsv_img(55, 100, 120)
    check('box off the image edge', cc.classify_person(img, (10, 10, 100, 240), P)[0], cc.NONE)
    check('degenerate tiny box', cc.classify_person(img, (100, 100, 3, 3), P)[0], cc.NONE)

    print('Frame-level result:')
    check('any red -> red', cc.overall_flag([cc.NONE, cc.RED, cc.NONE]), cc.RED)
    check('no red -> none', cc.overall_flag([cc.NONE, cc.NONE]), cc.NONE)
    check('no people -> none', cc.overall_flag([]), cc.NONE)

    tex_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools', 'textures')
    red_path = os.path.join(tex_dir, 'rescue_randy_red.png')
    green_path = os.path.join(tex_dir, 'rescue_randy_green.png')
    if os.path.isfile(red_path) and os.path.isfile(green_path):
        print('Real textures (default thresholds):')
        red_img, green_img = cv2.imread(red_path), cv2.imread(green_path)
        for name, img in (('red', red_img), ('green', green_img)):
            _, rf = cc.classify_crop(img, P)
            print(f'  {name} texture sheet (whole): {rf:.0%} of pixels count as red')
        # A patch of solid suit material from each sheet (bottom-right area).
        check('red texture, suit-material patch', cc.classify_crop(red_img[1500:1900, 1300:1900], P)[0], cc.RED)
        check('green texture, suit-material patch is NOT flagged',
              cc.classify_crop(green_img[1500:1900, 1300:1900], P)[0], cc.NONE)
        # Grey jacket panel and face/skin, from the red sheet (identical in the others).
        check('grey central panel of the texture', cc.classify_crop(red_img[300:1000, 700:1200], P)[0], cc.NONE)
        check('face / skin area of the texture', cc.classify_crop(red_img[300:800, 1500:1800], P)[0], cc.NONE)

    print('\nALL PASSED' if not FAILS else f'\n{len(FAILS)} FAILED: {FAILS}')
    sys.exit(1 if FAILS else 0)


if __name__ == '__main__':
    main()
