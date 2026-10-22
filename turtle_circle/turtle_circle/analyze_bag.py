#!/usr/bin/env python3
"""Analyze a rosbag2 recording of the turtle_circle run.

Reads ``/turtle1/cmd_vel`` (the velocity commands that were *sent*) and
``/turtle1/pose`` (the turtle *position* reported by turtlesim), then plots
velocity and position versus time.

Usage::

    ros2 run turtle_circle analyze_bag --bag <bag_dir> [--out docs/plots]
"""

import argparse
import csv
import os
import sys

import numpy as np
import yaml

import matplotlib
matplotlib.use('Agg')  # headless: just write PNG files
import matplotlib.pyplot as plt
from matplotlib import font_manager

from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TWIST_TYPE = 'geometry_msgs/msg/Twist'
POSE_TYPE = 'turtlesim/msg/Pose'

# --------------------------------------------------------------------------
# CJK font handling: use a Chinese font when available, fall back to English
# --------------------------------------------------------------------------
_CJK_CANDIDATES = [
    'Noto Sans CJK SC', 'Noto Sans CJK JP', 'Noto Sans SC',
    'WenQuanYi Zen Hei', 'WenQuanYi Micro Hei', 'Source Han Sans CN',
    'AR PL UMing CN', 'SimHei', 'Microsoft YaHei',
]


def pick_cjk_font():
    available = {f.name for f in font_manager.fontManager.ttflist}
    for name in _CJK_CANDIDATES:
        if name in available:
            return name
    return None


CJK = pick_cjk_font()
if CJK:
    plt.rcParams['font.sans-serif'] = [CJK]
    plt.rcParams['axes.unicode_minus'] = False


def L(zh: str, en: str) -> str:
    """Bilingual label helper: Chinese if a CJK font is available, else English."""
    return zh if CJK else en


# --------------------------------------------------------------------------
# bag reading
# --------------------------------------------------------------------------
def read_storage_id(bag_dir: str) -> str:
    meta = os.path.join(bag_dir, 'metadata.yaml')
    with open(meta, 'r', encoding='utf-8') as fh:
        info = yaml.safe_load(fh)['rosbag2_bagfile_information']
    return info.get('storage_identifier', 'mcap')


def read_bag(bag_dir: str):
    """Return (cmd_list, pose_list) where each entry is (t_seconds, msg)."""
    reader = SequentialReader()
    storage_id = read_storage_id(bag_dir)
    reader.open(
        StorageOptions(uri=bag_dir, storage_id=storage_id),
        ConverterOptions(input_serialization_format='cdr',
                         output_serialization_format='cdr'),
    )

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    cmd_topic = next((n for n, t in type_map.items() if t == TWIST_TYPE), None)
    pose_topic = next((n for n, t in type_map.items() if t == POSE_TYPE), None)
    if cmd_topic is None or pose_topic is None:
        raise RuntimeError(
            'bag %s does not contain both a Twist and a Pose topic (found: %s)'
            % (bag_dir, type_map)
        )

    msg_cls = {cmd_topic: get_message(TWIST_TYPE), pose_topic: get_message(POSE_TYPE)}
    cmds, poses = [], []
    t0 = None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in msg_cls:
            continue
        t = stamp / 1e9
        if t0 is None:
            t0 = t
        msg = deserialize_message(data, msg_cls[topic])
        (cmds if topic == cmd_topic else poses).append((t - t0, msg))

    return cmds, poses


# --------------------------------------------------------------------------
# plotting
# --------------------------------------------------------------------------
def _style(ax, title, xlabel, ylabel):
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, linestyle='--', alpha=0.4)


def make_plots(cmds, poses, out_dir: str) -> dict:
    os.makedirs(out_dir, exist_ok=True)

    ct = np.array([t for t, _ in cmds])
    lin = np.array([m.linear.x for _, m in cmds])
    ang = np.array([m.angular.z for _, m in cmds])

    pt = np.array([t for t, _ in poses])
    px = np.array([m.x for _, m in poses])
    py = np.array([m.y for _, m in poses])
    pth = np.array([m.theta for _, m in poses])

    tlab = L('时间 t / s', 'time t / s')
    saved = {}

    # 1) velocity vs time ---------------------------------------------------
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    axes[0].plot(ct, lin, color='tab:blue')
    _style(axes[0], L('发送的速度指令 - 线速度', 'Sent velocity command - linear'),
           '', L('线速度 v / (m/s)', 'linear v / (m/s)'))
    axes[1].plot(ct, ang, color='tab:red')
    _style(axes[1], L('发送的速度指令 - 角速度', 'Sent velocity command - angular'),
           tlab, L('角速度 ω / (rad/s)', 'angular ω / (rad/s)'))
    fig.tight_layout()
    p = os.path.join(out_dir, 'velocity_vs_time.png')
    fig.savefig(p, dpi=130)
    plt.close(fig)
    saved['velocity'] = p

    # 2) position vs time ---------------------------------------------------
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(pt, px, color='tab:green')
    _style(axes[0], L('乌龟位置 - x', 'Turtle position - x'),
           '', L('x / m', 'x / m'))
    axes[1].plot(pt, py, color='tab:orange')
    _style(axes[1], L('乌龟位置 - y', 'Turtle position - y'),
           '', L('y / m', 'y / m'))
    axes[2].plot(pt, pth, color='tab:purple')
    _style(axes[2], L('乌龟姿态 - θ', 'Turtle orientation - θ'),
           tlab, L('θ / rad', 'θ / rad'))
    fig.tight_layout()
    p = os.path.join(out_dir, 'position_vs_time.png')
    fig.savefig(p, dpi=130)
    plt.close(fig)
    saved['position'] = p

    # 3) trajectory (x-y) ---------------------------------------------------
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(px, py, color='tab:blue')
    ax.scatter([px[0]], [py[0]], color='green', zorder=3, label=L('起点', 'start'))
    ax.scatter([px[-1]], [py[-1]], color='red', zorder=3, label=L('终点', 'end'))
    ax.set_aspect('equal', adjustable='datalim')
    _style(ax, L('乌龟轨迹 (x - y)', 'Turtle trajectory (x - y)'),
           L('x / m', 'x / m'), L('y / m', 'y / m'))
    ax.legend()
    fig.tight_layout()
    p = os.path.join(out_dir, 'trajectory_xy.png')
    fig.savefig(p, dpi=130)
    plt.close(fig)
    saved['trajectory'] = p

    # 4) raw data csv -------------------------------------------------------
    csv_path = os.path.join(out_dir, 'recorded_data.csv')
    with open(csv_path, 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(['t_cmd_s', 'linear_x', 'angular_z'])
        for t, m in cmds:
            w.writerow(['%.6f' % t, '%.6f' % m.linear.x, '%.6f' % m.angular.z])
        w.writerow([])
        w.writerow(['t_pose_s', 'x', 'y', 'theta'])
        for t, m in poses:
            w.writerow(['%.6f' % t, '%.6f' % m.x, '%.6f' % m.y, '%.6f' % m.theta])
    saved['csv'] = csv_path

    # ---- console summary --------------------------------------------------
    print('=' * 62)
    print('bag summary')
    print('-' * 62)
    print('cmd_vel samples : %d   (duration %.2f s)' % (len(cmds), ct[-1] - ct[0]))
    print('pose    samples : %d   (duration %.2f s)' % (len(poses), pt[-1] - pt[0]))
    print('linear.x  : mean=%.3f  std=%.4f  (m/s)' % (lin.mean(), lin.std()))
    print('angular.z : mean=%.3f  std=%.4f  (rad/s)' % (ang.mean(), ang.std()))
    print('x range   : [%.2f, %.2f]' % (px.min(), px.max()))
    print('y range   : [%.2f, %.2f]' % (py.min(), py.max()))
    if abs(ang.mean()) > 1e-6:
        print('implied radius = v/omega = %.3f m' % (lin.mean() / ang.mean()))
        print('implied period = 2*pi/omega = %.3f s' % (2 * np.pi / abs(ang.mean())))
    print('=' * 62)
    return saved


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description='Plot velocity/position from a turtle_circle bag.')
    parser.add_argument('--bag', required=True, help='rosbag2 directory')
    parser.add_argument('--out', default='docs/plots', help='output directory for figures')
    args = parser.parse_args(argv)

    cmds, poses = read_bag(args.bag)
    if not cmds or not poses:
        print('ERROR: empty bag (cmds=%d poses=%d)' % (len(cmds), len(poses)), file=sys.stderr)
        sys.exit(1)
    saved = make_plots(cmds, poses, args.out)
    print('saved figures:')
    for k, v in saved.items():
        print('  %-11s %s' % (k, v))


if __name__ == '__main__':
    main()
