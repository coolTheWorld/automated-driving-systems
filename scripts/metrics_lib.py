#!/usr/bin/env python3
# Copyright 2026 孙帅
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Append per-criterion rows to the metrics history CSV (SPEC S8 L4).

SPEC §8 L4 要求「记录关键指标随时间的变化曲线」—— 本模块是那条曲线的
数据落点：**一判据一行、追加制、永不改写历史**。

    from metrics_lib import MetricsWriter
    metrics = MetricsWriter(args.metrics_out, scenario='S03', layer='truth')
    metrics.add('ttc_min_s', 2.81, 2.0, 'PASS')   # 判据循环里顺带一行
    metrics.flush()

四个 record_*_run 脚本共用（放公共模块的判据：有第二个消费者且有逻辑 ——
这里的逻辑是 schema/表头管理与 git sha 提取，四份各写一遍必漂移）。

⚠️ **本模块不做判定**。判定留在各记录器的判据表里（判据来自 plan.md，
   脚本不重新发明）——这里只如实落盘别人判完的结果。
"""

from __future__ import annotations

import csv
from datetime import datetime, timezone
from pathlib import Path
import subprocess

FIELDS = ['utc', 'git_sha', 'scenario', 'layer', 'criterion', 'value', 'limit', 'passed']


def _git_sha() -> str:
    """
    Return the short git sha of the working tree, or 'unknown'.

    :return: 短 sha
    """
    try:
        return subprocess.run(
            ['git', 'rev-parse', '--short', 'HEAD'],
            capture_output=True, text=True, timeout=10,
            cwd=Path(__file__).resolve().parent).stdout.strip() or 'unknown'
    except (OSError, subprocess.SubprocessError):
        return 'unknown'


class MetricsWriter:
    """Buffer criterion rows and append them to the history CSV on flush."""

    def __init__(self, out_path, scenario: str, layer: str = '-'):
        """
        Prepare a writer; out_path 为空/None 时本对象是空操作.

        :param out_path: history CSV 路径（追加；不存在则建并写表头）
        :param scenario: 场景标签（S01…S07 / junction / regression_p6 …）
        :param layer: truth / perception / '-'（无分层的场景）
        """
        self.out_path = Path(out_path) if out_path else None
        self.scenario = scenario
        self.layer = layer
        self.rows: list[dict] = []
        self._stamp = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
        self._sha = _git_sha()

    def add(self, criterion: str, value, limit, passed) -> None:
        """
        Record one criterion outcome.

        :param criterion: 判据名（与打印表里的一致，别另起名字）
        :param value: 实测值（数值或字符串）
        :param limit: 判据（照打印表的文本，如 '> 2 s' / 2.0）
        :param passed: bool 或 'PASS'/'FAIL' 字符串
        """
        if self.out_path is None:
            return
        if isinstance(passed, bool):
            passed = 'PASS' if passed else 'FAIL'
        self.rows.append({
            'utc': self._stamp, 'git_sha': self._sha, 'scenario': self.scenario,
            'layer': self.layer, 'criterion': criterion, 'value': value,
            'limit': limit, 'passed': passed,
        })

    def flush(self) -> None:
        """Append buffered rows; create the file with a header when absent."""
        if self.out_path is None or not self.rows:
            return
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        fresh = not self.out_path.exists()
        with open(self.out_path, 'a', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=FIELDS)
            if fresh:
                writer.writeheader()
            writer.writerows(self.rows)
        self.rows.clear()
