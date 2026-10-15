"""Read the editable C++ limits for offline integration fixtures."""

from pathlib import Path
import re


header = (Path(__file__).resolve().parents[1] /
          'src/rh5dg2_driver/include/rh5dg2_driver/registers.hpp').read_text()
header = re.sub(r'//[^\n]*|/\*.*?\*/', '', header, flags=re.S)
ANGLE_MIN, ANGLE_MAX = (
    [int(value) for value in re.findall(r'-?\d+',
        re.search(r'\b' + name + r'\s*\{([^}]+)\}', header).group(1))]
    for name in ('kAngleMin', 'kAngleMax'))
assert len(ANGLE_MIN) == len(ANGLE_MAX) == 13
