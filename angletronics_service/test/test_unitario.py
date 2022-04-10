import sys
sys.path.insert(1, '../angletronics_service')
from movement_server import Service as f

import math


def test_calc_velocidad_angular():
    assert f.calcular_velocidad_angular(1.0, 1.0) == 1.0
    assert f.calcular_velocidad_angular(2.0, 1.0) == 2.0