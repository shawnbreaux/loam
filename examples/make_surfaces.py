"""Generate the example design (FG) and existing-ground (EG) LandXML surfaces
used by pad_north.loam. Coordinates are site-local feet.

    python examples/make_surfaces.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from loam.landxml import write_grid_tin

here = os.path.dirname(os.path.abspath(__file__))
xs = np.arange(-20, 181, 10.0)  # 10 ft TIN spacing, a 20 ft apron around the pad
ys = np.arange(-20, 121, 10.0)
X, Y = np.meshgrid(xs, ys)

# Finished pad: 100.00 ft at the north edge, 1% fall to the south for drainage.
design = 100.0 - 0.01 * (100 - Y)
write_grid_tin(os.path.join(here, "pad_north_design.xml"), "FG", xs, ys, design)

# Existing ground: falls about 5 ft west to east, with a knob and a swale.
existing = (101.9 - 4.6 * (X / 160)
            + 0.8 * np.exp(-(((X - 40) / 18) ** 2 + ((Y - 60) / 20) ** 2))
            - 2.0 * np.exp(-(((X - 128) / 26) ** 2 + ((Y - 40) / 22) ** 2)))
write_grid_tin(os.path.join(here, "pad_north_existing.xml"), "EG", xs, ys, existing)
print("wrote pad_north_design.xml and pad_north_existing.xml")
