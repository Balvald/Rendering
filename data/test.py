"""aggregation script for fouranimals dataset."""

import os
import matplotlib.pyplot as plt
import re


regex = re.compile(r"([\d.e\+-]*) seconds.")

