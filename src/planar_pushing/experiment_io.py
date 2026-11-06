from datetime import datetime
from pathlib import Path
from uuid import uuid4
import json
import numpy as np


def new_run(base, label):
    path = Path(base) / f'{label}_{datetime.now().astimezone():%Y%m%dT%H%M%S%z}_{uuid4().hex[:6]}'
    path.mkdir(parents=True, exist_ok=False)
    return path


def save_json(path, value):
    def encode(obj):
        if isinstance(obj, np.ndarray): return obj.tolist()
        if isinstance(obj, np.generic): return obj.item()
        raise TypeError(type(obj).__name__)
    Path(path).write_text(json.dumps(value, indent=2, default=encode, allow_nan=False)+'\n')


def pyplot():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    return plt
