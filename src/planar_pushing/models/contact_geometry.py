"""Object-fixed face geometry; t=J@n, r=face_center+s*t (SI)."""
from enum import Enum
import numpy as np


class ContactFace(str, Enum):
    LEFT = 'left'
    RIGHT = 'right'
    TOP = 'top'
    BOTTOM = 'bottom'


def rotation(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def geometry(face, s, length, width):
    face = ContactFace(face)
    if not np.isfinite([s, length, width]).all() or min(length, width) <= 0:
        raise ValueError('geometry requires finite s and positive dimensions')
    normal, center, half = {
        ContactFace.LEFT: ([1., 0.], [-length/2, 0.], width/2),
        ContactFace.RIGHT: ([-1., 0.], [length/2, 0.], width/2),
        ContactFace.TOP: ([0., -1.], [0., width/2], length/2),
        ContactFace.BOTTOM: ([0., 1.], [0., -width/2], length/2),
    }[face]
    if abs(s) >= half:
        raise ValueError('s must be strictly inside the face; corners excluded')
    n = np.array(normal)
    t = np.array([-n[1], n[0]])
    r = np.array(center) + s*t
    A = np.array([[1., 0., -r[1]], [0., 1., r[0]]])
    return r, n, t, np.column_stack((n, t)), A


def physical_half_length(face, length, width):
    return width/2 if ContactFace(face) in (ContactFace.LEFT, ContactFace.RIGHT) else length/2
