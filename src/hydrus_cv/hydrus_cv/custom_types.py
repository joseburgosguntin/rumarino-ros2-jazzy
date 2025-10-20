from dataclasses import dataclass, field
import numpy as np

CameraIntrinsics = tuple[float, float, float, float]
DepthImage = np.ndarray
@dataclass
class Point3D:
    x: float
    y: float
    z: float


@dataclass
class Rotation3D:
    x: float
    y: float
    z: float
    w: float

@dataclass
class BoundingBox3D:
    rotation: Rotation3D
    width: float
    height: float
    length: float

@dataclass
class Detection:
    _x1 : int
    _y1 : int
    _x2 : int
    _y2 : int
    _cls: int
    _conf : float
    _distance: float | None = None
    _point: Point3D | None = None
    _bbox_3d: BoundingBox3D | None = None

@dataclass
class MapObject:
    track_id: int  
    cls: int
    conf: float
    point: Point3D
    bbox_3d: BoundingBox3D



@dataclass
class MapState:
    obj_frequencies: dict[str, int] = field(default_factory=dict)
    objects: list[MapObject] = field(default_factory=list)
    id_counter: int = 0
    map_id_objects: dict[int, MapObject] = field(default_factory=dict)
    point_clouds: list[np.ndarray] = field(default_factory=list)
