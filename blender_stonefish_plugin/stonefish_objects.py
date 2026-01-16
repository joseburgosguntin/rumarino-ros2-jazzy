"""stonefish_objects.py
In this module we have a Internal Representation of the stonefish objects that we want to interface.
"""

from dataclasses import dataclass, field
from typing import Tuple, Optional, List, Dict
from enum import Enum
from abc import ABCMeta, abstractmethod


class ComposableElement(metaclass=ABCMeta):
    """Base class for composable XML elements"""

    def __init__(self, children: Optional[List["ComposableElement"]] = None):
        self.children = children if children is not None else []

    @abstractmethod
    def generate_xml_string(self, indent: int = 0) -> str:
        """Generate XML string with proper indentation"""
        pass

    def _indent(self, level: int) -> str:
        """Helper to generate indentation"""
        return "\t" * level

    def _render_children(self, indent: int) -> str:
        """Render all children with proper indentation"""
        if not self.children:
            return ""
        return "\n".join(child.generate_xml_string(indent) for child in self.children)


@dataclass
class Material(ComposableElement):
    """
    Stonefish material definition.

    Attributes:
        name: Unique material identifier
        density: Density in kg/m³
        restitution: Coefficient of restitution (0.0 to 1.0)
        magnetic: Magnetic properties (<0 ferromagnetic, 0 nonmagnetic, >0 magnet)
    """

    name: str
    density: float
    restitution: float
    magnetic: float = 0.0

    def __post_init__(self):
        super().__init__()

    def generate_xml_string(self, indent: int = 0) -> str:
        i = self._indent(indent)
        return f'{i}<material name="{self.name}" density="{self.density:.1f}" restitution="{self.restitution:.1f}" />'


@dataclass
class FrictionPair(ComposableElement):
    """
    Friction coefficients between two materials.

    Attributes:
        material1: Name of first material
        material2: Name of second material
        static_friction: Static friction coefficient
        dynamic_friction: Dynamic friction coefficient
    """

    material1: str
    material2: str
    static_friction: float
    dynamic_friction: float

    def __post_init__(self):
        super().__init__()

    def generate_xml_string(self, indent: int = 0) -> str:
        """Generate XML for friction pair"""
        i = self._indent(indent)
        return (
            f'{i}<friction material1="{self.material1}" material2="{self.material2}" '
            f'static="{self.static_friction:.1f}" dynamic="{self.dynamic_friction:.1f}" />'
        )


@dataclass
class FrictionTable(ComposableElement):
    """
    Complete friction table for material interactions.

    Attributes:
        pairs: List of material pair friction definitions
    """

    pairs: List[FrictionPair] = field(default_factory=list)

    def __post_init__(self):
        children_list: List[ComposableElement] = list(self.pairs) if self.pairs else []
        super().__init__(children_list)

    def add_pair(self, mat1: str, mat2: str, static: float, dynamic: float):
        pair = FrictionPair(mat1, mat2, static, dynamic)
        self.pairs.append(pair)
        self.children.append(pair)

    def generate_xml_string(self, indent: int = 0) -> str:
        """Generate XML for friction table"""
        if not self.pairs:
            return ""
        i = self._indent(indent)
        lines = [f"{i}<friction_table>"]
        lines.append(self._render_children(indent + 1))
        lines.append(f"{i}</friction_table>")
        return "\n".join(lines)


@dataclass
class Look(ComposableElement):
    """
    Stonefish look (visual appearance) definition using PBR.

    Attributes:
        name: Unique look identifier
        color: RGB color values (0.0 to 1.0)
        roughness: Surface roughness (0.0 = smooth, 1.0 = rough)
        metalness: Metalness factor (0.0 = dielectric, 1.0 = metal)
        reflectivity: Reflectivity factor (0.0 to 1.0)
    """

    name: str
    color: Tuple[float, float, float]
    roughness: float = 0.5
    metalness: float = 0.0
    reflectivity: float = 0.5

    def __post_init__(self):
        super().__init__()

    def generate_xml_string(self, indent: int = 0) -> str:
        """Generate XML for look definition"""
        i = self._indent(indent)
        r, g, b = self.color
        return (
            f'{i}<look name="{self.name}" rgb="{r:.1f} {g:.1f} {b:.1f}" '
            f'roughness="{self.roughness:.1f}" metalness="{self.metalness:.1f}" />'
        )


@dataclass
class WorldTransform:
    """
    Transformation in world coordinates.

    Attributes:
        xyz: Position (x, y, z) in meters
        rpy: Orientation (roll, pitch, yaw) in radians
    """

    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]

    def xyz_string(self) -> str:
        """Format position as space-separated string"""
        return "{:.5f} {:.5f} {:.5f}".format(*self.xyz)

    def rpy_string(self) -> str:
        """Format orientation as space-separated string"""
        return "{:.5f} {:.5f} {:.5f}".format(*self.rpy)


class PhysicsType(Enum):
    """Physics simulation type"""

    SURFACE = "surface"
    FLOATING = "floating"
    SUBMERGED = "submerged"


@dataclass
class PhysicsProperties:
    """
    Physics properties for objects.

    Attributes:
        physics_type: Type of physics simulation
        buoyant: Enable buoyancy calculation
    """

    physics_type: PhysicsType = PhysicsType.SURFACE
    buoyant: bool = False


@dataclass
class StaticObject(ComposableElement):
    """Base class for all static objects"""

    name: str
    world_transform: WorldTransform
    material: str
    look: str
    physics: Optional[PhysicsProperties] = None

    def __post_init__(self):
        super().__init__()

    def _generate_world_transform_xml(self, indent: int) -> str:
        """Helper to generate world_transform XML"""
        i = self._indent(indent)
        return f'{i}<world_transform rpy="{self.world_transform.rpy_string()}" xyz="{self.world_transform.xyz_string()}" />'

    def _generate_material_xml(self, indent: int) -> str:
        """Helper to generate material XML"""
        i = self._indent(indent)
        return f'{i}<material name="{self.material}" />'

    def _generate_look_xml(self, indent: int) -> str:
        """Helper to generate look XML"""
        i = self._indent(indent)
        return f'{i}<look name="{self.look}" />'


@dataclass
class Plane(StaticObject):
    """
    Infinite plane (typically used for ground).

    Attributes:
        name: Object name
        world_transform: World transform
        material: Material name
        look: Look name
    """

    def generate_xml_string(self, indent: int = 0) -> str:
        """Generate XML for plane"""
        i = self._indent(indent)
        lines = [
            f'{i}<static name="{self.name}" type="plane">',
            self._generate_material_xml(indent + 1),
            self._generate_look_xml(indent + 1),
            self._generate_world_transform_xml(indent + 1),
            f"{i}</static>",
        ]
        return "\n".join(lines)


@dataclass
class Box(StaticObject):
    """
    Box primitive (rectangular prism).

    Attributes:
        name: Object name
        world_transform: World transform
        material: Material name
        look: Look name
        dimensions: Box dimensions (x, y, z) in meters
        thickness: Wall thickness for hollow box (None = solid)
    """

    dimensions: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    thickness: Optional[float] = None

    def generate_xml_string(self, indent: int = 0) -> str:
        """Generate XML for box"""
        i = self._indent(indent)
        dims = " ".join([f"{d:.4f}" for d in self.dimensions])

        lines = [f'{i}<static name="{self.name}" type="box">']

        if self.thickness:
            lines.append(
                f'{self._indent(indent + 1)}<dimensions xyz="{dims}" thickness="{self.thickness:.4f}" />'
            )
        else:
            lines.append(f'{self._indent(indent + 1)}<dimensions xyz="{dims}" />')

        lines.extend(
            [
                self._generate_world_transform_xml(indent + 1),
                self._generate_material_xml(indent + 1),
                self._generate_look_xml(indent + 1),
                f"{i}</static>",
            ]
        )
        return "\n".join(lines)


@dataclass
class Cylinder(StaticObject):
    """
    Cylinder primitive.

    Attributes:
        name: Object name
        world_transform: World transform
        material: Material name
        look: Look name
        radius: Cylinder radius in meters
        height: Cylinder height in meters
    """

    radius: float = 0.5
    height: float = 1.0

    def generate_xml_string(self, indent: int = 0) -> str:
        """Generate XML for cylinder"""
        i = self._indent(indent)
        lines = [
            f'{i}<static name="{self.name}" type="cylinder">',
            f'{self._indent(indent + 1)}<dimensions radius="{self.radius:.4f}" height="{self.height:.4f}" />',
            self._generate_world_transform_xml(indent + 1),
            self._generate_material_xml(indent + 1),
            self._generate_look_xml(indent + 1),
            f"{i}</static>",
        ]
        return "\n".join(lines)


@dataclass
class Sphere(StaticObject):
    """
    Sphere primitive.

    Attributes:
        name: Object name
        world_transform: World transform
        material: Material name
        look: Look name
        radius: Sphere radius in meters
    """

    radius: float = 0.5

    def generate_xml_string(self, indent: int = 0) -> str:
        """Generate XML for sphere"""
        i = self._indent(indent)
        lines = [
            f'{i}<static name="{self.name}" type="sphere">',
            f'{self._indent(indent + 1)}<dimensions radius="{self.radius:.4f}" />',
            self._generate_world_transform_xml(indent + 1),
            self._generate_material_xml(indent + 1),
            self._generate_look_xml(indent + 1),
            f"{i}</static>",
        ]
        return "\n".join(lines)


@dataclass
class Torus(StaticObject):
    """
    Torus primitive.

    Attributes:
        name: Object name
        world_transform: World transform
        material: Material name
        look: Look name
        major_radius: Major radius (from center to tube center) in meters
        minor_radius: Minor radius (tube radius) in meters
    """

    major_radius: float = 1.0
    minor_radius: float = 0.2

    def generate_xml_string(self, indent: int = 0) -> str:
        """Generate XML for torus"""
        i = self._indent(indent)
        lines = [
            f'{i}<static name="{self.name}" type="torus">',
            f'{self._indent(indent + 1)}<dimensions major_radius="{self.major_radius:.4f}" minor_radius="{self.minor_radius:.4f}" />',
            self._generate_world_transform_xml(indent + 1),
            self._generate_material_xml(indent + 1),
            self._generate_look_xml(indent + 1),
            f"{i}</static>",
        ]
        return "\n".join(lines)


@dataclass
class Model(StaticObject):
    """
    Custom mesh model from external file.

    Attributes:
        name: Object name
        world_transform: World transform
        material: Material name
        look: Look name
        mesh_filename: Path to mesh file (OBJ, STL, etc.)
        mesh_scale: Scale factor for mesh
        origin_rpy: Mesh origin rotation (roll, pitch, yaw)
        origin_xyz: Mesh origin translation (x, y, z)
    """

    mesh_filename: str = ""
    mesh_scale: float = 1.0
    origin_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    origin_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def generate_xml_string(self, indent: int = 0) -> str:
        """Generate XML for mesh model"""
        i = self._indent(indent)
        origin_rpy_str = " ".join([f"{r:.4f}" for r in self.origin_rpy])
        origin_xyz_str = " ".join([f"{x:.4f}" for x in self.origin_xyz])

        lines = [
            f'{i}<static name="{self.name}" type="model">',
            f"{self._indent(indent + 1)}<physical>",
            f'{self._indent(indent + 2)}<mesh filename="{self.mesh_filename}" scale="{self.mesh_scale:.4f}" />',
            f'{self._indent(indent + 2)}<origin rpy="{origin_rpy_str}" xyz="{origin_xyz_str}" />',
            f"{self._indent(indent + 1)}</physical>",
            self._generate_world_transform_xml(indent + 1),
            self._generate_material_xml(indent + 1),
            self._generate_look_xml(indent + 1),
            f"{i}</static>",
        ]
        return "\n".join(lines)


# ============================================================================
# Compound Objects (Multi-part rigid bodies)
# ============================================================================


@dataclass
class ExternalPart:
    """
    A part within a compound object.

    Attributes:
        name: Part name
        geometry_type: Type of geometry (cylinder, box, sphere, model)
        material: Material name
        look: Look name
        compound_transform: Transform relative to compound body origin
        origin_rpy: Geometry origin rotation (for mesh alignment)
        origin_xyz: Geometry origin translation (for mesh alignment)
        physics: Physics properties

        # Geometry-specific attributes (set based on geometry_type)
        radius: For cylinder/sphere
        height: For cylinder
        dimensions: For box (x, y, z)
        thickness: For hollow structures
        mesh_filename: For model type
        mesh_scale: For model type
    """

    name: str
    geometry_type: str  # "cylinder", "box", "sphere", "model"
    material: str
    look: str
    compound_transform: WorldTransform
    origin_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    origin_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    physics: Optional[PhysicsProperties] = None

    # Geometry parameters (use based on geometry_type)
    radius: Optional[float] = None
    height: Optional[float] = None
    dimensions: Optional[Tuple[float, float, float]] = None
    thickness: Optional[float] = None
    mesh_filename: Optional[str] = None
    mesh_scale: Optional[float] = None


@dataclass
@dataclass
class CompoundObject(ComposableElement):
    """
    Compound object made from multiple external parts.
    Used for complex rigid bodies like robots, vehicles, etc.

    Attributes:
        name: Compound object name
        world_transform: World transform
        external_parts: List of parts that make up the compound
        physics: Physics properties for the entire compound
        self_collisions: Enable collision between parts
    """

    name: str
    world_transform: WorldTransform
    external_parts: List[ExternalPart]
    physics: Optional[PhysicsProperties] = None
    self_collisions: bool = False

    def __post_init__(self):
        super().__init__()

    def generate_xml_string(self, indent: int = 0) -> str:
        """Generate XML for compound object"""
        i = self._indent(indent)
        lines = [f'{i}<compound name="{self.name}">']

        # World transform
        rpy_str = " ".join([f"{r:.4f}" for r in self.world_transform.rpy])
        xyz_str = " ".join([f"{x:.4f}" for x in self.world_transform.xyz])
        lines.append(
            f'{self._indent(indent + 1)}<world_transform rpy="{rpy_str}" xyz="{xyz_str}" />'
        )

        # External parts
        for part in self.external_parts:
            lines.append(
                f'{self._indent(indent + 1)}<external_part name="{part.name}" type="{part.geometry_type}" material="{part.material}" look="{part.look}">'
            )
            # Add part-specific XML here based on geometry_type
            # This is a simplified version - you may want to expand based on your needs
            lines.append(f"{self._indent(indent + 1)}</external_part>")

        lines.append(f"{i}</compound>")
        return "\n".join(lines)


# ============================================================================
# Original Compound Objects (Made from multiple primitives)
# ============================================================================@dataclass
class Gate:
    """
    Gate structure composed of two vertical posts and a horizontal bar.

    Attributes:
        name: Gate name prefix
        world_transform: Base transform (center of gate at ground level)
        material: Material for all components
        look: Look for all components
        width: Distance between posts (meters)
        height: Height of vertical posts (meters)
        cylinder_radius: Radius of cylinders (post/bar thickness)
    """

    name: str
    world_transform: WorldTransform
    material: str
    look: str
    width: float = 2.0
    height: float = 1.0
    cylinder_radius: float = 0.04

    def to_cylinders(self) -> List[Cylinder]:
        """Convert gate to list of cylinder primitives"""
        bx, by, bz = self.world_transform.xyz
        rx, ry, rz = self.world_transform.rpy

        # Left and right posts
        half_w = self.width / 2.0
        post_z = bz + self.height / 2.0

        left_post = Cylinder(
            name=f"{self.name}_Left",
            world_transform=WorldTransform(
                xyz=(bx - half_w, by, post_z), rpy=(rx, ry, rz)
            ),
            material=self.material,
            look=self.look,
            radius=self.cylinder_radius,
            height=self.height,
        )

        right_post = Cylinder(
            name=f"{self.name}_Right",
            world_transform=WorldTransform(
                xyz=(bx + half_w, by, post_z), rpy=(rx, ry, rz)
            ),
            material=self.material,
            look=self.look,
            radius=self.cylinder_radius,
            height=self.height,
        )

        # Top bar (rotated to horizontal)
        top_length = self.width + 2.0 * self.cylinder_radius
        top_bar = Cylinder(
            name=f"{self.name}_Top",
            world_transform=WorldTransform(
                xyz=(bx, by, bz + self.height),
                rpy=(1.5708, ry, rz),  # 90 degree rotation
            ),
            material=self.material,
            look=self.look,
            radius=self.cylinder_radius,
            height=top_length,
        )

        return [left_post, right_post, top_bar]


@dataclass
class Marker:
    """
    Marker buoy (cylinder standing on ground).

    Attributes:
        name: Marker name
        world_transform: Base transform (bottom center)
        material: Material name
        look: Look name
        diameter: Marker diameter in meters
        height: Marker height in meters
    """

    name: str
    world_transform: WorldTransform
    material: str
    look: str
    diameter: float = 0.3
    height: float = 1.5

    def to_cylinder(self) -> Cylinder:
        """Convert marker to cylinder primitive"""
        # Adjust Z to position at center of height
        x, y, z = self.world_transform.xyz
        adjusted_z = z + self.height / 2.0

        return Cylinder(
            name=self.name,
            world_transform=WorldTransform(
                xyz=(x, y, adjusted_z), rpy=self.world_transform.rpy
            ),
            material=self.material,
            look=self.look,
            radius=self.diameter / 2.0,
            height=self.height,
        )


# ============================================================================
# Environment
# ============================================================================


@dataclass
class Ocean(ComposableElement):
    """
    Ocean environment configuration.

    Attributes:
        waves: Wave height in meters (0.0 = calm)
        particles: Enable particle system for visualization
        current: Current velocity (x, y, z) in m/s
    """

    waves: float = 0.0
    particles: bool = True
    current: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self):
        super().__init__()

    def generate_xml_string(self, indent: int = 0) -> str:
        """Generate XML for ocean"""
        i = self._indent(indent)
        current_str = " ".join([f"{c:.4f}" for c in self.current])
        particles_str = "true" if self.particles else "false"

        return f'{i}<ocean waves="{self.waves:.2f}" particles="{particles_str}" current="{current_str}" />'


@dataclass
class Atmosphere(ComposableElement):
    """
    Atmospheric conditions.

    Attributes:
        pressure: Atmospheric pressure in Pa
        temperature: Temperature in Celsius
    """

    pressure: float = 101325.0  # Standard atmosphere
    temperature: float = 20.0

    def __post_init__(self):
        super().__init__()

    def generate_xml_string(self, indent: int = 0) -> str:
        """Generate XML for atmosphere"""
        i = self._indent(indent)
        return f'{i}<atmosphere pressure="{self.pressure:.1f}" temperature="{self.temperature:.1f}" />'


@dataclass
class Sun(ComposableElement):
    """
    Sun lighting configuration.

    Attributes:
        azimuth: Sun azimuth angle in degrees
        elevation: Sun elevation angle in degrees
    """

    azimuth: float = 0.0
    elevation: float = 60.0

    def __post_init__(self):
        super().__init__()

    def generate_xml_string(self, indent: int = 0) -> str:
        """Generate XML for sun"""
        i = self._indent(indent)
        return (
            f'{i}<sun azimuth="{self.azimuth:.1f}" elevation="{self.elevation:.1f}" />'
        )


@dataclass
class Environment(ComposableElement):
    """
    Complete environment configuration.

    Attributes:
        ocean: Ocean configuration (None = no ocean)
        atmosphere: Atmospheric configuration
        sun: Sun lighting configuration
    """

    ocean: Optional[Ocean] = None
    atmosphere: Atmosphere = field(default_factory=Atmosphere)
    sun: Sun = field(default_factory=Sun)

    def __post_init__(self):
        children: List[ComposableElement] = []
        if self.ocean:
            children.append(self.ocean)
        children.append(self.atmosphere)
        children.append(self.sun)
        super().__init__(children)

    def generate_xml_string(self, indent: int = 0) -> str:
        """Generate XML for environment"""
        i = self._indent(indent)
        lines = [f"{i}<environment>"]

        if self.ocean:
            lines.append(self.ocean.generate_xml_string(indent + 1))
        lines.append(self.atmosphere.generate_xml_string(indent + 1))
        lines.append(self.sun.generate_xml_string(indent + 1))

        lines.append(f"{i}</environment>")
        return "\n".join(lines)


# ============================================================================
# Complete Scenario
# ============================================================================


@dataclass
class StonefishScenario(ComposableElement):
    """
    Complete Stonefish scenario definition.

    Attributes:
        name: Scenario name
        materials: List of material definitions
        friction_table: Friction table for material interactions
        looks: List of look definitions
        static_objects: List of static objects in scene
        compound_objects: List of compound objects (multi-part rigid bodies)
        environment: Environment configuration
    """

    name: str = "Scenario"
    materials: List[Material] = field(default_factory=list)
    friction_table: FrictionTable = field(default_factory=FrictionTable)
    looks: List[Look] = field(default_factory=list)
    static_objects: List[StaticObject] = field(default_factory=list)
    compound_objects: List[CompoundObject] = field(default_factory=list)
    environment: Environment = field(default_factory=Environment)

    def __post_init__(self):
        # Build children list for composable structure
        children: List[ComposableElement] = []
        children.extend(self.materials)
        children.extend(self.looks)
        children.append(self.friction_table)
        children.append(self.environment)
        children.extend(self.static_objects)
        children.extend(self.compound_objects)
        super().__init__(children)

    def generate_xml_string(self, indent: int = 0) -> str:
        """Generate complete Stonefish XML scenario"""
        i = self._indent(indent)
        lines = [
            '<?xml version="1.0"?>',
            f"{i}<scenario>",
        ]

        # Materials
        for material in self.materials:
            lines.append(material.generate_xml_string(indent + 1))

        # Looks
        for look in self.looks:
            lines.append(look.generate_xml_string(indent + 1))

        # Friction table
        lines.append(self.friction_table.generate_xml_string(indent + 1))

        # Environment
        lines.append(self.environment.generate_xml_string(indent + 1))

        # Static objects
        for obj in self.static_objects:
            lines.append(obj.generate_xml_string(indent + 1))

        # Compound objects
        for obj in self.compound_objects:
            lines.append(obj.generate_xml_string(indent + 1))

        lines.append(f"{i}</scenario>")
        return "\n".join(lines)

    def add_material(self, material: Material):
        """Add a material to the scenario"""
        if material.name not in [m.name for m in self.materials]:
            self.materials.append(material)
            self.children.append(material)

    def add_look(self, look: Look):
        """Add a look to the scenario"""
        if look.name not in [l.name for l in self.looks]:
            self.looks.append(look)
            self.children.append(look)

    def add_object(self, obj: StaticObject):
        """Add a static object to the scenario"""
        self.static_objects.append(obj)
        self.children.append(obj)

    def add_compound(self, compound: CompoundObject):
        """Add a compound object to the scenario"""
        self.compound_objects.append(compound)
