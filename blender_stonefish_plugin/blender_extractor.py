"""
Blender Object Extractor

This module reads Blender .blend files and extracts static objects
for conversion to Stonefish scenarios.

It focuses on:
- Object geometry (type, dimensions, transforms)
- Custom properties (material, look)
- Object naming conventions
"""

import sys
import struct
from pathlib import Path
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass

try:
    import blendfile
except ImportError:
    print("Error: blendfile module not found. Make sure blendfile.py is in the same directory.")
    sys.exit(1)


# IDProperty type constants from Blender
IDP_STRING = 0
IDP_INT = 1
IDP_FLOAT = 2
IDP_ARRAY = 5
IDP_GROUP = 6
IDP_ID = 7
IDP_DOUBLE = 8


@dataclass
class BlenderObject:
    """
    Represents a Blender object with relevant properties for Stonefish conversion.

    Attributes:
        name: Object name
        object_type: Blender object type (MESH, EMPTY, etc.)
        geometry_type: Derived geometry type (PLANE, CYLINDER, BOX, SPHERE, MESH)
        location: World location (x, y, z)
        rotation: World rotation in Euler angles (x, y, z) in radians
        scale: Object scale (x, y, z)
        dimensions: Object dimensions (x, y, z) in Blender units

        # Custom properties from Blender
        material: Material name (from custom property or default)
        look: Look name (from custom property or default)

        # For mesh objects
        mesh_name: Name of mesh data (for custom meshes)
    """
    name: str
    object_type: str
    geometry_type: str
    location: Tuple[float, float, float]
    rotation: Tuple[float, float, float]
    scale: Tuple[float, float, float]
    dimensions: Tuple[float, float, float]

    # Custom properties (StaticEntity properties)
    material: str
    look: str

    # Optional mesh data
    mesh_name: Optional[str] = None


class IDPropertyReader:
    """
    Reads Blender custom properties (IDProperties) from .blend files.

    blendfile.py doesn't support reading IDPropertyData unions, so we manually
    parse the binary data based on the type field to extract property values.
    """

    def __init__(self, blend_file):
        self.blend = blend_file
        self.pointer_size = 8 if blend_file.header.pointer_size == 8 else 4

    def read_properties(self, obj_block) -> Dict[str, str]:
        """
        Read all custom properties from an object.

        Args:
            obj_block: Object block from blendfile

        Returns:
            Dict of property name -> value (as strings)
        """
        try:
            # Get IDProperty pointer from object
            props_addr = obj_block.get((b'id', b'properties'))
            if not props_addr or props_addr == 0:
                return {}

            # Find the IDProperty block
            props_block = self._find_block(props_addr)
            if not props_block:
                return {}

            # Check if it's a group (IDP_GROUP = 6)
            prop_type = props_block.get(b'type')
            if prop_type != IDP_GROUP:
                return {}

            # Read properties from the group
            return self._read_group(props_block)

        except Exception as e:
            print(f"      Warning: Error reading custom properties: {e}")
            return {}

    def _find_block(self, address):
        """Find block at given memory address"""
        for block in self.blend.blocks:
            if block.addr_old == address:
                return block
        return None

    def _read_group(self, group_block) -> Dict[str, str]:
        """
        Read properties from IDP_GROUP.

        IDProperty structure (DNA):
        - Offset 88: data (IDPropertyData union, 32 bytes)
          - Offset 0: pointer (8 bytes)
          - Offset 8: group (ListBase: first, last pointers, 16 bytes)  <-- We need this
          - Offset 24: val (int, 4 bytes)
          - Offset 28: val2 (int, 4 bytes)

        For IDP_GROUP, read ListBase at offset 88+8=96
        """
        properties = {}

        try:
            # Read ListBase (linked list) from data.group at offset 96
            self.blend.handle.seek(group_block.file_offset + 96)

            # Read first and last pointers
            if self.pointer_size == 8:
                first_ptr = struct.unpack('<Q', self.blend.handle.read(8))[0]
            else:
                first_ptr = struct.unpack('<I', self.blend.handle.read(4))[0]

            # Traverse linked list
            current_ptr = first_ptr
            safety_limit = 100  # Prevent infinite loops

            while current_ptr and current_ptr != 0 and safety_limit > 0:
                prop_block = self._find_block(current_ptr)
                if not prop_block:
                    break

                # Read property name and value
                name, value = self._read_property(prop_block)
                if name:
                    properties[name] = value

                # Get next pointer (at offset 0)
                self.blend.handle.seek(prop_block.file_offset)
                if self.pointer_size == 8:
                    current_ptr = struct.unpack('<Q', self.blend.handle.read(8))[0]
                else:
                    current_ptr = struct.unpack('<I', self.blend.handle.read(4))[0]

                safety_limit -= 1

        except Exception as e:
            print(f"      Warning: Error reading group: {e}")

        return properties

    def _read_property(self, prop_block) -> Tuple[Optional[str], Optional[str]]:
        """Read single property name and value"""
        try:
            # Get property type and name
            prop_type = prop_block.get(b'type')
            name_bytes = prop_block.get(b'name')
            name = name_bytes.decode('utf-8').rstrip('\x00') if isinstance(name_bytes, bytes) else str(name_bytes).rstrip('\x00')

            # Read value from data union at offset 88
            self.blend.handle.seek(prop_block.file_offset + 88)

            if prop_type == IDP_STRING:  # String: data.pointer points to string
                if self.pointer_size == 8:
                    string_ptr = struct.unpack('<Q', self.blend.handle.read(8))[0]
                else:
                    string_ptr = struct.unpack('<I', self.blend.handle.read(4))[0]

                if string_ptr and string_ptr != 0:
                    string_block = self._find_block(string_ptr)
                    if string_block:
                        prop_len = prop_block.get(b'len')
                        self.blend.handle.seek(string_block.file_offset)
                        value = self.blend.handle.read(prop_len).decode('utf-8').rstrip('\x00')
                        return (name, value)

            elif prop_type == IDP_INT:  # Int: data.val (4 bytes)
                value = struct.unpack('<i', self.blend.handle.read(4))[0]
                return (name, str(value))

            elif prop_type == IDP_FLOAT:  # Float: data.val (4 bytes)
                value = struct.unpack('<f', self.blend.handle.read(4))[0]
                return (name, str(value))

            elif prop_type == IDP_DOUBLE:  # Double: 8 bytes
                value = struct.unpack('<d', self.blend.handle.read(8))[0]
                return (name, str(value))

        except Exception as e:
            print(f"      Warning: Error reading property: {e}")

        return (None, None)


class BlenderExtractor:
    """
    Extracts objects from Blender files for Stonefish conversion.
    """

    def __init__(self, blend_file: str, defaults: Dict = None):
        """
        Initialize the extractor.

        Args:
            blend_file: Path to .blend file
            defaults: Default values for material and look
        """
        self.blend_file = blend_file
        self.defaults = defaults or {'material': 'steel', 'look': 'gray'}
        self.blend = None
        self.objects = []
        self.id_reader = None

    def extract(self) -> List[BlenderObject]:
        """
        Extract all static objects from the Blender file.

        Returns:
            List of BlenderObject instances
        """
        try:
            self.blend = blendfile.open_blend(self.blend_file)
            self.id_reader = IDPropertyReader(self.blend)
        except FileNotFoundError as e:
            print(f"Error: Blend file not found: {self.blend_file}")
            print(f"  {e}")
            return []
        except AssertionError as e:
            print(f"Error: Blender file format not supported!")
            print(f"  File: {self.blend_file}")
            print(f"  The blendfile.py library doesn't support Blender 5.0+ format yet.")
            print(f"  Please save your .blend file in an older format:")
            print(f"    1. In Blender: File → Save As")
            print(f"    2. Look for format dropdown (near filename or bottom)")
            print(f"    3. Select 'Blender 2.93' or 'Blender 3.6'")
            print(f"    4. Save and try again")
            print(f"\n  Or export your scene and re-import in Blender 3.6 to convert.")
            return []
        except Exception as e:
            print(f"Error opening blend file: {self.blend_file}")
            print(f"  Exception type: {type(e).__name__}")
            print(f"  Exception message: {e}")
            import traceback
            traceback.print_exc()
            return []

        # Get all object blocks directly
        object_blocks = self.blend.find_blocks_from_code(b'OB')


        # Extract each object
        for obj_block in object_blocks:
            blender_obj = self._extract_object(obj_block)
            if blender_obj:
                self.objects.append(blender_obj)

        if len(self.objects) == 0:
            print("\n⚠ No STATIC objects found!")
            print("  Make sure your objects are named with 'STATIC' prefix")
            print("  Example: STATIC_Ground, STATIC_Wall1, etc.")

        print(f"✓ Extracted {len(self.objects)} STATIC objects from Blender file")
        return self.objects

    def _extract_scene_objects(self, scene_block) -> List[BlenderObject]:
        """Extract objects from a scene block"""
        scene_objects = []

        # Get scene data from block
        try:
            scene = scene_block.get_pointer((b'Scene', b'id'))
        except:
            scene = scene_block

        # Get scene name for logging
        scene_name = self._get_string_property(scene, 'id.name')
        if scene_name:
            scene_name = scene_name[2:]  # Remove "SC" prefix
            print(f"  Processing scene: {scene_name}")

        # Iterate through objects in scene
        try:
            # Try to get objects collection from scene
            objects_collection = scene.get('object')
            if objects_collection is None:
                print("    No objects found in scene")
                return scene_objects

            for obj_ref in objects_collection:
                # Dereference the pointer
                obj = obj_ref.dereference()
                if obj is None:
                    continue

                # Extract object data
                blender_obj = self._extract_object(obj)
                if blender_obj:
                    scene_objects.append(blender_obj)

        except Exception as e:
            print(f"    Error extracting objects: {e}")

        return scene_objects

    def _extract_object(self, block) -> Optional[BlenderObject]:
        """
        Extract data from a single Blender object block.

        Only processes objects with names starting with "STATIC".
        Reads geometry type, material, and look from custom properties.

        Args:
            block: Blender object block from blendfile

        Returns:
            BlenderObject or None if object should be skipped
        """
        try:
            # Get object name - using the same pattern as simple_get_objects.py
            name_data = block.get((b'id', b'name'))
            if isinstance(name_data, bytes):
                name = name_data.decode('utf-8').rstrip('\x00')[2:]  # Remove OB prefix
            else:
                name = str(name_data).rstrip('\x00')[2:]

            # FILTER: Only process objects starting with "STATIC"
            if not name.startswith('STATIC'):
                # Silently skip non-STATIC objects
                return None

            # Get object type
            obj_type = block.get(b'type')
            obj_type_name = self._get_object_type_name(obj_type)

            # Skip non-mesh objects for now (cameras, lights, etc.)
            if obj_type_name not in ['MESH', 'EMPTY', 'CURVE']:
                print(f"    Skipping {name} (type: {obj_type_name})")
                return None

            # Get transform data - using simple pattern
            location = tuple(block.get(b'loc'))
            rotation = tuple(block.get(b'rot'))  # in radians
            scale = tuple(block.get(b'size'))
            dimensions = self._get_dimensions(block)

            # Read custom properties
            custom_props = self.id_reader.read_properties(block)

            # Required: geometry_type (plane, cylinder, box, sphere, mesh)
            geometry_type_raw = custom_props.get('geometry_type')
            if not geometry_type_raw:
                print(f"    ⚠ Skipping {name}: missing 'geometry_type' custom property")
                return None
            geometry_type = geometry_type_raw.upper()

            # Required: material and look
            material = custom_props.get('material')
            look = custom_props.get('look')

            # Use defaults if not specified
            if not material:
                material = self.defaults.get('material', 'steel')
                print(f"    ⚠ {name}: using default material '{material}'")
            if not look:
                look = self.defaults.get('look', 'gray')
                print(f"    ⚠ {name}: using default look '{look}'")

            # Get mesh name for custom meshes
            mesh_name = None
            if geometry_type == 'MESH':
                mesh_name = self._get_mesh_name(block)

            blender_obj = BlenderObject(
                name=name,
                object_type=obj_type_name,
                geometry_type=geometry_type,
                location=location,
                rotation=rotation,
                scale=scale,
                dimensions=dimensions,
                material=material,
                look=look,
                mesh_name=mesh_name
            )

            print(f"    ✓ {name} [{geometry_type}] - material:{material}, look:{look}")
            return blender_obj

        except Exception as e:
            print(f"    Error extracting object: {e}")
            return None

    def _get_object_type_name(self, obj_type: int) -> str:
        """Convert object type integer to name"""
        type_map = {
            0: 'EMPTY',
            1: 'MESH',
            2: 'CURVE',
            4: 'SURFACE',
            5: 'META',
            6: 'FONT',
            10: 'LAMP',
            11: 'CAMERA',
            12: 'SPEAKER',
            25: 'ARMATURE',
            26: 'LATTICE',
        }
        return type_map.get(obj_type, 'UNKNOWN')

    def _get_dimensions(self, block) -> Tuple[float, float, float]:
        """Get object dimensions (bounding box)"""
        try:
            # Get mesh data
            data = block.get(b'data')
            if data:
                data_obj = data.dereference()
                if data_obj:
                    # Try to get bounding box
                    bb = data_obj.get(b'bb')
                    if bb:
                        # Calculate dimensions from bounding box
                        min_x = min(v[0] for v in bb)
                        max_x = max(v[0] for v in bb)
                        min_y = min(v[1] for v in bb)
                        max_y = max(v[1] for v in bb)
                        min_z = min(v[2] for v in bb)
                        max_z = max(v[2] for v in bb)

                        # Apply scale
                        scale = tuple(block.get(b'size'))
                        return (
                            abs(max_x - min_x) * scale[0],
                            abs(max_y - min_y) * scale[1],
                            abs(max_z - min_z) * scale[2]
                        )
        except:
            pass

        # Fallback to scale if dimensions can't be determined
        scale = tuple(block.get(b'size'))
        return (scale[0] * 2.0, scale[1] * 2.0, scale[2] * 2.0)

        return None

    def _get_mesh_name(self, block) -> Optional[str]:
        """Get the name of the mesh data"""
        try:
            data = block.get(b'data')
            if data:
                data_obj = data.dereference()
                if data_obj:
                    mesh_name_data = data_obj.get((b'id', b'name'))
                    if mesh_name_data:
                        if hasattr(mesh_name_data, 'decode'):
                            mesh_name = mesh_name_data.decode('utf-8').rstrip('\x00')
                            if mesh_name.startswith('ME'):
                                return mesh_name[2:]  # Remove "ME" prefix
                            return mesh_name
                        return str(mesh_name_data)
        except:
            pass

        return None

    def _infer_geometry_type(self, obj, name: str, dimensions: Tuple[float, float, float]) -> str:
        """
        Infer Stonefish geometry type from Blender object.

        Priority:
        1. Name-based inference (PLANE_, CYLINDER_, BOX_, SPHERE_ prefix)
        2. Dimension-based inference (flat = plane, equal dims = sphere, etc.)
        3. Default to MESH for complex geometry

        Args:
            obj: Blender object
            name: Object name
            dimensions: Object dimensions

        Returns:
            Geometry type: PLANE, CYLINDER, BOX, SPHERE, or MESH
        """
        # Name-based inference
        name_upper = name.upper()
        if name_upper.startswith('PLANE') or 'GROUND' in name_upper:
            return 'PLANE'
        if name_upper.startswith('CYLINDER') or name_upper.startswith('CYL'):
            return 'CYLINDER'
        if name_upper.startswith('BOX') or name_upper.startswith('CUBE'):
            return 'BOX'
        if name_upper.startswith('SPHERE') or name_upper.startswith('BALL'):
            return 'SPHERE'

        # Dimension-based inference
        x, y, z = dimensions
        tolerance = 0.01

        # Sphere: all dimensions roughly equal
        if abs(x - y) < tolerance and abs(y - z) < tolerance and abs(x - z) < tolerance:
            if x > 0.1:  # Avoid tiny objects
                return 'SPHERE'

        # Plane: one dimension much smaller than others
        if z < 0.01 and x > 0.1 and y > 0.1:
            return 'PLANE'

        # Cylinder: two dimensions equal, one different
        if abs(x - y) < tolerance and abs(x - z) > tolerance:
            return 'CYLINDER'

        # Default to mesh for complex shapes
        return 'MESH'

    def filter_objects(self, filter_func) -> List[BlenderObject]:
        """
        Filter extracted objects using a custom function.

        Args:
            filter_func: Function that takes BlenderObject and returns bool

        Returns:
            Filtered list of objects
        """
        return [obj for obj in self.objects if filter_func(obj)]


