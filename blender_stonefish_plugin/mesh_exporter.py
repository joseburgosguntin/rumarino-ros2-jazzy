"""
Mesh Exporter for Blender .blend files

This module handles exporting mesh data from Blender files to OBJ format.
It supports both legacy (pre-3.0) and modern (3.0+) Blender mesh formats.

Modern Blender stores mesh data in CustomData layers which require special parsing.
"""

import os
import struct
import logging
from pathlib import Path
from typing import List, Tuple, Optional

logger = logging.getLogger("blender_stonefish_plugin.mesh_exporter")


class CustomDataLayer:
    """Represents a CustomData layer in Blender"""

    def __init__(self, layer_block):
        self.block = layer_block
        self.type = None
        self.name = None
        self.data_ptr = None

    def read_info(self):
        """Read layer information"""
        try:
            self.type = self.block.get(b"type", default=0)
            self.name = self.block.get(b"name", default=b"")
            self.data_ptr = self.block.get(b"data", default=0)
            return True
        except:
            return False


class MeshExporter:
    """
    Exports mesh data from Blender files to OBJ format.

    Handles both legacy (mvert/mpoly/mloop) and modern (CustomData) formats.
    """

    # CustomData type constants from Blender DNA
    CD_MVERT = 0
    CD_MSTICKY = 1
    CD_MDEFORMVERT = 2
    CD_MEDGE = 3
    CD_MFACE = 4
    CD_MTFACE = 5
    CD_MCOL = 6
    CD_ORIGINDEX = 7
    CD_NORMAL = 8
    CD_FACEMAP = 9
    CD_PROP_FLOAT = 10
    CD_PROP_INT32 = 11
    CD_PROP_STRING = 12
    CD_ORIGSPACE = 13
    CD_ORCO = 14
    CD_MTEXPOLY = 15
    CD_MLOOPUV = 16
    CD_MLOOPCOL = 17
    CD_TANGENT = 18
    CD_MDISPS = 19
    CD_PREVIEW_MCOL = 20
    CD_PAINT_MASK = 21
    CD_GRID_PAINT_MASK = 22
    CD_MVERT_SKIN = 23
    CD_FREESTYLE_EDGE = 24
    CD_FREESTYLE_FACE = 25
    CD_MLOOPTANGENT = 26
    CD_TESSLOOPNORMAL = 27
    CD_CUSTOMLOOPNORMAL = 28
    CD_SCULPT_FACE_SETS = 29
    CD_LOCATION = 30
    CD_RADIUS = 31
    CD_PROP_FLOAT3 = 32
    CD_PROP_BOOL = 33
    CD_CREASE = 34
    CD_PROP_FLOAT2 = 35
    CD_PROP_INT8 = 36
    CD_PROP_QUATERNION = 45
    CD_PROP_COLOR = 47
    CD_POSITION = 48  # Modern Blender position attribute

    def __init__(self, blend_file):
        """
        Initialize the mesh exporter.

        Args:
            blend_file: BlendFile object from blendfile module
        """
        self.blend = blend_file
        self.header = blend_file.header

    def export_to_obj(
        self,
        mesh_block,
        output_path: str,
        scale: float = 1.0,
        transform_coords: bool = True,
    ) -> bool:
        """
        Export mesh to OBJ file format.

        Args:
            mesh_block: Mesh data block from Blender
            output_path: Path to output .obj file
            scale: Scale factor to apply
            transform_coords: Apply Blender->Stonefish coordinate transformation

        Returns:
            True if successful, False otherwise
        """
        logger.info(f" Exporting mesh to {output_path}")

        # Try legacy format first (faster and simpler)
        vertices, faces = self._try_legacy_format(mesh_block)

        if not vertices:
            # Try modern CustomData format
            logger.debug(f"    Legacy format not available, trying CustomData...")
            vertices, faces = self._try_customdata_format(mesh_block)

        if not vertices:
            logger.error(f" Could not read mesh data")
            return False

        if not faces:
            logger.warning(f" No faces found, creating point cloud")

        # Apply transformations
        transformed_vertices = []
        for v in vertices:
            x, y, z = v
            if transform_coords:
                # Blender (X=right, Y=forward, Z=up) -> Stonefish (X=forward, Y=left, Z=up)
                x_sf = y * scale
                y_sf = -x * scale
                z_sf = z * scale
                transformed_vertices.append((x_sf, y_sf, z_sf))
            else:
                transformed_vertices.append((x * scale, y * scale, z * scale))

        # Write OBJ file
        return self._write_obj_file(output_path, transformed_vertices, faces)

    def _try_legacy_format(
        self, mesh_block
    ) -> Tuple[List[Tuple[float, float, float]], List[List[int]]]:
        """
        Try to read mesh using legacy format (mvert, mpoly, mloop).

        Returns:
            (vertices, faces) tuple or ([], []) if not available
        """
        vertices = []
        faces = []

        try:
            # Check if legacy pointers exist
            mvert_ptr = mesh_block.get(b"mvert", default=0)
            totvert = mesh_block.get(b"totvert", default=0)

            if not mvert_ptr or mvert_ptr == 0 or not totvert:
                return vertices, faces

            logger.debug(f"    Reading {totvert} vertices (legacy format)...")

            # Read vertices
            vert_block = self.blend.find_block_from_offset(mvert_ptr)
            if vert_block:
                for i in range(totvert):
                    try:
                        co = vert_block.get(b"co", base_index=i, default=None)
                        if isinstance(co, (list, tuple)) and len(co) >= 3:
                            vertices.append((float(co[0]), float(co[1]), float(co[2])))
                    except:
                        continue

            if not vertices:
                return vertices, faces

            # Read faces
            mpoly_ptr = mesh_block.get(b"mpoly", default=0)
            totpoly = mesh_block.get(b"totpoly", default=0)
            mloop_ptr = mesh_block.get(b"mloop", default=0)

            if (
                mpoly_ptr
                and mpoly_ptr != 0
                and totpoly
                and mloop_ptr
                and mloop_ptr != 0
            ):
                logger.debug(f"    Reading {totpoly} faces (legacy format)...")
                poly_block = self.blend.find_block_from_offset(mpoly_ptr)
                loop_block = self.blend.find_block_from_offset(mloop_ptr)

                if poly_block and loop_block:
                    for i in range(totpoly):
                        try:
                            loopstart = poly_block.get(
                                b"loopstart", base_index=i, default=0
                            )
                            totloop = poly_block.get(
                                b"totloop", base_index=i, default=0
                            )

                            if isinstance(loopstart, (list, tuple)):
                                loopstart = loopstart[0] if loopstart else 0
                            if isinstance(totloop, (list, tuple)):
                                totloop = totloop[0] if totloop else 0

                            face_indices = []
                            for j in range(int(totloop)):
                                loop_idx = int(loopstart) + j
                                v_idx = loop_block.get(
                                    b"v", base_index=loop_idx, default=0
                                )
                                if isinstance(v_idx, (list, tuple)):
                                    v_idx = v_idx[0] if v_idx else 0
                                face_indices.append(
                                    int(v_idx) + 1
                                )  # OBJ uses 1-based indexing

                            if len(face_indices) >= 3:
                                faces.append(face_indices)
                        except:
                            continue

            logger.info(f" Read {len(vertices)} vertices, {len(faces)} faces")
            return vertices, faces

        except Exception as e:
            logger.warning(f" Legacy format read failed: {e}")
            return [], []

    def _try_customdata_format(
        self, mesh_block
    ) -> Tuple[List[Tuple[float, float, float]], List[List[int]]]:
        """
        Try to read mesh using modern CustomData format.

        Returns:
            (vertices, faces) tuple or ([], []) if not available
        """
        vertices = []
        faces = []

        try:
            totvert = mesh_block.get(b"totvert", default=0)
            totpoly = mesh_block.get(b"totpoly", default=0)

            if not totvert:
                return vertices, faces

            logger.debug(f"    Reading {totvert} vertices from CustomData...")

            # vdata is an embedded CustomData struct, not a pointer
            # We need to access its fields directly through the mesh block
            # CustomData fields: layers (pointer), totlayer, maxlayer
            try:
                layers_ptr = mesh_block.get((b"vdata", b"layers"), default=0)
                totlayer = mesh_block.get((b"vdata", b"totlayer"), default=0)
            except:
                logger.warning(f" Could not access vdata CustomData structure")
                return vertices, faces

            if not layers_ptr or not totlayer:
                logger.debug(f"      No CustomData layers found")
                return vertices, faces

            logger.debug(f"      Found {totlayer} CustomData layers in vdata")

            # Find the layers block
            layers_block = self.blend.find_block_from_offset(layers_ptr)
            if not layers_block:
                logger.debug(f"      Could not find layers block")
                return vertices, faces

            # Search for position data layer (CD_PROP_FLOAT3 or CD_LOCATION)
            for layer_idx in range(totlayer):
                try:
                    layer_type = layers_block.get(
                        b"type", base_index=layer_idx, default=-1
                    )
                    layer_name = layers_block.get(
                        b"name", base_index=layer_idx, default=b""
                    )

                    logger.debug(
                        f"      Layer {layer_idx}: type={layer_type}, name={layer_name}"
                    )

                    # Check if this is a position layer
                    # CD_PROP_FLOAT3 (32), CD_LOCATION (30), or CD_POSITION (48) store positions
                    if layer_type in [
                        self.CD_PROP_FLOAT3,
                        self.CD_LOCATION,
                        self.CD_POSITION,
                    ]:
                        data_ptr = layers_block.get(
                            b"data", base_index=layer_idx, default=0
                        )

                        if data_ptr and data_ptr != 0:
                            logger.info(
                                f" Found position layer at type {layer_type} (data_ptr=0x{data_ptr:x})"
                            )
                            vertices = self._read_float3_array(data_ptr, totvert)
                            if vertices:
                                break
                            else:
                                logger.warning(
                                    f" Could not read data from position layer"
                                )

                except Exception as e:
                    logger.warning(f" Error reading layer {layer_idx}: {e}")
                    continue

            if not vertices:
                logger.warning(f" Could not read vertices from vdata")
                return vertices, faces

            # Read polygon data from pdata and ldata
            if totpoly:
                logger.debug(f"    Reading {totpoly} faces from CustomData...")
                try:
                    faces = self._read_customdata_faces(mesh_block, totpoly)
                except Exception as e:
                    logger.debug(f"      Error reading faces: {e}")

            logger.info(f" Read {len(vertices)} vertices, {len(faces)} faces")
            return vertices, faces

        except Exception as e:
            logger.warning(f" CustomData format read failed: {e}")
            import traceback

            traceback.print_exc()
            return [], []

    def _read_customdata_vertices(
        self, vdata, totvert: int
    ) -> List[Tuple[float, float, float]]:
        """
        Read vertex positions from CustomData structure.

        CustomData contains layers of different types. We need to find the position layer.
        """
        vertices = []

        try:
            # CustomData structure has: typemap, totlayer, maxlayer, totsize, layers, pool, external
            layers_ptr = vdata.get(b"layers", default=0)
            totlayer = vdata.get(b"totlayer", default=0)

            if not layers_ptr or not totlayer:
                logger.debug(f"      No CustomData layers found")
                return vertices

            logger.debug(f"      Found {totlayer} CustomData layers")

            # Find the layers block
            layers_block = self.blend.find_block_from_offset(layers_ptr)
            if not layers_block:
                logger.debug(f"      Could not find layers block")
                return vertices

            # Search for position data layer (CD_PROP_FLOAT3 or CD_LOCATION)
            for layer_idx in range(totlayer):
                try:
                    layer_type = layers_block.get(
                        b"type", base_index=layer_idx, default=-1
                    )
                    layer_name = layers_block.get(
                        b"name", base_index=layer_idx, default=b""
                    )

                    logger.debug(
                        f"      Layer {layer_idx}: type={layer_type}, name={layer_name}"
                    )

                    # Check if this is a position layer
                    # CD_PROP_FLOAT3 (32) or CD_LOCATION (30) typically store positions
                    if layer_type in [self.CD_PROP_FLOAT3, self.CD_LOCATION]:
                        data_ptr = layers_block.get(
                            b"data", base_index=layer_idx, default=0
                        )

                        if data_ptr and data_ptr != 0:
                            logger.info(f" Found position layer at type {layer_type}")
                            vertices = self._read_float3_array(data_ptr, totvert)
                            if vertices:
                                return vertices

                except Exception as e:
                    logger.warning(f" Error reading layer {layer_idx}: {e}")
                    continue

            # If we didn't find position in layers, try alternative approach
            # Some Blender versions store positions directly
            logger.debug(f"      Trying alternative position reading...")

        except Exception as e:
            logger.debug(f"      Error in _read_customdata_vertices: {e}")
            import traceback

            traceback.print_exc()

        return vertices

    def _read_float3_array(
        self, data_ptr: int, count: int
    ) -> List[Tuple[float, float, float]]:
        """Read an array of float[3] (vec3) from a data pointer"""
        vertices = []

        try:
            data_block = self.blend.find_block_from_offset(data_ptr)
            if not data_block:
                logger.debug(f"        Could not find data block at 0x{data_ptr:x}")
                return vertices

            logger.debug(
                f"        Data block found: type={data_block.dna_type_name}, size={data_block.size}, count={data_block.count}"
            )

            # The data block should contain float[3] arrays
            # Try different reading strategies

            # Strategy 1: Read as raw float data
            try:
                # Use get_raw_data which reads primitive types directly
                logger.debug(f"        Trying get_raw_data method...")
                float_data = data_block.get_raw_data(b"float")
                if float_data and len(float_data) >= count * 3:
                    logger.info(f" Read {len(float_data)} floats")
                    for i in range(count):
                        idx = i * 3
                        vertices.append(
                            (
                                float(float_data[idx]),
                                float(float_data[idx + 1]),
                                float(float_data[idx + 2]),
                            )
                        )
                    return vertices
            except Exception as e:
                logger.debug(f"        get_raw_data failed: {e}")

            # Strategy 2: Try reading individual elements
            logger.debug(f"        Trying individual element reading...")
            for i in range(min(5, count)):  # Try first 5 to debug
                try:
                    # Try different access patterns
                    val = data_block[i] if hasattr(data_block, "__getitem__") else None
                    if val:
                        logger.debug(f"          Element {i}: {val}")
                except:
                    pass

        except Exception as e:
            logger.debug(f"        Error reading float3 array: {e}")
            import traceback

            traceback.print_exc()

        return vertices

    def _read_raw_float3_data(
        self, data_block, count: int
    ) -> List[Tuple[float, float, float]]:
        """Read raw float data directly from block"""
        vertices = []

        try:
            # Try to use get_raw_data if available
            float_data = data_block.get_raw_data(b"float", base_index=0)
            if float_data and len(float_data) >= count * 3:
                for i in range(count):
                    idx = i * 3
                    vertices.append(
                        (
                            float(float_data[idx]),
                            float(float_data[idx + 1]),
                            float(float_data[idx + 2]),
                        )
                    )
        except Exception as e:
            logger.debug(f"        Raw float read failed: {e}")

        return vertices

    def _read_customdata_faces(self, mesh_block, totpoly: int) -> List[List[int]]:
        """Read face data from CustomData"""
        faces = []

        try:
            # Get poly_offset_indices (modern Blender uses this to define polygon loop ranges)
            poly_offsets_ptr = mesh_block.get(b"poly_offset_indices", default=0)
            totloop = mesh_block.get(b"totloop", default=0)

            if not poly_offsets_ptr or not totloop:
                logger.debug(f"      No poly_offset_indices or totloop found")
                return faces

            logger.debug(
                f"      Reading face data: totpoly={totpoly}, totloop={totloop}"
            )

            # Read poly offsets
            offsets_block = self.blend.find_block_from_offset(poly_offsets_ptr)
            if not offsets_block:
                logger.debug(f"      Could not find poly_offset_indices block")
                return faces

            logger.debug(
                f"      Offsets block: type={offsets_block.dna_type_name}, size={offsets_block.size}, count={offsets_block.count}"
            )

            # Try to read offsets as raw int data
            try:
                logger.debug(f"      Trying to read poly offsets as raw int data...")
                offsets_data = offsets_block.get_raw_data(b"int")
                if offsets_data and len(offsets_data) >= totpoly:
                    offsets = [int(x) for x in offsets_data[: totpoly + 1]]
                    logger.info(
                        f" Read {len(offsets)} offsets: first few = {offsets[:min(5, len(offsets))]}"
                    )
                else:
                    logger.debug(
                        f"      Could not read offsets as raw int (got {len(offsets_data) if offsets_data else 0} values)"
                    )
                    offsets = []
            except Exception as e:
                logger.debug(f"      Error reading offsets: {e}")
                offsets = []

            # Read loop vertex indices from ldata
            # ldata is an embedded CustomData struct
            try:
                ldata_layers_ptr = mesh_block.get((b"ldata", b"layers"), default=0)
                ldata_totlayer = mesh_block.get((b"ldata", b"totlayer"), default=0)
            except:
                logger.debug(f"      Could not access ldata CustomData")
                return faces

            if not ldata_layers_ptr or not ldata_totlayer:
                logger.debug(f"      No ldata layers found")
                return faces

            logger.debug(f"      Found {ldata_totlayer} ldata layers")

            # Find vertex index layer in ldata
            ldata_layers_block = self.blend.find_block_from_offset(ldata_layers_ptr)
            if not ldata_layers_block:
                logger.debug(f"      Could not find ldata layers block")
                return faces

            # Search for vertex index layer (.v field of MLoop)
            loop_verts_ptr = None
            for layer_idx in range(ldata_totlayer):
                try:
                    layer_type = ldata_layers_block.get(
                        b"type", base_index=layer_idx, default=-1
                    )
                    layer_name = ldata_layers_block.get(
                        b"name", base_index=layer_idx, default=b""
                    )

                    logger.debug(
                        f"        ldata Layer {layer_idx}: type={layer_type}, name={layer_name}"
                    )

                    # Look for vertex index data (we need to find the right type)
                    # This might be stored differently, let's try to get any data pointer
                    data_ptr = ldata_layers_block.get(
                        b"data", base_index=layer_idx, default=0
                    )
                    if data_ptr and data_ptr != 0:
                        loop_verts_ptr = data_ptr
                        logger.info(
                            f" Using layer {layer_idx} for loop data (ptr=0x{data_ptr:x})"
                        )
                        break
                except:
                    continue

            if not loop_verts_ptr:
                logger.debug(f"      Could not find loop vertex data")
                return faces

            # Read loop vertex indices
            loop_block = self.blend.find_block_from_offset(loop_verts_ptr)
            if not loop_block:
                logger.debug(f"      Could not find loop data block")
                return faces

            logger.debug(
                f"      Loop block: type={loop_block.dna_type_name}, size={loop_block.size}, count={loop_block.count}"
            )

            # Try to read loop vertex indices as raw int data
            try:
                logger.debug(f"      Trying to read loop vertices as raw int data...")
                loop_verts_data = loop_block.get_raw_data(b"int")
                if loop_verts_data and len(loop_verts_data) >= totloop:
                    logger.info(f" Read {len(loop_verts_data)} loop vertex indices")
                else:
                    logger.debug(
                        f"      Could not read loop verts (got {len(loop_verts_data) if loop_verts_data else 0} values)"
                    )
                    loop_verts_data = []
            except Exception as e:
                logger.debug(f"      Error reading loop verts: {e}")
                loop_verts_data = []

            # Read polygon offsets and construct faces
            try:
                if not offsets:
                    logger.debug(f"      No offsets available")
                    return faces

                if not loop_verts_data:
                    logger.debug(f"      No loop vertex data available")
                    return faces

                # Build faces from offsets and loop vertex data
                for poly_idx in range(totpoly):
                    try:
                        loop_start = offsets[poly_idx]
                        loop_end = (
                            offsets[poly_idx + 1]
                            if poly_idx + 1 < len(offsets)
                            else totloop
                        )
                        loop_count = loop_end - loop_start

                        if loop_count < 3:
                            continue

                        # Get vertex indices for this face from loop_verts_data
                        face_indices = []
                        for loop_idx in range(loop_start, loop_end):
                            if loop_idx < len(loop_verts_data):
                                v_idx = loop_verts_data[loop_idx]
                                face_indices.append(
                                    int(v_idx) + 1
                                )  # OBJ uses 1-based indexing

                        if len(face_indices) >= 3:
                            faces.append(face_indices)
                    except Exception as e:
                        if poly_idx < 5:  # Only print first few errors
                            logger.debug(f"        Error on face {poly_idx}: {e}")
                        continue

                logger.info(f" Constructed {len(faces)} faces")

            except Exception as e:
                logger.debug(f"      Error constructing faces: {e}")
                import traceback

                traceback.print_exc()

        except Exception as e:
            logger.debug(f"      Error reading faces: {e}")
            import traceback

            traceback.print_exc()

        return faces

    def _read_faces_with_offsets(
        self, poly_offsets_ptr: int, totpoly: int, mesh_block
    ) -> List[List[int]]:
        """Read faces using poly_offset_indices"""
        faces = []

        try:
            offsets_block = self.blend.find_block_from_offset(poly_offsets_ptr)
            if not offsets_block:
                return faces

            # Get loop data
            totloop = mesh_block.get(b"totloop", default=0)
            ldata = mesh_block.get(b"ldata", default=None)

            if not totloop or not ldata:
                return faces

            # Read loop vertex indices from ldata
            # This is complex - would need to parse ldata CustomData layers
            logger.debug(
                f"        Reading loops from CustomData (totloop={totloop})..."
            )

        except Exception as e:
            logger.debug(f"        Error reading faces with offsets: {e}")

        return faces

    def _write_obj_file(
        self,
        output_path: str,
        vertices: List[Tuple[float, float, float]],
        faces: List[List[int]],
    ) -> bool:
        """Write vertices and faces to OBJ file"""
        try:
            Path(output_path).parent.mkdir(parents=True, exist_ok=True)

            with open(output_path, "w") as f:
                f.write(f"# Exported from Blender by blender_stonefish_plugin\n")
                f.write(f"# Vertices: {len(vertices)}, Faces: {len(faces)}\n\n")

                # Write vertices
                for v in vertices:
                    f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")

                f.write("\n")

                # Write faces
                for face in faces:
                    f.write(f"f {' '.join(str(idx) for idx in face)}\n")

            logger.info(
                f" Exported to {output_path} ({len(vertices)} vertices, {len(faces)} faces)"
            )
            return True

        except Exception as e:
            logger.error(f" Error writing OBJ file: {e}")
            import traceback

            traceback.print_exc()
            return False
