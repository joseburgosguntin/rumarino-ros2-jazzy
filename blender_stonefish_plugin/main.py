#!/usr/bin/env python3
"""
Blender to Stonefish Converter

Converts Blender scenes to Stonefish scenario files (.scn) with automatic
coordinate system transformation from Blender to Stonefish conventions.

Coordinate Transformation:
    - Blender: X=right, Y=forward, Z=up
    - Stonefish: X=forward, Y=left, Z=up

Usage:
    python3 main.py input.blend -o output.scn -c config.yaml
    python3 main.py scene.blend --config my_config.yaml --output scenario.scn
"""

import sys
import argparse
import yaml
from pathlib import Path
from typing import Dict, List, Optional
import logging
from stonefish_objects import (
    Material,
    Look,
    FrictionTable,
    FrictionPair,
    WorldTransform,
    StonefishScenario,
    Ocean,
    Atmosphere,
    Sun,
    Environment,
    Plane,
    Box,
    Cylinder,
    Sphere,
    Model,
    PhysicsProperties,
    PhysicsType,
)
from blender_extractor import BlenderExtractor, BlenderObject

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


class ConfigLoader:
    """Loads and validates configuration from YAML file"""

    def __init__(self, config_path: str):
        self.config_path = config_path

        self.config = self._load_config()

    def _load_config(self) -> Dict:
        with open(self.config_path, "r") as f:
            config = yaml.safe_load(f)
            logger.info(f"Loaded yaml configuration from {self.config_path}")
            return config

    def get_materials(self) -> List[Material]:
        """Convert config materials to Material objects"""
        materials = []
        for name, props in self.config["materials"].items():
            mat = Material(
                name=name,
                density=props["density"],
                restitution=props["restitution"],
                magnetic=props["magnetic"],
            )
            materials.append(mat)
        return materials

    def get_looks(self) -> List[Look]:
        """Convert config looks to Look objects"""
        looks = []
        for name, props in self.config["looks"].items():
            color = tuple(props["color"])
            look = Look(
                name=name,
                color=color,
                roughness=props["roughness"],
                metalness=props["metalness"],
                reflectivity=props["reflectivity"],
            )
            looks.append(look)
        return looks

    def get_friction_table(self) -> FrictionTable:
        """Convert config friction pairs to FrictionTable"""
        table = FrictionTable()
        for pair in self.config["friction"]["pairs"]:
            if len(pair) == 4:
                mat1, mat2, static, dynamic = pair
                table.add_pair(mat1, mat2, static, dynamic)
            else:
                raise Exception(
                    f"Error Trying to parse friction table. Expected 4 fields but got {len(pair)}"
                )
        return table

    def get_environment(self) -> Environment:
        """Convert config environment to Environment object"""
        env_config = self.config["environment"]

        ocean = None
        ocean_config = env_config["ocean"]
        if ocean_config["enabled"]:
            current_vel = ocean_config["current"]["velocity"]
            ocean = Ocean(
                waves=ocean_config["wave_height"],
                particles=ocean_config["particles"],
                current=tuple(current_vel),
            )

        # Atmosphere
        atm_config = env_config["atmosphere"]
        atmosphere = Atmosphere(
            pressure=atm_config["pressure"], temperature=atm_config["temperature"]
        )

        # Sun
        sun_config = env_config["sun"]
        sun = Sun(azimuth=sun_config["azimuth"], elevation=sun_config["elevation"])

        return Environment(ocean=ocean, atmosphere=atmosphere, sun=sun)

    def get(self, key: str, default=None):
        """Get configuration value by key path (e.g., 'coordinates.transform_coordinates')"""
        keys = key.split(".")
        value = self.config
        for k in keys:
            if not isinstance(value, dict):
                return default
            value = value.get(k, default)
        return value if value is not None else default


class StonefishScenarioBuilder:
    """Builds Stonefish scenarios from configuration and static entities (e.g., from Blender)"""

    def __init__(self, config: ConfigLoader):
        self.config = config
        self.scenario = StonefishScenario(name="Scenario")
        self._initialize_scenario()

    def _initialize_scenario(self):
        """Initialize scenario with materials, looks, friction, and environment from config"""
        for material in self.config.get_materials():
            self.scenario.add_material(material)

        for look in self.config.get_looks():
            self.scenario.add_look(look)

        self.scenario.friction_table = self.config.get_friction_table()
        self.scenario.environment = self.config.get_environment()

    def convert(self, blend_file: str, output_file: Path) -> str:
        """
        Convert Blender file to Stonefish scenario

        Args:
            blend_file: Path to .blend file
            output_file: Output .scn file path (optional)

        Returns:
            Path to generated scenario file
        """
        # Extract objects from Blender file
        defaults = {
            "material": self.config.get("defaults.material", "steel"),
            "look": self.config.get("defaults.look", "gray"),
        }
        self.extractor = BlenderExtractor(blend_file, defaults)
        blender_objects = self.extractor.extract()

        if not blender_objects:
            logger.warning("No Blender Objects Extracted.")
        else:
            self._convert_blender_objects(blender_objects)

        xml_content = self._generate_xml()

        with open(output_file, "w") as f:
            f.write(xml_content)

        return str(output_file)

    def _convert_blender_objects(self, blender_objects: List[BlenderObject]):
        """
        Convert extracted Blender objects to Stonefish objects.

        Args:
            blender_objects: List of BlenderObject instances
        """
        logger.info(
            f"Converting {len(blender_objects)} Blender objects to Stonefish..."
        )

        for blender_obj in blender_objects:
            # Transform coordinates: Blender (X=right, Y=forward, Z=up) -> Stonefish (X=forward, Y=left, Z=up)
            # This transformation is always applied as it's the purpose of this tool
            x_sf = blender_obj.location[1]  # Blender Y -> Stonefish X (forward)
            y_sf = -blender_obj.location[0]  # Blender -X -> Stonefish Y (left)
            z_sf = -blender_obj.location[2]  # Blender Z -> Stonefish Z (up)

            # Rotation transformation (approximate - needs proper rotation matrix conversion)
            r_sf = blender_obj.rotation[
                1
            ]  # Blender Y rotation -> Stonefish X rotation (pitch)
            p_sf = -blender_obj.rotation[
                0
            ]  # Blender -X rotation -> Stonefish Y rotation (roll)
            y_sf_rot = blender_obj.rotation[
                2
            ]  # Blender Z rotation -> Stonefish Z rotation (yaw)

            world_transform = WorldTransform(
                xyz=(x_sf, y_sf, z_sf), rpy=(r_sf, p_sf, y_sf_rot)
            )

            # Create appropriate Stonefish object based on geometry type
            if blender_obj.geometry_type == "PLANE":
                obj = Plane(
                    name=blender_obj.name,
                    world_transform=world_transform,
                    material=blender_obj.material,
                    look=blender_obj.look,
                )
                self.scenario.add_object(obj)

            elif blender_obj.geometry_type == "CYLINDER":
                # For cylinder, determine radius and height from dimensions
                x, y, z = blender_obj.dimensions
                # Swap dimensions according to Blender->Stonefish coordinate system
                radius = max(x, z) / 2.0
                height = y

                obj = Cylinder(
                    name=blender_obj.name,
                    world_transform=world_transform,
                    material=blender_obj.material,
                    look=blender_obj.look,
                    radius=radius,
                    height=height,
                )
                self.scenario.add_object(obj)

            elif blender_obj.geometry_type == "BOX":
                # Box dimensions: Blender (X,Y,Z) -> Stonefish (Y,X,Z)
                dimensions = (
                    blender_obj.dimensions[1],
                    blender_obj.dimensions[0],
                    blender_obj.dimensions[2],
                )

                obj = Box(
                    name=blender_obj.name,
                    world_transform=world_transform,
                    material=blender_obj.material,
                    look=blender_obj.look,
                    dimensions=dimensions,
                )
                self.scenario.add_object(obj)

            elif blender_obj.geometry_type == "SPHERE":
                # Sphere radius from average dimension
                radius = (
                    sum(blender_obj.dimensions) / 6.0
                )  # Average of 3 dimensions / 2

                obj = Sphere(
                    name=blender_obj.name,
                    world_transform=world_transform,
                    material=blender_obj.material,
                    look=blender_obj.look,
                    radius=radius,
                )
                self.scenario.add_object(obj)

            elif blender_obj.geometry_type == "MESH":
                # Custom mesh - export to file
                mesh_dir = self.config.get("meshes.output_dir", "models")
                mesh_format = self.config.get("meshes.format", "obj")
                mesh_scale_val = self.config.get("meshes.scale", 0.01)
                mesh_scale = float(mesh_scale_val) if mesh_scale_val is not None else 0.01  # type: ignore

                mesh_filename = f"{mesh_dir}/{blender_obj.mesh_name or blender_obj.name}.{mesh_format}"

                # Export mesh data to OBJ file (always transform coords for Blender->Stonefish)
                if blender_obj.mesh_data_block:
                    success = self.extractor.export_mesh_to_obj(
                        blender_obj.mesh_data_block,
                        mesh_filename,
                        scale=mesh_scale,
                        transform_coords=True,
                    )
                    if success:
                        logger.info(
                            f"Exported mesh '{blender_obj.name}' to {mesh_filename}"
                        )
                    else:
                        logger.error(f"Failed to export mesh '{blender_obj.name}'")
                else:
                    logger.warning(f"No mesh data available for '{blender_obj.name}'")

                obj = Model(
                    name=blender_obj.name,
                    world_transform=world_transform,
                    material=blender_obj.material,
                    look=blender_obj.look,
                    mesh_filename=mesh_filename,
                    mesh_scale=mesh_scale,
                )
                self.scenario.add_object(obj)

        logger.info(f"Converted {len(self.scenario.static_objects)} static objects")

    def _generate_xml(self) -> str:
        """Generate Stonefish XML from scenario using composable pattern"""
        return self.scenario.generate_xml_string()


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description="Convert Blender scenes to Stonefish scenarios",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("input", help="Input .blend file")
    parser.add_argument("-o", "--output", help="Output .scn file (default: input.scn)")
    parser.add_argument("-c", "--config", help="Configuration YAML file")

    args = parser.parse_args()

    if not Path(args.input).exists():
        logging.error(f"Input Blender file not found: {args.input}")
        sys.exit(1)

    config = ConfigLoader(args.config)

    builder = StonefishScenarioBuilder(config)
    output_path = builder.convert(args.input, args.output)
    logger.info(f"Scenario written to: {output_path}")


if __name__ == "__main__":
    main()
