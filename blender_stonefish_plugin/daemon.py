#!/usr/bin/env python3
"""
Stonefish Converter Daemon

Watches Blender and config files for changes and automatically regenerates
the Stonefish scenario file.

Usage:
    python3 watch_and_convert.py <blend_file> <config_file> <output_file>
    python3 watch_and_convert.py scene.blend config_new.yaml output.scn
"""

import sys
import logging
import argparse
from pathlib import Path
from datetime import datetime
from typing import Set, Optional

import inotify.adapters

from main import ConfigLoader, StonefishScenarioBuilder

logger = logging.getLogger("blender_stonefish_plugin.daemon")


class FileWatcher:
    """Watches files for modifications and triggers conversion using inotify"""

    def __init__(self, blend_file: Path, config_file: Path, output_file: Path):
        self.blend_file = blend_file
        self.config_file = config_file
        self.output_file = output_file

        # Track if this is first run
        self.first_run = True

        # Get unique directories to watch
        self.watch_dirs = set()
        self.watch_files = {}  # Maps directory -> set of filenames we care about

        for filepath in [self.blend_file, self.config_file]:
            parent_dir = str(filepath.parent.resolve())
            filename = filepath.name

            if parent_dir not in self.watch_files:
                self.watch_files[parent_dir] = set()
            self.watch_files[parent_dir].add(filename)
            self.watch_dirs.add(parent_dir)

    def _convert(self, changed_files: Optional[Set[Path]] = None):
        """Run the conversion"""
        try:
            if self.first_run:
                logger.info("Starting initial conversion...")
                self.first_run = False
            else:
                changed_names = [f.name for f in changed_files] if changed_files else []
                logger.info(f"Change detected in: {', '.join(changed_names)}")
                logger.info("Regenerating scenario...")

            # Load config and convert
            config = ConfigLoader(str(self.config_file))
            builder = StonefishScenarioBuilder(config)

            output_path = builder.convert(str(self.blend_file), self.output_file)

            logger.info(f"Successfully wrote: {output_path}")

        except FileNotFoundError as e:
            logger.error(f"File not found - {e}")
        except Exception as e:
            logger.error(f"Error during conversion: {e}")
            import traceback

            traceback.print_exc()

    def watch(self):
        """Watch files and trigger conversion on changes using inotify"""
        logger.info("=" * 80)
        logger.info("Stonefish Converter Daemon (inotify)")
        logger.info("=" * 80)
        logger.info("Watching:")
        logger.info(f"  Blender file: {self.blend_file}")
        logger.info(f"  Config file:  {self.config_file}")
        logger.info(f"  Output file:  {self.output_file}")
        logger.info("Using inotify for efficient file monitoring")
        logger.info("Press Ctrl+C to stop")
        logger.info("=" * 80)

        # Do initial conversion
        self._convert()

        try:
            # Create inotify adapter watching all relevant directories
            i = inotify.adapters.Inotify()

            for watch_dir in self.watch_dirs:
                i.add_watch(watch_dir.encode("utf-8"))
                logger.info(f"Watching directory: {watch_dir}")

            # Watch for modifications, moves, and close-after-write events
            for event in i.event_gen(yield_nones=False):
                (_, type_names, path, filename) = event

                path_str = path.decode("utf-8")
                filename_str = filename.decode("utf-8")

                # Check if this is a file we care about
                if (
                    path_str in self.watch_files
                    and filename_str in self.watch_files[path_str]
                ):
                    # Filter for actual modification events
                    if any(
                        t in type_names
                        for t in ["IN_MODIFY", "IN_CLOSE_WRITE", "IN_MOVED_TO"]
                    ):
                        changed_file = Path(path_str) / filename_str
                        self._convert({changed_file})

        except KeyboardInterrupt:
            logger.info("Stopping file watcher. Goodbye!")
            sys.exit(0)


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description="Watch Blender and config files, auto-regenerate Stonefish scenarios",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
  python3 daemon.py scene.blend config_new.yaml output.scn
        """,
    )

    parser.add_argument("blend_file", help="Blender .blend file to watch")
    parser.add_argument("config_file", help="Configuration YAML file to watch")
    parser.add_argument("output_file", help="Output .scn file path")

    args = parser.parse_args()

    # Validate inputs
    blend_path = Path(args.blend_file)
    config_path = Path(args.config_file)
    output_path = Path(args.output_file)

    if not blend_path.exists():
        logger.error(f"Blender file not found: {blend_path}")
        sys.exit(1)

    if not config_path.exists():
        logger.error(f"Config file not found: {config_path}")
        sys.exit(1)

    # Create and start watcher
    watcher = FileWatcher(blend_path, config_path, output_path)
    watcher.watch()


if __name__ == "__main__":
    main()
