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
import time
import argparse
from pathlib import Path
from datetime import datetime
from typing import Set, Optional

from main import ConfigLoader, StonefishScenarioBuilder


class FileWatcher:
    """Watches files for modifications and triggers conversion"""

    def __init__(self, blend_file: Path, config_file: Path, output_file: Path):
        self.blend_file = blend_file
        self.config_file = config_file
        self.output_file = output_file

        # Track last modification times
        self.last_mod_times = {}
        self._update_mod_times()

        # Track if this is first run
        self.first_run = True

    def _update_mod_times(self):
        """Update modification times for watched files"""
        for filepath in [self.blend_file, self.config_file]:
            if filepath.exists():
                self.last_mod_times[filepath] = filepath.stat().st_mtime

    def _check_for_changes(self) -> Set[Path]:
        """Check if any watched files have changed"""
        changed_files = set()

        for filepath in [self.blend_file, self.config_file]:
            if not filepath.exists():
                continue

            current_mtime = filepath.stat().st_mtime
            last_mtime = self.last_mod_times.get(filepath, 0)

            if current_mtime > last_mtime:
                changed_files.add(filepath)
                self.last_mod_times[filepath] = current_mtime

        return changed_files

    def _convert(self, changed_files: Optional[Set[Path]] = None):
        """Run the conversion"""
        try:
            timestamp = datetime.now().strftime("%H:%M:%S")

            if self.first_run:
                print(f"\n[{timestamp}] 🚀 Starting initial conversion...")
                self.first_run = False
            else:
                changed_names = [f.name for f in changed_files] if changed_files else []
                print(f"\n[{timestamp}] 🔄 Change detected in: {', '.join(changed_names)}")
                print(f"[{timestamp}] Regenerating scenario...")

            # Load config and convert
            config = ConfigLoader(str(self.config_file))
            builder = StonefishScenarioBuilder(config)

            output_path = builder.convert(str(self.blend_file), self.output_file)

            timestamp = datetime.now().strftime("%H:%M:%S")
            print(f"[{timestamp}] ✓ Successfully wrote: {output_path}")

        except FileNotFoundError as e:
            timestamp = datetime.now().strftime("%H:%M:%S")
            print(f"[{timestamp}] ✗ Error: File not found - {e}")
        except Exception as e:
            timestamp = datetime.now().strftime("%H:%M:%S")
            print(f"[{timestamp}] ✗ Error during conversion: {e}")
            import traceback
            traceback.print_exc()

    def watch(self, interval: float = 1.0):
        """
        Watch files and trigger conversion on changes

        Args:
            interval: Check interval in seconds
        """
        print("=" * 80)
        print("🔍 Stonefish Converter Daemon")
        print("=" * 80)
        print(f"Watching:")
        print(f"  📦 Blender file: {self.blend_file}")
        print(f"  ⚙️  Config file:  {self.config_file}")
        print(f"  📄 Output file:  {self.output_file}")
        print(f"\nCheck interval: {interval}s")
        print("Press Ctrl+C to stop")
        print("=" * 80)

        # Do initial conversion
        self._convert()

        try:
            while True:
                time.sleep(interval)

                changed_files = self._check_for_changes()

                if changed_files:
                    self._convert(changed_files)

        except KeyboardInterrupt:
            timestamp = datetime.now().strftime("%H:%M:%S")
            print(f"\n[{timestamp}] 👋 Stopping file watcher. Goodbye!")
            sys.exit(0)


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='Watch Blender and config files, auto-regenerate Stonefish scenarios',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Watch with default 1 second interval
  python3 watch_and_convert.py scene.blend config_new.yaml output.scn

  # Watch with custom interval (2 seconds)
  python3 watch_and_convert.py scene.blend config_new.yaml output.scn -i 2.0
        """
    )

    parser.add_argument('blend_file', help='Blender .blend file to watch')
    parser.add_argument('config_file', help='Configuration YAML file to watch')
    parser.add_argument('output_file', help='Output .scn file path')
    parser.add_argument(
        '-i', '--interval',
        type=float,
        default=1.0,
        help='Check interval in seconds (default: 1.0)'
    )

    args = parser.parse_args()

    # Validate inputs
    blend_path = Path(args.blend_file)
    config_path = Path(args.config_file)
    output_path = Path(args.output_file)

    if not blend_path.exists():
        print(f"✗ Error: Blender file not found: {blend_path}")
        sys.exit(1)

    if not config_path.exists():
        print(f"✗ Error: Config file not found: {config_path}")
        sys.exit(1)

    if args.interval <= 0:
        print(f"✗ Error: Interval must be positive, got: {args.interval}")
        sys.exit(1)

    # Create and start watcher
    watcher = FileWatcher(blend_path, config_path, output_path)
    watcher.watch(interval=args.interval)


if __name__ == "__main__":
    main()
