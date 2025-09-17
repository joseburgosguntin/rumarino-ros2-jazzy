import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'controller_stonefish'

def files_under(root_dir):
    # Yield (install_dir, [file_path]) pairs for every file under root_dir
    pairs = []
    for path in glob(os.path.join(root_dir, '**', '*'), recursive=True):
        if os.path.isfile(path):
            # Preserve subfolder structure under share/<pkg>/<root_dir>/...
            rel_dir = os.path.dirname(os.path.relpath(path, start='.'))
            # rel_dir like "data/maps" or "launch"
            install_dir = os.path.join('share', package_name, rel_dir)
            pairs.append((install_dir, [path]))
    return pairs

data_files = [
    ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
    ('share/' + package_name, ['package.xml']),
]

# Always include top-level launch files (and any nested ones)
data_files += files_under('launch')
# Include everything under data/ (recursively)
data_files += files_under('data')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ed',
    maintainer_email='edyancruz@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thruster_teleop = controller_stonefish.thruster_teleop:main',
        ],
    },
)
