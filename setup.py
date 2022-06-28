import platform
from setuptools import setup, find_packages

with open("README.md", 'r') as f:
    long_description = f.read()

setup(
    name='nuro-arm',
    version='0.0.5',
    description='Simple control interface for low-cost robotic arm.',
    license="MIT License",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author='David Klee',
    author_email='klee.d@northeastern.edu',
    url='https://github.com/dmklee/nuro-arm',
    packages=find_packages(),
    include_package_data=True,
    python_requires='>3.6.0',
    setup_requires='wheel',
    install_requires=[
        "numpy",
        "scipy",
        "pillow",
        "pybullet>=3.1.7",
        "easyhid; platform_system=='Linux'",
        "hid; platform_system=='Windows'",
        "hidapi; platform_system=='Darwin'",
    ],
    extras_require={
        'all': ['opencv-contrib-python']
    },
    entry_points={
        'console_scripts': [
            'calibrate_xarm=nuro_arm.scripts.calibrate_xarm:main',
            'calibrate_camera=nuro_arm.scripts.calibrate_camera:main',
            'move_arm_with_gui=nuro_arm.scripts.move_arm_with_gui:main',
            'record_movements=nuro_arm.scripts.record_movements:main',
            'generate_aruco_tags=nuro_arm.scripts.generate_aruco_tags:main',
        ]
    },
    keywords=[
        'robotics',
        'educational-project',
        'low-cost-robot',
        'robotic-manipulation',
    ],
)
