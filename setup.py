from setuptools import setup, find_packages

setup(
    name='neu-ro-arm',
    version='0.0.1',
    description='NEU Robotics Outreach: Learning Robotic Manipulation',
    author='David Klee',
    author_email='klee.d@northeastern.edu',
    url='https://github.com/dmklee/neu-ro-arm',
    packages=find_packages(),
    python_requires='>3.6.0',
    setup_requires="wheel",
    install_requires=[
        "numpy",
        "opencv-contrib-python",
        "pybullet==3.1.7",
        "matplotlib",
        "easyhid;platform_system=='Linux'",
        "hid;platform_system=='Windows'",
        "sklearn",
        "pyyaml",
        "scipy",
        # "pywin32 >= 1.0;platform_system=='Darwin'"
    ],
)
