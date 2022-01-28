from setuptools import setup, find_packages

setup(
    name='nuro-arm',
    version='0.0.1',
    description='NU Robotics Outreach: Learning Robotic Manipulation',
    author='David Klee',
    author_email='klee.d@northeastern.edu',
    url='https://github.com/dmklee/nuro-arm',
    packages=find_packages(),
    include_package_data=True,
    python_requires='>3.6.0',
    setup_requires="wheel",
    install_requires=[
        "numpy",
        "opencv-contrib-python",
        "pybullet>=3.1.7",
        "matplotlib",
        "easyhid;platform_system=='Linux'",
        "hid;platform_system=='Windows'",
        "sklearn",
        "scipy",
        "gym",
    ],
)
