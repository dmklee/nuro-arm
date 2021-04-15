from setuptools import setup, find_packages

setup(
    name='neu-ro-arm',
    version='0.0.1',
    description='NEU Robotics Outreach: Learning Robotic Manipulation',
    author='David Klee',
    author_email='klee.d@northeastern.edu',
    url='https://github.com/dmklee/neu-ro-arm',
    packages=find_packages(include=['neu_ro_arm']),
    python_requires='>3.6.0',
    setup_requires="wheel",
    install_requires=[
        "numpy",
        "opencv-contrib-python",
        "pybullet",
        "matplotlib",
        "easyhid;platform_system=='Linux'",
        "serial;platform_system=='Windows'",
        # "pywin32 >= 1.0;platform_system=='Darwin'"
    ],
)
