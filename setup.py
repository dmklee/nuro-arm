from setuptools import setup

setup(
    name='neu-ro-arm',
    version='0.0.1',
    description='NEU Robotics Outreach: Learning Robotic Manipulation',
    author='David Klee',
    author_email='klee.d@northeastern.edu',
    url='https://github.com/dmklee/neu-ro-arm',
    packages=['neu_ro_arm'],
    install_requires=['pybullet', 'numpy', 'gym']
)
