from setuptools import setup, find_packages

with open("README.md", 'r') as f:
    long_description = f.read()

setup(
    name='nuro-arm',
    version='0.0.1',
    description='NURO Arm: Accessible Robotics Educational Platform',
    license="MIT License",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author='David Klee',
    author_email='klee.d@northeastern.edu',
    url='https://github.com/dmklee/nuro-arm',
    packages=find_packages(),
    include_package_data=True,
    python_requires='>3.6.0',
    setup_requires="wheel",
    install_requires=[
        "numpy",
        "scipy",
        "pillow",
        "pybullet>=3.1.7",
        "easyhid;platform_system=='Linux'",
        "hid;platform_system=='Windows'",
        "hidapi;platform_system=='Darwin'",
    ],

    keywords=[
        'robotics',
        'educational-project',
        'low-cost-robot',
        'robotic-manipulation',
    ],
)
