import platform
from setuptools import setup, find_packages

with open("README.md", 'r') as f:
    long_description = f.read()

# figure out what HID repo to use
hid_library = {'Linux' : 'easyhid',
               'Windows' : 'hid',
               'Darwin' : 'hidapi',
              }[platform.system()]

setup(
    name='nuro-arm',
    version='0.0.1',
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
    setup_requires="wheel",
    install_requires=[
        "numpy",
        "scipy",
        "pillow",
        "pybullet>=3.1.7",
        hid_library,
    ],

    keywords=[
        'robotics',
        'educational-project',
        'low-cost-robot',
        'robotic-manipulation',
    ],
)
