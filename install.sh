#!/bin/bash
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
	conda env create -n robot python=3.8.5
	conda activate robot

	conda install -c conda-forge pybullet
	conda install -c conda-forge matplotlib
	pip install opencv-contrib-python
	pip install easyhid

	sudo apt-get install libhidapi-hidraw0 libhidapi-libusb0

elif [[ "$OSTYPE" == "darwin"* ]]; then
	conda env create -n robot python=3.8.5
	conda activate robot

	conda install -c conda-forge pybullet
	conda install -c conda-forge matplotlib
	pip install opencv-contrib-python
	pip install hidapi

elif [[ "$OSTYPE" == "win32" ]]; then
	conda env create -n robot python=3.8.5
	conda activate robot

	conda install -c conda-forge pybullet
	conda install -c conda-forge matplotlib
	pip install opencv-contrib-python
	pip install hid

	curl -LO https://github.com/libusb/hidapi/releases/download/hidapi-0.10.1/hidapi-win.zip --silent
	unzip hidapi-win.zip

	python_file=(where python)
	python_path=(dirname "${python_file}")
	mv hidapi-win/x64/* ${python_path}
