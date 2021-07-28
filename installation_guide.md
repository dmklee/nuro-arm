# Software Installation Guide
## Table of Contents
1. [General Installation](#general)
2. [Windows](#windows)
2. [Linux](#linux)
3. [FAQ](#faq)

<a name="general"></a>
## General
1. Install pybullet with conda.
```
conda install -c conda-forge pybullet
```
2. Install the package

### Install API and Create Virtual Environment
1. Open terminal application and enter the following command:
	```
	git clone https://github.com/dmklee/nuro-arm.git
	```
2. Move into the repository directory with this command:
	```
	cd nuro-arm/
	```
3. Create the virtual environment:
    1. [**Windows Only**]
        ```
        conda env create -f environment_win.yml
        ```
    2. [**Mac Only**]
        ```
        conda env create -f environment_mac.yml
        ```
4. Activate the virtual environment:
	```
	conda activate robot
	```
4. Install Robot API so it exists in Python path:
	```
	pip install .
	```
5. [**Windows only**] Add libraries for handling usb-hid commands:
	1. Download this [zip file])https://github.com/libusb/hidapi/releases/download/hidapi-0.10.1/hidapi-win.zip)
	2. In File Explorer, navigate into downloaded folder {your downloads location}\hidapi-win\x64
	3. Copy the three files ("hidapi.dll","hidapi.lib","hidapi.pdb")
	4. Paste them in "C:\Users\[username]\Anaconda3\envs\robot\"

5. [Linux only (**not for Mac or Windows**)] Install libraries for usb-hid:
	1. In terminal, enter following command:
    ```
    sudo apt-get install libhidapi-hidraw0 libhidapi-libusb0
    ```
	2. You may need to run the following command so that the robot can be connected
	```
	sudo service fwupd stop
	```

