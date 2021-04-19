## Table of Contents
1. [Software](#software)
2. [Hardware](#hardware)

<a name="software"></a>
## Software (estimated time 15 min)

### Install Anaconda
1. Download the Python 3.8 64-Bit Graphical Installer from this [link](https://www.anaconda.com/products/individual)
2. Open downloaded file and click thru the Installer, selecting the default options.

### Install Git
1. Open a terminal application (for Mac it is called Terminal, for Windows it is Anaconda Prompt)
2. Enter the following command and hit enter:
	```
	conda install -c anaconda git
	```

### Install API and Create Virtual Environment
1. Open terminal application and enter the following command:
	```
	git clone https://github.com/dmklee/neu-ro-arm.git
	```
2. Move into the repository directory with this command:
	```
	cd neu-ro-arm/
	```
3. Create the virtual environment:
	```
	conda env create -f environment_mac.yml
	```
4. Install Robot API so it exists in Python path:
	```
	pip install .
	```
5. [Windows only] Add libraries for handling usb commands:
	1. Download this [zip file])https://github.com/libusb/hidapi/releases/download/hidapi-0.10.1/hidapi-win.zip)
	2. In File Explorer, navigate into downloaded folder {your downloads location}\hidapi-win\x64
	3. Copy the three files ("hidapi.dll","hidapi.lib","hidapi.pdb")
	4. Paste them in "C:\Users\[username]\Anaconda3\envs\robot\"

### [Recommended] Install Visual Studio Code
1. Download VS Code for your device by clicking this [link](https://code.visualstudio.com/download)
2. With VS Code open, click the Extensions icon on the left panel.  Under the popular tab, click Install on the Python extension.

<a name="hardware"></a>
## Hardware
### Robot
#### Assembly Instructions
#### Calibration

### Camera
#### Assembly Instructions
#### Calibration

### Cubes


