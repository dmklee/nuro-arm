# install conda
1. Download the Python 3.8 64-Bit Graphical Installer from this [link](https://www.anaconda.com/products/individual)
2. Open downloaded file and click thru the Installer, selecting the default options.

# install git
1. Open Anaconda Prompt application and enter the following command, enter y when prompted.
	```
	conda install -c anaconda git
	```

# Download Robot API
1. In Anaconda Prompt, download the repository
	```
	git clone https://github.com/dmklee/neu-ro-arm.git
	```
2. Change directories to the repository directory
	```
	cd neu-ro-arm/
	```
3. Create virtual environment by entering the following command
	```
	conda env create -f environment_win.yml
	```
4. Install the Robot API so it exists in python path.
	```
	pip install .
	```
5. Add libraries for handling usb commands:
	1. Download this [zip file])https://github.com/libusb/hidapi/releases/download/hidapi-0.10.1/hidapi-win.zip)
	2. In File Explorer, navigate into downloaded folder {your downloads location}\hidapi-win\x64
	3. Copy the three files ("hidapi.dll","hidapi.lib","hidapi.pdb")
	4. Paste them in "C:\Users\[username]\Anaconda3\envs\robot\"

# [Optional] Install IDE
1. Download Visual Studio Code Windows-version from this [link](https://code.visualstudio.com/Download)
2. Check out the brief introduction [here](https://code.visualstudio.com/docs) to learn more.
3. [Optional] Download the following extensions which will help improve your experience writing Python scripts.


# [Recommended] Install Visual Studio Code
1. Download Visual Studio Code Windows-version from this [link](https://code.visualstudio.com/Download)
2. Check out the brief introduction [here](https://code.visualstudio.com/docs) to learn more.
3. [Optional] Download the following extensions which will help improve your experience writing Python scripts.

