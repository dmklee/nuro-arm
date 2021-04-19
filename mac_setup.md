# install conda
1. Download the Python 3.8 64-Bit Graphical Installer from this [link](https://www.anaconda.com/products/individual)
2. Open downloaded file and click thru the Installer, selecting the default options.

# install git
1. Open terminal application and enter the following command
	```
	conda install -c anaconda git
	```

# Install API and create environment
1. Open terminal application and enter the following command
	```
	git clone https://github.com/dmklee/neu-ro-arm.git
	```
2. Move into the repository directory:
	```
	cd neu-ro-arm/
	```
3. Create the virtual environment
	```
	conda env create -f environment_mac.yml
	```

[optional] conda update -n base defaults conda
conda create -n robot python=3.8 anaconda
conda activate robot
cd neu-ro-arm/
pip install .
?brew install hidapi
?brew install libusb

# install vs code
1. Download VS Code for Mac by clicking this [link](https://code.visualstudio.com/download)
2. [Optional] With VS Code open, click the Extensions icon on the left panel.  Under the popular tab, click Install on the Python extension.
