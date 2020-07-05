# Virtual-Firmware-grSim Dependencies Install

### System Minimum Requirements

Linux, recommended enviroment: Ubuntu 18.04 or above

- Dual Core CPU (2.0 Ghz+)
- 1GB of RAM
- 256MB nVidia or ATI graphics card





### Software to Install

* grSim with customized setting:
* google protobuffer library
* c++ armadillo library 
* c++ boost library



##### grSim

1. clone this repo in your preferred location https://github.com/RoboCup-SSL/grSim
2. Follow install instructions in https://github.com/RoboCup-SSL/grSim/blob/master/INSTALL.md
3. Navigate into grSim/bin, run ./grSim   to open grSim 
4. In the Configuration window on the upper left side, under Game/Division, set it to 'Division B'
5. Under Game/Robots Count, set it to 6
6. Under "Geometry/Blue Team" and "Geometry/Yellow Team", you can pick a particular team whose configuration files is located in grSim/config/*.ini
7. Add our own configuration file https://github.com/IEEE-UCSD-RoboCup-2020/Virtual-Firmware-grSim/Triton.ini to "grSim/config/", then navigate back to "grSim/" and run make     





##### protobuffer 

1. install latest release from https://github.com/protocolbuffers/protobuf/releases/tag/v3.12.3
2. unzip and keep the protobuf source code in your preferred location
3. Open the README.md file and follow the install instructions there
4. Follow the instructions for C++
5. This might take a while



##### C++ Armadillo library

Download the latest version and follow the README.md

http://arma.sourceforge.net/download.html

In the linux installation section in README.md:

* ignore the first 2 steps if your already have gcc and cmake installed

* in step3 in README.md, "Ensure LAPACK and BLAS (or preferably OpenBLAS) are installed on your system." use the following

  * ```bash
    sudo apt install cmake libopenblas-dev liblapack-dev libarpack2-dev libsuperlu-dev
    ```

* Then for step4 ~ 6, cd into the armadillo directory, run the following

  * ```bash
    cmake .
    ```

  * ```bash
    make	
    ```

  * ```bash
    sudo make install		
    ```



##### C++ boost library

```bash
sudo apt-get install libboost-all-dev
```

