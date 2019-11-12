# ESP32 apps

<p>
  <a href="https://travis-ci.com/alxhoff/esp32-applications">
  <img src="https://travis-ci.com/alxhoff/esp32-applications.svg?branch=master">
  </a>
  <a href="https://github.com/alxhoff/esp32-applications/blob/master/LICENSE">
    <img src="https://img.shields.io/badge/license-GPLv3-blue.svg" />
  </a>
</p>

Playing around with the ESP32

## Applications

### Housemate scanner - Work in progress

To hopefully solve the problem of knowing who's the last person to leave in the morning (to lock the door) by using an APR scanner style app to track phone MACs to show who is still hiding away in their rooms.

## Toolchain Setup

### Prerequisites

`sudo pacman -S --needed gcc git make ncurses flex bison gperf python2-pip python2-pyserial python2-click python2-cryptography python2-future python2-pyparsing python2-pyelftools cmake ninja ccache`

*python2-pyelftools appears to only be available through AUR*

`yaourt python2-pyelftools`

### Toolchain from AUR

```
yaourt gcc-xtensa-esp32-elf-bin
yaourt openocd-esp32
```

### IDF install

Clone the idf repo into somewhere useful

``` bash
mkdir ~/esp
cd ~/esp
git clone  --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
```
Make sure system python is set to python2.7

```python --version```

if not then symlink py2.7 to system python2

```
sudo ln -sf /usr/bin/python2.7 /usr/bin/python
```
*Afterwards this should be revered by symlinking python3.7 back to system python*

Install python requirements from ISF requirements document

``` python
python2.7 -m pip install --user -r $HOME/esp/esp-idf/requirements.txt
```

If you want to install the toolchain from the IDF (not the AUR) then run.

```
./install
```

### Environment Variables

Your system needs the IDF_PATH in its IDF_PATH

```
export IDF_PATH=~/esp/esp-idf
```

or add it to your ~/.bashrc

```
echo "export IDF_PATH=$HOME/esp/esp-idf" >> $HOME/.bashrc
source $HOME/.bashrc
```

### Test build

Enter a simple example and test a build

```
cd $HOME/esp/esp-idf/examples/wifi/scan
make menuconfig #configure SDK options
make flash monitor #with device plugged in (should appear as /dev/ttyUSB0)
```
