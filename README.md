# tiny41
Raspberry Pico implementation of a HP41 bus analyzer

export PICO_SDK_PATH=/home/EU/sefangeth/pico-sdk

$ cd build/
$ cmake .. -DPICO_BOARD=pimoroni_tiny2040
$ cmake .. -DPICO_BOARD=pimoroni_picolipo_16mb
$ cmake .. -DPICO_BOARD=pico2
$ make
$ cp tiny41.uf2 /media/thomas/RPI-RP2
$ cp tiny41.uf2 /media/sefangeth/RP2350/