# install arduino deps

pio lib install DFRobotDFPlayerMini

# create arduiono proj.

mkdir proj_dir
cd proj_dir
platformio init --ide clion --board uno
    // evtl. toolchain uno adden  