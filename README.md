# BzBox_ALPHA
Initial alpha-release of development firmware for BzBox PCBA

Save to new folder.

Build main_alpha.cpp using Particle CLI command 'particle compile photon . --saveTo firmware.bin'

Upload 'firmware.bin' to Photon using USB cable and DFU mode:

hold SETUP button on RESET until blinking YELLOW

Send 'particle flash --usb firmware.bin'

Cloud release coming soon...
