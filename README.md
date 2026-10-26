# Eco-Marathon driver information system

Proposal for the Imperial Eco-Marathon electronics task: a self-contained dash for the battery-electric prototype, powered by its own 18650 cell and with no connection to the propulsion electronics.

- `index.html`: the proposal (static site, deployed on Vercel)
- `firmware/dis_firmware/`: Arduino sketch for the ESP32-S3 (`config.h` holds the race, wheel and strategy assumptions)
- `test/speed_logic_test.py`: bench check of the speed decoding and pace logic (`python3 test/speed_logic_test.py`)
