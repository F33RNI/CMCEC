# CMCEC
## Cheap 2 channel MIDI to CV converter on 2x12bit R2R DAC with supply voltage compensation, analog portamento and optical isolation

<div style="width:100%;text-align:center;">
    <p align="center">
        <a href="https://twitter.com/f33rni"><img alt="Twitter" src="https://img.shields.io/twitter/url?label=My%20twitter&style=social&url=https%3A%2F%2Ftwitter.com%2Ff33rni" ></a>
        <img src="https://badges.frapsoft.com/os/v1/open-source.png?v=103" >
        <a href="https://soundcloud.com/f3rni"><img alt="SoundCloud" src="https://img.shields.io/badge/-SoundCloud-orange" ></a>
    </p>
</div>
<div style="width:100%;text-align:center;">
    <p align="center">
        <img src="IMAGES/SMCEC_FRONT.jpg" width="auto" height="300">
        <img src="IMAGES/SMCEC_STANDALONE.jpg" width="auto" height="300">
        <img src="IMAGES/SMCEC_BACK.jpg" width="auto" height="300">
    </p>
</div>

----------

### Schematic

PDF-version available at the root of the repo

<div style="width:100%;text-align:center;">
    <p align="center">
        <img src="IMAGES/SCHEMATIC.png" width="100%" height="auto">
    </p>
</div>

----------

### R2R DAC

To make this project as cheap as possible I decided to use DIY R2R DAC based on 74HC595 shift registers instead of buying one

<div style="width:100%;text-align:center;">
    <p align="center">
        <img src="IMAGES/DAC_FRONT.jpg" width="auto" height="300">
        <img src="IMAGES/DAC_BACK.jpg" width="auto" height="300">
    </p>
</div>

----------

### Calibration

CMCEC features automatic voltage compensation by measuring it internally. In order to use this feature as well as setup 1V/octave you need to perform calibration

1. Uncomment `//#define CALIBRATION 1` line and set it to the `#define CALIBRATION 0`
2. Connect Arduino to PC or UART at 9600. Follow the instructions
3. Uncomment `//#define CALIBRATION 1` line and set it to the `#define CALIBRATION 1`
4. Connect Arduino to PC or UART at 9600. Follow the instructions. Use Excel file in the root of the repo to simplify gain calibration
5. Comment `//#define CALIBRATION 1` line
