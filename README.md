<!--
*** Thanks for checking out this README Template. If you have a suggestion that would
*** make this better, please fork the repo and create a pull request or simply open
*** an issue with the tag "enhancement".
*** Thanks again! Now go create something AMAZING! :D
***
***
***
*** To avoid retyping too much info. Do a search and replace for the following:
*** github_username, repo, twitter_handle, email
-->





<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]




<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/dianlight/a8i2cGateway">
    <!--
    <img src="images/logo.png" alt="Logo" width="80" height="80">
    -->
  </a>

  <h3 align="center">AT8 I2C Gateway</h3>

  <p align="center">
    An active I2C port extender based on an ATMEGA8 chip
    <br />
    <!--
    <a href="https://github.com/dianlight/a8i2cGateway"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/dianlight/a8i2cGateway">View Demo</a>
    ·
    -->
    <a href="https://github.com/dianlight/a8i2cGateway/issues">a8i2cGatewayrt Bug</a>
    ·
    <a href="https://github.com/dianlight/a8i2cGateway/issues">Request Feature</a>
  </p>
</p>

<!-- Donations -->

<a href="https://www.buymeacoffee.com/ypKZ2I0" target="_blank"><img src="https://cdn.buymeacoffee.com/buttons/default-orange.png" alt="Buy Me A Coffee" style="height: 51px !important;width: 217px !important;" ></a>


<!-- TABLE OF CONTENTS -->
## Table of Contents

- [Table of Contents](#table-of-contents)
- [About The Project](#about-the-project)
  - [Built With](#built-with)
- [Getting Started](#getting-started)
  - [Hardware Prerequisites](#hardware-prerequisites)
  - [Software Prerequisites](#software-prerequisites)
  - [Installation](#installation)
- [Supported Sensors](#supported-sensors)
- [Config.h options](#configh-options)
  - [Debug setting](#debug-setting)
    - [`I2CADDRESS`](#i2caddress)
    - [`DEBUG_I2C_OUT` (Optional)](#debugi2cout-optional)
    - [`DEBUG_I2C_IN`  (Oprional)](#debugi2cin-oprional)
    - [`DEBUG_EVENT` (Optional)](#debugevent-optional)
    - [`HAS_DHT` (Optional)](#hasdht-optional)
      - [`DHTPIN` (Only if HAS_DHT is enabled)](#dhtpin-only-if-hasdht-is-enabled)
      - [`TEMP_COUNT` (Only if HAS_DHT is enabled)](#tempcount-only-if-hasdht-is-enabled)
      - [`FIXED_CORRECTION` (Only if HAS_DHT is enabled)](#fixedcorrection-only-if-hasdht-is-enabled)
    - [`HAS_RELAY` (Optional)](#hasrelay-optional)
      - [`RELAYPIN` (Only if HAS_RELAY is enabled)](#relaypin-only-if-hasrelay-is-enabled)
    - [`HAS_ROTARY_ENCODER` (Optional)](#hasrotaryencoder-optional)
      - [`ENCODERSWPIN` (Only if HAS_ROTARY_ENCODER is enabled)](#encoderswpin-only-if-hasrotaryencoder-is-enabled)
      - [`ENCODERCLKPIN` (Only if HAS_ROTARY_ENCODER is enabled)](#encoderclkpin-only-if-hasrotaryencoder-is-enabled)
      - [`ENCODERDTPIN` (Only if HAS_ROTARY_ENCODER is enabled)](#encoderdtpin-only-if-hasrotaryencoder-is-enabled)
      - [`STEPS` (Only if HAS_ROTARY_ENCODER is enabled)](#steps-only-if-hasrotaryencoder-is-enabled)
- [Roadmap](#roadmap)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)
- [Acknowledgements](#acknowledgements)



<!-- ABOUT THE PROJECT -->
## About The Project

<!-- [![Product Name Screen Shot][product-screenshot]](https://example.com) -->
One of the things that always happens to me when I start a new project is that the MCU I have chosen or the package does not have all the digital and analog ports that I would like.

The now historic ATMEGA8 has a few kb but a lot of GPIO. So why not use it?

This project is a customizable firmware to allow the display of sensors connected to an ATMEGA8 on an i2c / twi bus which acts as an i2c slave.

***The project was initially born as a subproject of [SmartTemp](https://github.com/dianlight/SmartTemp) an open source smart thermostat*** 

### Built With

* [Platform IO]()
* [Arduino IDE]()


<!-- GETTING STARTED -->
## Getting Started

To get a local copy up and running follow these simple steps.

### Hardware Prerequisites

1. ATMEGA8 chip
2. White Broadband
3. An SPI/ASP programmer

### Software Prerequisites

1. Arduino IDE ( to install the bootmanager on the chip )

2. Platform IO ( to compile and generate the firmware )

### Installation
 
1. Burn the bootloader into the ATMEGA8 with Arduino IDE and the SPI/ASP programmer
2. Clone the a8i2cGateway
```sh
git clone https://github.com/dianlight/a8i2cGateway.git
```
3. Open the project with Platform IO
4. Configure the firmware 
```src/include/Config.h```
5. Build and flash your firmware

## Supported Sensors
* DHT11/22: ***Temperature and Humidity read froma a DHT sensor*** [DHTStable by RobTillaart]([DHTStable](https://github.com/RobTillaart/Arduino/tree/master/libraries/DHTstable)) _(MIT License)_
* ClickEncoder: ***Rotary Encodes with buttons library*** [Encoder by Soligen2010](https://github.com/soligen2010/encoder) _(custom License)_
* Single pin relay
  

<!-- USAGE EXAMPLES -->
## Config.h options

### Debug setting
Be carefull because can be hungry of flash and ram

#### `I2CADDRESS` 
Address of the ATMEGA on I2C BUS

#### `DEBUG_I2C_OUT` (Optional)
Display on Serial all message to I2C bus

#### `DEBUG_I2C_IN`  (Oprional)
 Display on Serial all message from I2C bus

#### `DEBUG_EVENT` (Optional)
 Display on Serial all message to I2C bus

#### `HAS_DHT` (Optional)
 A DHT sensor is connected to the ATMEGA
 ##### `DHTPIN` (Only if HAS_DHT is enabled)
 The Digital pin to read the Temp and Humidity information

 ##### `TEMP_COUNT` (Only if HAS_DHT is enabled)
 The numeber of read to merge in avarage count. 
 To eliminate temp fluctuation in reading on the I2C bus is send the avg math of the last `TEMP_COUNT` reads.

 ##### `FIXED_CORRECTION` (Only if HAS_DHT is enabled)
 Some Chinese DHT sensor are not calibrated right and has a fixed up or down error. The value of `FIXED_CORRECTION` is added to readed temp.

#### `HAS_RELAY` (Optional)
 A RELAY is connected to a AT8MEGA's pin.
 ##### `RELAYPIN` (Only if HAS_RELAY is enabled)
 The Digital pin to control the Relay

#### `HAS_ROTARY_ENCODER` (Optional)
 A rotary encoder with button is connected to AT8MEGA
 ##### `ENCODERSWPIN` (Only if HAS_ROTARY_ENCODER is enabled)
 The Digital pin to control the encoder SW ( Button pin )
 ##### `ENCODERCLKPIN` (Only if HAS_ROTARY_ENCODER is enabled)
 The Digital pin to control the encoder CLK ( A pin )
 ##### `ENCODERDTPIN` (Only if HAS_ROTARY_ENCODER is enabled)
 The Digital pin to control the encoder DT ( B pin )
 ##### `STEPS` (Only if HAS_ROTARY_ENCODER is enabled)
 How many steps are send from the encoder for 1 tic.

<!-- ROADMAP -->
## Roadmap

See the [open issues](https://github.com/dianlight/a8i2cGateway/issues) for a list of proposed features (and known issues).



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request



<!-- LICENSE -->
## License

Distributed under the GPL3 License. See `LICENSE` for more information.



<!-- CONTACT -->
## Contact

<!--
Your Name - [@twitter_handle](https://twitter.com/twitter_handle) - email
-->

Project Link: [https://github.com/dianlight/a8i2cGateway](https://github.com/dianlight/a8i2cGateway)



<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements

* [RobTillaart](https://github.com/RobTillaart/) for the Arduino libraries
* [soligen2010](https://github.com/soligen2010/) for the ClickEncoder fork







<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/dianlight/a8i2cGateway.svg?style=flat-square
[contributors-url]: https://github.com/dianlight/a8i2cGateway/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/dianlight/a8i2cGateway.svg?style=flat-square
[forks-url]: https://github.com/dianlight/a8i2cGateway/network/members
[stars-shield]: https://img.shields.io/github/stars/dianlight/a8i2cGateway.svg?style=flat-square
[stars-url]: https://github.com/dianlight/a8i2cGateway/stargazers
[issues-shield]: https://img.shields.io/github/issues/dianlight/a8i2cGateway.svg?style=flat-square
[issues-url]: https://github.com/dianlight/a8i2cGateway/issues
[license-shield]: https://img.shields.io/github/license/dianlight/a8i2cGateway.svg?style=flat-square
[license-url]: https://github.com/dianlight/a8i2cGateway/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=flat-square&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/lucio-tarantino-8ab9a3/
[product-screenshot]: images/screenshot.png
[buy-me-a-coffe]: https://www.buymeacoffee.com/ypKZ2I0