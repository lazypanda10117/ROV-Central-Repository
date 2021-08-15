# Description of the code

<br>

**Pressure Sensor**:

The pressure sensor is a relatively small and delicate device, and thus, during the soldering process, it requires definite precision. The pressure sensors after soldering is then waterproofed so that it can be used under water with full exposure. Our system uses an I2C bus communication protocol to interact with the pressure sensor. The data retrieved from the sensor is in mBar (0.1 kilopascal). The main usage of the pressure sensor is to measure the depth of the ROV while under water. Our team has devised an equation for calculating depth based on standard results in fluid mechanics.

<br>

**Temperature Sensor**:

The temperature sensor we have chosen for our ROV is model DS18B20. We deliberately chose DS18B20 as our thermometer because it is waterproof and it uses a unique 1-Wire interface, which requires only one port for communication. The temperature sensor can accurately measures temperatures from –55°C to +125°C (–67°F to +257°F) with only ±0.5°C error.

<br>

**Transmission of data**:

These sensors' data will first be sent to the Arduino under water. Then, it is processed before being sent to the Arduino above water through a CAT-6 ethernet cable. These data will then be displayed on our computer in a custom application we developed using Processing.
