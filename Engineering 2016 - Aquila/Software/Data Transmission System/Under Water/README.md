Using I2C protocol to get underwater raw data to the above water Arduino and process the raw data to human readable data with some conversion in units.

Pressure Sensor

The pressure sensor is a relatively small and delicate device, and thus, during the soldering process, it requires definite precision. The pressure sensors after soldering is then waterproofed so that it can used underwater with full exposure. Our system uses I2C bus communication to interact with the pressure sensors and the data retrieved from the sensors is in mBar, in other words, 0.1 kilopascal. The main usage of the pressure sensor is to measure the depth of the ROV underwater. Our team has devised an equation (int)((pressure*100*1000)-101325)/(0.993*9.8) for calculating depth based on the equation pgh in fluid mechanics. The design of the system is that the pressure sensor will send data to the arduino underwater, then the data is processed into depth through the equation above and then send to the arduino above water through ethernet wire and softwareSerial protocol. The data will then display on the computer's monitor through an application we wrote in Processing, which reads the serial output of the data and display in an organized form on the screen.

Temperature Sensor

The temperature sensor we have chosen for our ROV is model DS18B20. We deliberately chose DS18B20 as our thermometer as it’s waterproof and it uses a Unique 1-Wire interface which requires only one port pin for communication. Our temperature sensor can accurately measures temperatures from –55°C to +125°C (–67°F to +257°F) with only ±0.5°C error.

