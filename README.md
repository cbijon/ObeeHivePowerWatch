# Beehive Protection System with Electrified Harps

## Overview

This repository contains the Arduino sketch for a beehive protection system with electrified harps. The system is designed to protect beehives from Asian hornets by centralizing control and communication with multiple satellite boards. It uses ESP32 microcontrollers, LoRa communication, and ESP-NOW Wi-Fi for coordination and data transmission. Satellite code are here : https://github.com/cbijon/ObeeHarpe

## Key Features

- Centralized control of electrified harps for beehive protection.
- Communication with multiple satellite boards over ESP-NOW Wi-Fi.
- Data transmission to a private TTN (The Things Network) using LoRaWAN.
- Monitoring of various environmental factors, such as light and rain conditions.
- Relay control for power management.
- Alarm and alert mechanisms to notify the user of system status.

## Code Structure

The code is organized as follows:

- **Libraries:** The code utilizes various libraries, including LMIC and ESP-NOW, to facilitate LoRa and Wi-Fi communication.

- **Configuration:** You can configure various constants and parameters, such as Wi-Fi channel, LoRa settings, data transmission intervals, and alarm thresholds in the sketch.

- **Data Structures:** The code defines data structures to store information about satellite boards and LoRa transmission payloads.

- **Setup Functions:** The setup functions configure Wi-Fi (ESP-NOW), LoRa, GPIO pins, and initialize the system.

- **Event Handling:** The `onEvent` function manages LoRa-related events and provides event handling for various scenarios.

- **Buzzer Functions:** The code includes functions to produce sound from a buzzer to indicate different system events.

- **Main Loop:** The primary loop schedules LoRa data transmissions and handles LoRa communication.

- **Additional Functions:** There are several additional functions to manage the state of satellite boards, control relay states, prepare LoRa payload data, and monitor system components.

## Getting Started

1. **Hardware Requirements:** Ensure you have the required hardware components, including ESP32 microcontrollers, sensors, relays, and a buzzer.

2. **Installation:** Upload the Arduino sketch to your ESP32 devices (TTGO Lora32 without OLED) using the Arduino IDE or your preferred development environment.

3. **Configuration:** Modify the code to match your specific configuration, such as Wi-Fi credentials, LoRa keys, and GPIO pin assignments.

4. **Deployment:** Deploy the central unit and satellites boards in your beehive protection system.

5. **Monitoring:** Monitor the system status and data transmissions using the provided code and hardware components.

## License

This project is open-source and distributed under the [MIT License](LICENSE). You are welcome to use and modify the code for your own beehive protection system.

## Author

- [Charles Bijon](mailto:bijon.charles@gmail.com)

## Acknowledgments

We thank the open-source community for their contributions and support in developing this beehive protection system.

