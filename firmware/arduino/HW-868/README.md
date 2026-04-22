# HW-868 RFID Sample

This folder contains a basic Arduino sample for an `HW-868` / `MFRC522`-style 13.56 MHz RFID reader.

The sample file is:

- [HW-868.ino](F:\李仁杰\香港大学\2025-2026\ELEC3848\Group project\firmware\arduino\HW-868\HW-868.ino)

## What This Code Does

This sketch polls for a nearby RFID card, reads its card type and UID, and prints the result to the serial monitor.

When a card is detected, it also:

- turns on an LED
- activates a buzzer
- prints the card UID
- compares the UID against two hard-coded examples
- prints a greeting based on the detected UID

The sample is written in a low-level style. It does not use the common `MFRC522` Arduino library. Instead, it directly talks to the RFID chip registers through SPI and implements functions such as:

- reader initialization
- card request
- anti-collision / UID read
- CRC calculation
- card halt

This makes it useful as a reference if you want to understand the reader protocol more deeply or later adapt the code into your own project firmware.

## Main Runtime Flow

In `loop()`, the sketch does the following:

1. Calls `MFRC522_Request(PICC_REQIDL, str)` to check whether a card is present.
2. If a card is found, prints the card type with `ShowCardType(str)`.
3. Calls `MFRC522_Anticoll(str)` to get the card UID.
4. Copies the UID into `serNum`.
5. Prints the UID with `ShowCardID(serNum)`.
6. Compares the first 4 UID bytes against two hard-coded IDs:
   - `4B E6 D1 3B` -> `Hello Mary!`
   - `3B E6 D1 3B` -> `Hello Greg!`
7. Calls `MFRC522_Halt()` to put the card into halt state.

## Pin Mapping Used In This Sample

Important:

- the pin definitions in `HW-868.ino` are not the confirmed pin mapping for this project
- they are the seller's sample-code pin definitions
- you should not assume these pins match the actual wiring on your Arduino Mega or AMR hardware

The sketch defines these pins:

- `chipSelectPin = 10`
- `NRSTPD = 5`
- `led1 = 2`
- `led2 = 3`
- `led3 = 4`
- `BUZZ = 6`

The RFID communication itself uses SPI, so the board's hardware SPI pins are also required.

For an Arduino Uno, that usually means:

- `MOSI = 11`
- `MISO = 12`
- `SCK = 13`
- `SS / SDA = 10`

You should confirm the actual `HW-868` module labels before wiring.
You should also remap the pins according to your real project wiring before integrating this code into proposal firmware.

## Serial Output

The sketch starts serial communication with:

```cpp
Serial.begin(57600);
```

So the Serial Monitor should be set to `57600 baud`.

Typical output looks like:

```text
Card type: MFOne-S50
The card's number is: F915B7B8
Hello unkown guy!
```

Note:

- the sample prints `unkown` instead of `unknown`
- the UID is printed as 4 bytes in hex

## Important Functions

The most important functions in this file are:

- `MFRC522_Init()`
  - Initializes the MFRC522 chip and turns on the antenna.
- `MFRC522_Request()`
  - Checks whether a card is in the field and returns the card type.
- `MFRC522_Anticoll()`
  - Reads the UID and performs XOR checksum validation.
- `MFRC522_ToCard()`
  - Core low-level function that sends commands/data to the card and reads the response back.
- `ShowCardType()`
  - Converts the returned card type bytes into readable text.
- `ShowCardID()`
  - Prints the UID in hex format.
- `MFRC522_Halt()`
  - Puts the current card into halt state after reading.

## Limitations Of This Sample

This is a simple demo and not yet a final project-ready RFID module. Current limitations include:

- no Arduino library abstraction
- no structured station-ID mapping for the AMR workflow
- no integration with the project telemetry JSON
- no error recovery beyond basic status checks
- no real proposal state-machine integration
- no real station verification logic beyond UID comparison with hard-coded examples

Also, while the file includes `MFRC522_Write()`, the current `loop()` does not use it. The sketch is primarily a card-detection and UID-reading demo.

## How This Can Be Reused In The Project

For the current AMR project, this sample is most useful as a reference for the future `station verification` module.

A practical next step would be:

1. keep the card read path
2. replace the hard-coded `Mary / Greg` logic with station names such as `PICKUP_A`, `DELIVERY_B`, etc.
3. output the detected UID and station result into the proposal telemetry JSON
4. use the match result to control whether the robot may continue pickup or delivery

## Suggested Improvements Before Integration

- rename pins and variables to project-specific names
- add comments for the actual wiring used on Arduino Mega
- define a clear mapping table from UID to station name
- expose RFID result in the same telemetry schema used by proposal firmware
- convert the raw sample into a reusable helper module or into a dedicated test sketch under `proposal_amr`

## Summary

`HW-868.ino` is a working low-level RFID sample for detecting MFRC522-compatible cards, reading card type and UID, and triggering simple serial / buzzer / LED feedback.

It is a good hardware bring-up reference, but it is not yet the final RFID verification implementation for the AMR workflow.
