===============================================================================
                           GPS DRIVER README
===============================================================================

Project Name  : GPS Driver for Quectel L89 GNSS Module
Author        : KHUSHI GAJJAR
Completed On  : 09-09-2026

Target MCU    : STM32L471VET6
Platform      : STM32 HAL Driver
Communication : UART DMA (Circular Mode)

===============================================================================
1. INTRODUCTION
===============================================================================

This driver provides an interface between an STM32 microcontroller and the
Quectel L89 GNSS receiver.

The driver uses UART DMA in Circular Mode to continuously receive NMEA
sentences without blocking the CPU.

Received NMEA data is validated using checksum verification before being
parsed into useful GPS information.

The driver currently supports:

    • Latitude
    • Longitude
    • Altitude
    • UTC Time
    • UTC Date
    • IST Time
    • IST Date
    • GPS Position Status

===============================================================================
2. HARDWARE USED
===============================================================================

Microcontroller
---------------
STMicroelectronics STM32L471VET6

GPS Module
----------
Quectel L89 GNSS Receiver

Communication Interface
-----------------------
UART DMA (Circular Mode)

===============================================================================
3. DRIVER FEATURES
===============================================================================

✓ UART DMA Circular Reception
✓ Continuous GPS Data Reception
✓ NMEA Checksum Verification
✓ Automatic NMEA Sentence Detection
✓ RMC Sentence Parsing
✓ GGA Sentence Parsing
✓ Latitude/Longitude Conversion
✓ UTC to IST Conversion
✓ Configurable Update Interval
✓ Optional Debug UART Output

===============================================================================
4. NMEA SENTENCES
===============================================================================

The Quectel L89 can output multiple NMEA sentence types.

This driver enables only the required sentences.

Enabled
-------

RMC
    Used for:
        • UTC Time
        • UTC Date
        • Latitude
        • Longitude
        • GPS Position Status

GGA
    Used for:
        • Altitude

Disabled
--------

GLL
GSA
GSV
VTG
ZDA
GRS
GST

Disabling unused sentences reduces UART traffic and CPU load.

===============================================================================
5. PAIR062 CONFIGURATION
===============================================================================

Index   Sentence     Status

0       GGA          Enabled
1       GLL          Disabled
2       GSA          Disabled
3       GSV          Disabled
4       RMC          Enabled
5       VTG          Disabled
6       ZDA          Disabled
7       GRS          Disabled
8       GST          Disabled

===============================================================================
6. DRIVER ARCHITECTURE
===============================================================================

                 Quectel L89
                      │
                UART (NMEA)
                      │
                      ▼
          DMA Circular Buffer
             dma_buffer[]
                      │
                      ▼
             GPS_DMA_Read()
                      │
                      ▼
          Process Buffer
         process_buffer[]
                      │
                      ▼
           GPS_ProcessByte()
                      │
                      ▼
          Sentence Buffer
         sentence_buffer[]
                      │
                      ▼
         GPS_Checksum()
                      │
             Checksum Valid?
              │          │
             No         Yes
              │          ▼
          Discard   GPS_ParseSentence()
                           │
                  ┌────────┴─────────┐
                  ▼                  ▼
            latest_rmc[]       latest_gga[]
                  │                  │
                  └────────┬─────────┘
                           ▼
                  GPS_ParseRMC()
                  GPS_ParseGGA()
                           │
                           ▼
                GPS_ConvertUTCToIST()
                           │
                           ▼
                     GPS_Data_t
                           │
                           ▼
                     GPS_GetData()

===============================================================================
7. BUFFER DESCRIPTION
===============================================================================

dma_buffer
----------

UART DMA Circular Buffer.

DMA continuously writes incoming UART bytes into this buffer.

No CPU intervention is required while DMA is receiving data.


process_buffer
--------------

Stores only the newly received bytes copied from the DMA buffer.

GPS_ProcessByte() processes these bytes one by one.


sentence_buffer
---------------

Builds one complete NMEA sentence.

Starts with:

    $

Ends with:

    \n

After a complete sentence is received, checksum verification is performed.


latest_rmc
----------

Stores the latest valid RMC sentence.

This sentence is parsed for:

    • UTC Time
    • UTC Date
    • Latitude
    • Longitude
    • Position Status


latest_gga
----------

Stores the latest valid GGA sentence.

This sentence is parsed for:

    • Altitude

===============================================================================
8. TIMER DESCRIPTION
===============================================================================

process_timer
-------------

Runs every GPS_PROCESS_INTERVAL.

Responsibilities:

    • Read DMA buffer
    • Copy new bytes
    • Process received bytes
    • Assemble NMEA sentences


update_timer
------------

Runs every GPS_UPDATE_INTERVAL.

Responsibilities:

    • Parse latest RMC sentence
    • Parse latest GGA sentence
    • Convert UTC to IST
    • Update GPS_Data_t
    • Print Debug Information

===============================================================================
9. GPS DATA STRUCTURE
===============================================================================

GPS_Data_t stores:

Latitude

Longitude

Altitude

UTC Time

UTC Date

IST Time

IST Date

GPS Position Status

===============================================================================
10. RMC FORMAT
===============================================================================

Example

$GNRMC,044413.000,A,2306.2335,N,07235.6633,E,0.00,0.00,090726,,,A*6A

Field

0  Sentence Identifier

1  UTC Time

2  Position Status

3  Latitude

4  North / South

5  Longitude

6  East / West

7  Speed

8  Course

9  UTC Date

10 Magnetic Variation

11 Magnetic Direction

12 Mode Indicator

13 Checksum

Driver Uses

UTC Time

UTC Date

Latitude

Longitude

Position Status

===============================================================================
11. GGA FORMAT
===============================================================================

Example

$GNGGA,044413.000,2306.2335,N,07235.6633,E,1,13,1.0,123.4,M,-34.0,M,,*47

Field

0 Sentence Identifier

1 UTC Time

2 Latitude

3 North/South

4 Longitude

5 East/West

6 Fix Quality

7 Satellites

8 HDOP

9 Altitude

10 Altitude Unit

11 Geoid Separation

12 Geoid Unit

13 DGPS Age

14 DGPS Station

15 Checksum

Driver Uses

Altitude

===============================================================================
12. PUBLIC API
===============================================================================

GPS_Init()

Initializes the GPS driver and starts UART DMA reception.


GPS_Task()

Main driver task.

Must be called continuously from the main loop.


GPS_GetData()

Returns the latest valid GPS information.

===============================================================================
13. DEBUG SUPPORT
===============================================================================

Debug messages can be enabled or disabled using:

#define GPS_DEBUG 1

or

#define GPS_DEBUG 0

When disabled, no debug UART transmissions are performed.

===============================================================================
14. FUTURE IMPROVEMENTS
===============================================================================

Possible future enhancements:

• Satellite Count
• HDOP
• Speed
• Course Over Ground
• Fix Quality
• Date/Time Timezone Selection
• PPS Support
• RTOS Compatibility
• Multiple GPS Module Support

===============================================================================
15. VERSION
===============================================================================

Version : 1.0

Developed By

Khushi Gajjar

Quectel L89 GNSS Driver
STM32 HAL UART DMA Implementation

===============================================================================