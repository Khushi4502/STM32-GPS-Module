/******************************************************************************
 * @file        gps.h
 * @brief       GPS Driver for Quectel L89 GNSS Receiver
 *
 * ---------------------------------------------------------------------------
 * Module Overview
 * ---------------------------------------------------------------------------
 * This driver interfaces with the Quectel L89 GNSS module using UART DMA in
 * circular mode. Incoming NMEA data is received continuously, validated using
 * checksum verification and parsed to obtain GPS information.
 *
 * Driver Flow
 * ---------------------------------------------------------------------------
 *
 * Quectel L89
 *      │
 *      ▼
 * UART DMA Circular Buffer (dma_buffer)
 *      │
 *      │  Every GPS_PROCESS_INTERVAL (50 ms)
 *      ▼
 * GPS_DMA_Read()
 *      │
 *      ▼
 * Process Buffer (process_buffer)
 *      │
 *      ▼
 * GPS_ProcessByte()
 *      │
 *      ▼
 * Sentence Buffer (sentence_buffer)
 *      │
 *      ▼
 * GPS_Checksum()
 *      │
 *      ├── Invalid Checksum → Sentence Discarded
 *      │
 *      └── Valid Checksum
 *              │
 *              ▼
 *      GPS_ParseSentence()
 *              │
 *      ┌───────┴────────┐
 *      ▼                ▼
 * latest_rmc[]      latest_gga[]
 *      │                │
 *      └───────┬────────┘
 *              ▼
 * Every GPS_UPDATE_INTERVAL (30000 ms)
 *              │
 *      GPS_ParseRMC()
 *      GPS_ParseGGA()
 *              │
 *              ▼
 *      GPS_ConvertUTCToIST()
 *              │
 *              ▼
 *          GPS_Data_t
 *              │
 *              ▼
 *        GPS_GetData()
 *
 * Buffer Description
 * ---------------------------------------------------------------------------
 *
 * dma_buffer
 * ----------
 * UART DMA circular receive buffer.
 * Continuously filled by the DMA controller with incoming GPS data.
 *
 * process_buffer
 * --------------
 * Stores only the newly received bytes copied from the DMA buffer.
 * These bytes are processed one at a time.
 *
 * sentence_buffer
 * ---------------
 * Builds one complete NMEA sentence.
 * Starts at '$' and ends at '\n'.
 * The completed sentence is verified before parsing.
 *
 * latest_rmc
 * ----------
 * Stores the latest valid RMC sentence.
 * Used to extract:
 *   - UTC Time
 *   - UTC Date
 *   - Latitude
 *   - Longitude
 *   - Position Status
 *
 * latest_gga
 * ----------
 * Stores the latest valid GGA sentence.
 * Used to extract:
 *   - Altitude
 *
 * Timer Description
 * ---------------------------------------------------------------------------
 *
 * process_timer
 * -------------
 * Executes every GPS_PROCESS_INTERVAL.
 * Reads newly received bytes from the DMA buffer and processes them.
 * This keeps up with continuous UART reception.
 *
 * update_timer
 * ------------
 * Executes every GPS_UPDATE_INTERVAL.
 * Parses the latest RMC and GGA sentences, converts UTC to IST,
 * updates GPS_Data_t and makes the latest GPS information available
 * to the application.
 *
 * GPS_Data_t
 * ---------------------------------------------------------------------------
 *
 * Stores the latest parsed GPS information:
 *
 *   • Latitude
 *   • Longitude
 *   • Altitude
 *   • UTC Time
 *   • UTC Date
 *   • IST Time
 *   • IST Date
 *   • GPS Position Status
 *
 * L89 NMEA Command Output Configuration (PAIR062)
 *
 * +-------+----------+-------------------------------------------------------+
 * | Index | Sentence | Parameters                                            |
 * +-------+----------+-------------------------------------------------------+
 * |   0   | GGA      | UTC Time, Latitude, Longitude, Fix Quality,           |
 * |       |          | Number of Satellites, HDOP, Altitude, Geoid Height    |
 * +-------+----------+-------------------------------------------------------+
 * |   1   | GLL      | Latitude, Longitude, UTC Time, Position Status         |
 * +-------+----------+-------------------------------------------------------+
 * |   2   | GSA      | Fix Type, PDOP, HDOP, VDOP, Active Satellites          |
 * +-------+----------+-------------------------------------------------------+
 * |   3   | GSV      | Satellites In View, Elevation, Azimuth, SNR            |
 * +-------+----------+-------------------------------------------------------+
 * |   4   | RMC      | UTC Time, Position Status, Latitude, Longitude,        |
 * |       |          | Speed, Course, UTC Date, Magnetic Variation            |
 * +-------+----------+-------------------------------------------------------+
 * |   5   | VTG      | Course Over Ground, Magnetic Course,                  |
 * |       |          | Ground Speed (Knots / km/h)                            |
 * +-------+----------+-------------------------------------------------------+
 * |   6   | ZDA      | UTC Time, Day, Month, Year, Local Time Zone            |
 * +-------+----------+-------------------------------------------------------+
 * |   7   | GRS      | GNSS Range Residual Error Information                  |
 * +-------+----------+-------------------------------------------------------+
 * |   8   | GST      | GNSS Position Error Statistics                         |
 * +-------+----------+-------------------------------------------------------+
 *
 * Enabled  : RMC, GGA
 * Disabled : GLL, GSA, GSV, VTG, ZDA, GRS, GST
 *
 * Driver Uses:
 * ---------------------------------------------------------------------------
 * RMC : UTC Time, UTC Date, Latitude, Longitude, Position Status
 * GGA : Altitude
 *
 * Driver Features
 * ---------------------------------------------------------------------------
 *  • UART DMA Circular Reception
 *  • NMEA Sentence Assembly
 *  • Checksum Verification
 *  • RMC & GGA Parsing
 *  • Latitude / Longitude Conversion
 *  • UTC to IST Conversion
 *  • Configurable Update Interval
 *  • Optional Debug Output
 *
 * Code Completion

 ******************************************************************************/

#ifndef __GPS_H__
#define __GPS_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Macros --------------------------------------------------------------------*/
#define GPS_DMA_BUFFER_SIZE         					1024    								   // DMA Circular Buffer Size (Bytes)
#define GPS_PROCESS_BUFFER_SIZE      					200    									   // Maximum bytes copied from DMA buffer for processing
#define GPS_SENTENCE_BUFFER_SIZE     					160    									   // Maximum size of one complete NMEA sentence
#define GPS_PROCESS_INTERVAL          					50    			                           // Interval to read new data from DMA buffer (ms)
#define GPS_UPDATE_INTERVAL        						30000    								   // Interval to update parsed GPS data and print debug info (ms)

#define GPS_DEBUG    									1

/* GPS Data Structure --------------------------------------------------------*/

typedef struct {
	/* Position */
	double 												latitude;
	double 												longitude;

	/* Direction */
	char 												latitude_dir;   	                       // 'N' or 'S'
	char 												longitude_dir;  	                       // 'E' or 'W'

	/* Altitude */
	float 												altitude;

	/* UTC Time */
	uint8_t 											hour;
	uint8_t 											minute;
	uint8_t 											second;

	/* UTC Date */
	uint8_t 											day;
	uint8_t 											month;
	uint16_t 											year;

	/* Indian Standard Time (UTC +05:30) */
	uint8_t 											ist_hour;
	uint8_t 											ist_minute;
	uint8_t 											ist_second;

	/* IST Date */
	uint8_t 											ist_day;
	uint8_t 											ist_month;
	uint16_t 											ist_year;

	/*
	 * GPS Position Fix Status
	 * true  -> Receiver has calculated a valid geographic position
	 *          (RMC Status = 'A')
	 * false -> Receiver has not yet obtained a valid position
	 *          (RMC Status = 'V')
	 */
	bool 												gps_position;

} GPS_Data_t;

/* GPS Driver Structure ------------------------------------------------------*/

typedef struct
{
    UART_HandleTypeDef 									*gps_uart;                                 // GPS UART Handle
    UART_HandleTypeDef 									*debug_uart;                               // Debug UART Handle

    uint8_t 											dma_buffer[GPS_DMA_BUFFER_SIZE];           	// DMA Circular Buffer
    uint8_t 											process_buffer[GPS_PROCESS_BUFFER_SIZE];   	// Processing Buffer

    //example frame
    char 												sentence_buffer[GPS_SENTENCE_BUFFER_SIZE]; // Current NMEA Sentence
    char 												latest_rmc[GPS_SENTENCE_BUFFER_SIZE];      // Latest RMC Sentence - lat/long/date/time
    char 												latest_gga[GPS_SENTENCE_BUFFER_SIZE];      // Latest GGA Sentence - altitude

    uint16_t 											old_position;                              // Previous DMA Position
    uint16_t 											current_position;                          // Current DMA Position

    uint16_t 											process_length;                            // Bytes To Process

    uint16_t 											sentence_index;                            // Current Sentence Index

    bool 												sentence_started;                          // NMEA Sentence Reception Flag

    uint32_t 											process_timer;                             // DMA Processing Timer
    uint32_t 											update_timer;                              // GPS Update Timer

    uint32_t 											update_interval;                           // GPS Update Interval (ms)

    GPS_Data_t 											data;              	                       // Parsed GPS Data

} GPS_t;

/* Function Prototypes -------------------------------------------------------*/

/* Initialization Functions -------------------------------------------------*/
void GPS_Init(GPS_t *gps, UART_HandleTypeDef *gps_uart,UART_HandleTypeDef *debug_uart);

/* Driver Functions ---------------------------------------------------------*/
void GPS_Task(GPS_t *gps);
bool GPS_GetData(GPS_t *gps, GPS_Data_t *data);

/* DMA Processing Functions -------------------------------------------------*/
void GPS_DMA_Read(GPS_t *gps);
void GPS_ProcessByte(GPS_t *gps, uint8_t byte);

/* NMEA Processing Functions ------------------------------------------------*/
bool GPS_Checksum(char *sentence);
void GPS_ParseSentence(GPS_t *gps);
void GPS_ParseRMC(GPS_t *gps);
void GPS_ParseGGA(GPS_t *gps);

/* Utility Functions --------------------------------------------------------*/
double GPS_ConvertCoordinate(char *coord, bool latitude);
void GPS_ConvertUTCToIST(GPS_t *gps);
uint8_t GPS_HexToNibble(char c);
void GPS_DebugPrint(GPS_t *gps, const char *msg);

#endif
