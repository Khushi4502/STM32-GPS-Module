/******************************************************************************
 * @file    gps.c
 * @brief   GPS Driver

 ******************************************************************************/

#include "gps.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/****************************************************************************
 * Function    : GPS_Init
 * Description : Initialize GPS driver, start UART DMA reception and
 *               configure GPS NMEA output.
 * Parameters  :
 *               gps         -> GPS Driver Structure
 *               gps_uart    -> GPS UART Handle
 *               debug_uart  -> Debug UART Handle (NULL to disable debug)
 * Return      : None
 ****************************************************************************/
void GPS_Init(GPS_t *gps, UART_HandleTypeDef *gps_uart,UART_HandleTypeDef *debug_uart) {
	/* Check Parameters */
	if ((gps == NULL) || (gps_uart == NULL)) {
		return;
	}

	/* Store UART Handles */
	gps->gps_uart 										= gps_uart;
	gps->debug_uart 									= debug_uart;

	/* Clear Driver Data */
	memset(gps->dma_buffer, 0, sizeof(gps->dma_buffer));
	memset(gps->process_buffer, 0, sizeof(gps->process_buffer));
	memset(gps->sentence_buffer, 0, sizeof(gps->sentence_buffer));
	memset(gps->latest_rmc, 0, sizeof(gps->latest_rmc));
	memset(gps->latest_gga, 0, sizeof(gps->latest_gga));
	memset(&gps->data, 0, sizeof(GPS_Data_t));

	/* Initialize Driver Variables */
	gps->old_position 									= 0;
	gps->current_position 								= 0;
	gps->process_length 								= 0;

	gps->sentence_index 								= 0;
	gps->sentence_started 								= false;
	gps->process_timer 									= HAL_GetTick();
	gps->update_timer 									= HAL_GetTick();

	gps->update_interval 								= GPS_UPDATE_INTERVAL;

	/* Start UART DMA Reception */
	if (HAL_UART_Receive_DMA(gps->gps_uart, gps->dma_buffer,GPS_DMA_BUFFER_SIZE) != HAL_OK)
	{
		GPS_DebugPrint(gps, "\r\nERROR : GPS DMA Start Failed\r\n");
		return;
	}

	GPS_DebugPrint(gps, "\r\nGPS DMA Started Successfully\r\n");

	/* Wait For GPS Startup */
	HAL_Delay(500);

	/* Configure NMEA Output */
	const char *gps_cfg[] = { "$PAIR062,0,1*3F\r\n", /* Enable GGA */
	"$PAIR062,4,1*3B\r\n", /* Enable RMC */

	"$PAIR062,1,0*3F\r\n", /* Disable GLL */
	"$PAIR062,2,0*3C\r\n", /* Disable GSA */
	"$PAIR062,3,0*3D\r\n", /* Disable GSV */
	"$PAIR062,5,0*3B\r\n", /* Disable VTG */
	"$PAIR062,6,0*38\r\n", /* Disable ZDA */
	"$PAIR062,7,0*39\r\n", /* Disable GRS */
	"$PAIR062,8,0*36\r\n"  /* Disable GST */
	};

	for (uint8_t i = 0; i < (sizeof(gps_cfg) / sizeof(gps_cfg[0])); i++) {
		HAL_UART_Transmit(gps->gps_uart, (uint8_t*) gps_cfg[i],strlen(gps_cfg[i]), 100);
		HAL_Delay(100);
	}

	/* Synchronize DMA Position */
	gps->old_position 									= GPS_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(gps->gps_uart->hdmarx);

	gps->current_position 								= gps->old_position;

	gps->process_length 								= 0;
	gps->sentence_index 								= 0;
	gps->sentence_started 								= false;

	memset(gps->process_buffer, 0, sizeof(gps->process_buffer));
	memset(gps->sentence_buffer, 0, sizeof(gps->sentence_buffer));

	GPS_DebugPrint(gps, "GPS DMA Synchronized\r\n");
}

/****************************************************************************
 * Function    : GPS_Task
 * Description : Execute GPS background tasks including DMA data processing,
 *               NMEA sentence parsing and periodic GPS data update.
 * Parameters  :
 *               gps -> GPS Driver Structure
 * Return      : None
 ****************************************************************************/
void GPS_Task(GPS_t *gps) {
	// Check Driver Pointer
	if (gps == NULL) {
		return;
	}

	uint32_t current_tick 								= HAL_GetTick();

	// Read New Data From Circular DMA Buffer

	if ((current_tick - gps->process_timer) >= GPS_PROCESS_INTERVAL) {
		gps->process_timer 								= current_tick;

		GPS_DMA_Read(gps);
	}

	// Update GPS Data Periodically

	if ((current_tick - gps->update_timer) >= gps->update_interval) {
		gps->update_timer 								= current_tick;

		/* Parse Latest RMC Sentence */
		if (strlen(gps->latest_rmc) > 0) {
			GPS_ParseRMC(gps);
		}

		/* Parse Latest GGA Sentence */
		if (strlen(gps->latest_gga) > 0) {
			GPS_ParseGGA(gps);
		}

		GPS_ConvertUTCToIST(gps);

		// Print Latest GPS Information

		GPS_DebugPrint(gps, "\r\n================ GPS =================\r\n");

		GPS_DebugPrint(gps, "Latest RMC : ");
		GPS_DebugPrint(gps, gps->latest_rmc);
		GPS_DebugPrint(gps, "\r\n");

		GPS_DebugPrint(gps, "Latest GGA : ");
		GPS_DebugPrint(gps, gps->latest_gga);
		GPS_DebugPrint(gps, "\r\n");

		GPS_DebugPrint(gps, "--------------------------------------\r\n");

		char msg[200];

		snprintf(msg, sizeof(msg), "Latitude   : %.6f\r\n"
				"Longitude  : %.6f\r\n"
				"Altitude   : %.2f m\r\n"
				"UTC Time   : %02d:%02d:%02d\r\n"
				"UTC Date   : %02d/%02d/%04d\r\n"
				"IST Time   : %02d:%02d:%02d\r\n"
				"IST Date   : %02d/%02d/%04d\r\n"
				"GPS Position: %s\r\n",
				gps->data.latitude,
				gps->data.longitude,
				gps->data.altitude,
				gps->data.hour,
				gps->data.minute,
				gps->data.second,
				gps->data.day,
				gps->data.month,
				gps->data.year,
				gps->data.ist_hour,
				gps->data.ist_minute,
				gps->data.ist_second,
				gps->data.ist_day,
				gps->data.ist_month,
				gps->data.ist_year,
				gps->data.gps_position ? "VALID" : "INVALID");

		GPS_DebugPrint(gps, msg);

		GPS_DebugPrint(gps, "======================================\r\n");
	}
}

/****************************************************************************
 * Function    : GPS_DMA_Read
 * Description : Read newly received bytes from the UART DMA circular buffer,
 *               copy them into the processing buffer and process each byte.
 * Parameters  :
 *               gps -> GPS Driver Structure
 * Return      : None
 ****************************************************************************/
void GPS_DMA_Read(GPS_t *gps) {
	// Check Driver Pointer
	if (gps == NULL)
	{
		return;
	}

	// Get Current DMA Write Position
	gps->current_position 								=	GPS_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(gps->gps_uart->hdmarx);

	// No New Data Received
	if (gps->current_position == gps->old_position)
	{
		return;
	}

	// Reset Process Buffer Length
	gps->process_length 								= 0;

	// Copy Data Without Buffer Wrap
	if (gps->current_position > gps->old_position) {
		while (gps->old_position < gps->current_position)
		{
			if (gps->process_length >= GPS_PROCESS_BUFFER_SIZE)
			{
				GPS_DebugPrint(gps, "WARNING : Process Buffer Full\r\n");

				break;
			}

			gps->process_buffer[gps->process_length++] 	= gps->dma_buffer[gps->old_position++];
		}
	}

	else {
		// Copy Remaining Data To End Of DMA Buffer
		while (gps->old_position < GPS_DMA_BUFFER_SIZE)
		{
			if (gps->process_length >= GPS_PROCESS_BUFFER_SIZE)
			{
				GPS_DebugPrint(gps, "WARNING : Process Buffer Full\r\n");

				break;
			}
			gps->process_buffer[gps->process_length++] 	= gps->dma_buffer[gps->old_position++];
		}

		// Continue Copy From Start Of DMA Buffer
		gps->old_position 								= 0;

		while (gps->old_position < gps->current_position)
		{
			if (gps->process_length >= GPS_PROCESS_BUFFER_SIZE)
			{
				GPS_DebugPrint(gps, "WARNING : Process Buffer Full\r\n");
				break;
			}
			gps->process_buffer[gps->process_length++] 	= gps->dma_buffer[gps->old_position++];
		}
	}

	// Process Received Bytes
	for (uint16_t i = 0; i < gps->process_length; i++)
	{
		GPS_ProcessByte(gps, gps->process_buffer[i]);
	}
}

/****************************************************************************
 * Function    : GPS_ProcessByte
 * Description : Process each received byte, build a complete NMEA sentence,
 *               verify its checksum and store the latest valid sentence.
 * Parameters  :
 *               gps  -> GPS Driver Structure
 *               byte -> Received UART Byte
 * Return      : None
 ****************************************************************************/
void GPS_ProcessByte(GPS_t *gps, uint8_t byte) {
	// Check Driver Pointer
	if (gps == NULL) {
		return;
	}

	// Detect Start Of NMEA Sentence
	if (byte == '$') {
		gps->sentence_started 							= true;
		gps->sentence_index 							= 0;

		memset(gps->sentence_buffer, 0,GPS_SENTENCE_BUFFER_SIZE);

		gps->sentence_buffer[gps->sentence_index++] 	= byte;

		return;
	}

	// Ignore Data Until Start Character Is Received
	if (gps->sentence_started == false) {
		return;
	}

	// Store Received Byte
	if (gps->sentence_index < (GPS_SENTENCE_BUFFER_SIZE - 1))
	{
		gps->sentence_buffer[gps->sentence_index++] = byte;
	} else {
		GPS_DebugPrint(gps, "ERROR : Sentence Buffer Overflow\r\n");

		gps->sentence_started = false;
		gps->sentence_index = 0;

		return;
	}

	// Complete NMEA Sentence Received
	if (byte == '\n') {
		gps->sentence_buffer[gps->sentence_index] = '\0';

		// Verify NMEA Checksum
		if (GPS_Checksum(gps->sentence_buffer)) {

		// Store Latest Valid NMEA Sentence
			GPS_ParseSentence(gps);
		} else {

			GPS_DebugPrint(gps, "Checksum : FAILED\r\n");
		}
		// Prepare For Next Sentence
		gps->sentence_started = false;
		gps->sentence_index = 0;
	}
}

/****************************************************************************
 * Function    : GPS_HexToNibble
 * Description : Convert an ASCII hexadecimal character to its numeric value.
 * Parameters  :
 *               c -> ASCII Hex Character ('0'-'9', 'A'-'F', 'a'-'f')
 * Return      : Hexadecimal Value (0-15)
 ****************************************************************************/
uint8_t GPS_HexToNibble(char c) {
	// Convert Numeric Character
	if ((c >= '0') && (c <= '9')) {
		return (uint8_t) (c - '0');
	}

	// Convert Uppercase Hex Character
	if ((c >= 'A') && (c <= 'F')) {
		return (uint8_t) (c - 'A' + 10);
	}

	// Convert Lowercase Hex Character
	if ((c >= 'a') && (c <= 'f')) {
		return (uint8_t) (c - 'a' + 10);
	}

	// Invalid Hex Character
	return 0;
}

/****************************************************************************
 * Function    : GPS_Checksum
 * Description : Verify the checksum of a received NMEA sentence.
 * Parameters  :
 *               sentence -> NMEA Sentence
 * Return      : true  -> Checksum Matched
 *               false -> Checksum Failed
 ****************************************************************************/
bool GPS_Checksum(char *sentence) {
	uint8_t calculated_checksum 						= 0;
	uint8_t received_checksum 							= 0;

	char *ptr;

	// Check Input Pointer
	if (sentence == NULL) {
		return false;
	}

	// Verify Start Of NMEA Sentence
	if (sentence[0] != '$') {
		return false;
	}

	// Skip '$' Character
	ptr = &sentence[1];

	// Calculate Checksum
	while ((*ptr != '*') &&
			(*ptr != '\0') &&
			(*ptr != '\r') &&
			(*ptr != '\n'))
	{
		calculated_checksum ^= (uint8_t) (*ptr);
		ptr++;
	}

	// Verify Checksum Delimiter
	if (*ptr != '*') {
		return false;
	}

	// Move To Checksum Field
	ptr++;

	// Verify Checksum Length
	if ((ptr[0] == '\0') || (ptr[1] == '\0')) {
		return false;
	}

	// Convert Received Checksum
	received_checksum = (GPS_HexToNibble(ptr[0]) << 4)| GPS_HexToNibble(ptr[1]);

	// Compare Checksums
	return (calculated_checksum == received_checksum);
}

/****************************************************************************
 * Function    : GPS_ParseSentence
 * Description : Identify the received NMEA sentence and store the latest
 *               valid RMC or GGA sentence.
 * Parameters  :
 *               gps -> GPS Driver Structure
 * Return      : None
 ****************************************************************************/
void GPS_ParseSentence(GPS_t *gps) {
	// Check Driver Pointer
	if (gps == NULL) {
		return;
	}

	// Store Latest RMC Sentence
	if (strncmp(gps->sentence_buffer, "$GNRMC", 6) == 0) {
		strncpy(gps->latest_rmc, gps->sentence_buffer,GPS_SENTENCE_BUFFER_SIZE - 1);

		gps->latest_rmc[GPS_SENTENCE_BUFFER_SIZE - 1] 	= '\0';
		return;
	}

	// Store Latest GGA Sentence
	if (strncmp(gps->sentence_buffer, "$GNGGA", 6) == 0) {
		strncpy(gps->latest_gga, gps->sentence_buffer,GPS_SENTENCE_BUFFER_SIZE - 1);

		gps->latest_gga[GPS_SENTENCE_BUFFER_SIZE - 1] 	= '\0';
		return;
	}
}

/****************************************************************************
 * Function    : GPS_ConvertCoordinate
 * Description : Convert NMEA latitude or longitude coordinate to
 *               decimal degrees.
 * NMEA Format:
 * Latitude  : ddmm.mmmm
 * Longitude : dddmm.mmmm
 * Parameters  :
 *               coord    -> NMEA Coordinate String
 *               latitude -> Coordinate Type (Unused)
 * Return      : Coordinate In Decimal Degrees
 ****************************************************************************/
double GPS_ConvertCoordinate(char *coord, bool latitude) {
	double value;
	int degree;
	double minute;

	// Suppress Unused Parameter Warning
	(void) latitude;

	// Check Input Pointer
	if (coord == NULL) {
		return 0.0;
	}

	// Convert Coordinate String To Number
	value 												= atof(coord);

	// Extract Degrees
	degree 												= (int) (value / 100);

	// Extract Minutes
	minute 												= value - (degree * 100);

	// Convert To Decimal Degrees
	return (degree + (minute / 60.0));
}

/****************************************************************************
 * Function    : GPS_ConvertUTCToIST
 * Description : Convert UTC date and time to Indian Standard Time (IST).
 * Parameters  :
 *               gps -> GPS Driver Structure
 * Return      : None
 ****************************************************************************/
void GPS_ConvertUTCToIST(GPS_t *gps) {
	if (gps == NULL) {
		return;
	}

	// Copy UTC Date
	gps->data.ist_day 									= gps->data.day;
	gps->data.ist_month 								= gps->data.month;
	gps->data.ist_year 									= gps->data.year;

	// Copy UTC  Time
	gps->data.ist_hour 									= gps->data.hour;
	gps->data.ist_minute 								= gps->data.minute;
	gps->data.ist_second 								= gps->data.second;

	// Add 30 Minutes
	gps->data.ist_minute += 30;

	if (gps->data.ist_minute >= 60) {
		gps->data.ist_minute -= 60;
		gps->data.ist_hour++;
	}

	// Add 5 Hours
	gps->data.ist_hour 									+= 5;

	if (gps->data.ist_hour >= 24) {
		gps->data.ist_hour 								-= 24;
	}
}

/****************************************************************************
 * Function    : GPS_ParseRMC
 * Description : Parse the latest RMC sentence and extract UTC time, date,
 *               position, latitude and longitude information.
 * Parameters  :
 *               gps -> GPS Driver Structure
 * Return      : None
 ****************************************************************************/
void GPS_ParseRMC(GPS_t *gps) {
	char buffer[GPS_SENTENCE_BUFFER_SIZE];
	char *token;
	uint8_t field 										= 0;

	// Check Driver Pointer
	if (gps == NULL) {
		return;
	}

	// Check Latest RMC Available
	if (strlen(gps->latest_rmc) == 0) {
		return;
	}

	// Copy Sentence (strtok modifies string)
	strncpy(buffer, gps->latest_rmc,
	GPS_SENTENCE_BUFFER_SIZE);

	buffer[GPS_SENTENCE_BUFFER_SIZE - 1] 				= '\0';

	token = strtok(buffer, ",");

	while (token != NULL) {
		switch (field) {
		// Parse UTC Time
		case 1: {
			if (strlen(token) >= 6) {
				char temp[3];
				temp[2] 								= '\0';

				memcpy(temp, token, 2);
				gps->data.hour 							= atoi(temp);

				memcpy(temp, token + 2, 2);
				gps->data.minute 						= atoi(temp);

				memcpy(temp, token + 4, 2);
				gps->data.second 						= atoi(temp);
			}
		}
			break;

		// Parse GPS Position Status (A = Valid, V = Invalid)
		case 2: {
			gps->data.gps_position = (token[0] == 'A');
		}
			break;

		// Parse Latitude
		case 3: {
			if (gps->data.gps_position) {
				gps->data.latitude 						= GPS_ConvertCoordinate(token, true);
			}
		}
			break;

		// Apply North / South Direction
		case 4: {
			if (token[0] == 'S') {
				gps->data.latitude 						*= -1.0;
			}
		}
			break;

		// Parse Longitude
		case 5: {
			if (gps->data.gps_position) {
				gps->data.longitude 					= GPS_ConvertCoordinate(token, false);
			}
		}
			break;

		// Apply East / West Direction
		case 6: {
			if (token[0] == 'W') {
				gps->data.longitude 					*= -1.0;
			}
		}
			break;

		// Parse UTC Date
		case 9: {
			if (strlen(token) >= 6) {
				char temp[3];
				temp[2]									 = '\0';

				memcpy(temp, token, 2);
				gps->data.day 							= atoi(temp);

				memcpy(temp, token + 2, 2);
				gps->data.month 						= atoi(temp);

				memcpy(temp, token + 4, 2);
				gps->data.year 							= 2000 + atoi(temp);
			}
		}
			break;

		default:
			break;
		}

		token 											= strtok(NULL, ",");
		field++;
	}
}

/****************************************************************************
 * Function    : GPS_ParseGGA
 * Description : Parse the latest GGA sentence and extract altitude
 *               information.
 * Parameters  :
 *               gps -> GPS Driver Structure
 * Return      : None
 ****************************************************************************/
void GPS_ParseGGA(GPS_t *gps) {
	char buffer[GPS_SENTENCE_BUFFER_SIZE];
	char *token;
	uint8_t field									 	= 0;

	// Check Driver Pointer
	if (gps == NULL) {
		return;
	}

	// Check Latest GGA Available
	if (strlen(gps->latest_gga) == 0) {
		return;
	}

	// Copy Sentence (strtok modifies string)
	strncpy(buffer, gps->latest_gga,
	GPS_SENTENCE_BUFFER_SIZE);

	buffer[GPS_SENTENCE_BUFFER_SIZE - 1] 				= '\0';

	token 												= strtok(buffer, ",");

	while (token != NULL) {
		switch (field) {
		// Parse Altitude
		case 9: {
			if (gps->data.gps_position) {
				gps->data.altitude = (float) atof(token);
			}
		}
			break;

		default:
			break;
		}

		token = strtok(NULL, ",");
		field++;
	}
}

/****************************************************************************
 * Function    : GPS_DebugPrint
 * Description : Send a debug message through the debug UART.
 * Parameters  :
 *               gps -> GPS Driver Structure
 *               msg -> Debug Message String
 * Return      : None
 ****************************************************************************/
void GPS_DebugPrint(GPS_t *gps, const char *msg) {
#if GPS_DEBUG
	if (gps == NULL) {
		return;
	}

	// Check Debug UART Handle
	if (gps->debug_uart == NULL) {
		return;
	}

	// Check Message Pointer
	if (msg == NULL) {
		return;
	}

	// Transmit Debug Messag
	HAL_UART_Transmit(gps->debug_uart, (uint8_t*) msg, strlen(msg), 1000);
#endif
}

/****************************************************************************
 * Function    : GPS_GetData
 * Description : Return the latest valid GPS data.
 * Parameters  :
 *               gps  -> GPS Driver Structure
 *               data -> GPS Data Structure
 * Return      : true  -> Valid GPS Data Available
 *               false -> Invalid GPS Data
 ****************************************************************************/
bool GPS_GetData(GPS_t *gps, GPS_Data_t *data) {
	// Check Input Pointers
	if (gps == NULL || data == NULL) {
		return false;
	}

	// Check GPS Position Status
	if (!gps->data.gps_position) {
		return false;
	}

	// Copy Latest GPS Data
	*data = gps->data;

	return true;
}
