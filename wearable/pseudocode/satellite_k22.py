constants:
	READINGS_BUFFER_REDLINE
	MAX_DATUM_COUNT_PER_TRANSFER
	SPI_REQUEST_TYPES = [ SPI_RQ_ACTIVE, SPI_RQ_INACTIVE, SPI_RQ_SHUT_UP, SPI_RQ_CALIBRATE, SPI_RQ_SET_RTC ]

globals:
	asleep = False
	can_send = True
	should_calibrate = False
	readings = [] #circular
	spi_stm_data = None

def spi_received(type, data):
	if SPI_RQ_ACTIVE:
		asleep = False
		can_send = True
	elif SPI_RQ_CALIBRATE:
		should_calibrate = True
	elif SPI_RQ_INACTIVE:
		asleep = True
	elif SPI_RQ_SHUT_UP:
		can_send = False
	elif SPI_RQ_SET_RTC:
		set_rtc(data)
	if can_send:
		spi_stm_data = readings[:MAX_DATUM_COUNT_PER_TRANSFER]
		readings = readings[MAX_DATUM_COUNT_PER_TRANSFER:]
	register_spi_answer(spi_stm_data)

register_spi_answer(spi_stm_data)
register_spi_callback(spi_received)

while True:
	if asleep:
		invensense_turn_off()
		while asleep:
			_WFE()
		invensense_turn_on()
	if should_calibrate:
		invensense_calibrate()
		should_calibrate = False
	if(len(readings) <= READINGS_BUFFER_REDLINE):
		readings += sensor_fusion_process_data(invensense_get_data())
	elif readings[-1] != TRUNCATION_WARNING:
		readings.append(TRUNCATION_WARNING)
		