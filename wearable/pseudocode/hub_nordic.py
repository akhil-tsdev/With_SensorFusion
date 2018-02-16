constants:
	BUFFER_REDLINE
	MAX_DATUM_COUNT_PER_TRANSFER
	ESB_TRANSMIT_TYPES = [ ESB_TX_CONNECT, ESB_TX_DATA ]
	ESB_RESPONSE_TYPES = [ ESB_RS_NOT_YOU, ESB_RS_STOP, ESB_RS_START, ESB_RS_SHUT_UP, ESB_RS_CALIBRATE, ESB_RS_SET_RTC ]
	SPI_REQUEST_TYPES = [ SPI_RQ_SET_SATELLITE_LIST, SPI_RQ_START, SPI_RQ_STOP, SPI_RQ_SHUT_UP, SPI_RQ_CALIBRATE, SPI_RQ_SET_RTC ]

globals:
	satellites { 
		satellite_id,
		satellite {
			pending_messages = []
		} 
	}
	active = False
	can_send = True
	
	my_satellite_ids = []
	data_buffer = []
	spi_stm_data = None

def spi_received(type, data):
	if SPI_RQ_START:
		active = True
		can_send = True
	elif SPI_RQ_CALIBRATE:
		add_message_to_satellites(ESB_RS_CALIBRATE)
	elif SPI_RQ_STOP:
		active = False
	elif SPI_RQ_SHUT_UP:
		can_send = False
	elif SPI_RQ_SET_RTC:
		set_rtc(data)
		add_message_to_satellites(ESB_RS_SET_RTC, data)
	elif SPI_RQ_SET_SATELLITE_LIST:
		my_satellite_ids = data
	if can_send:
		spi_stm_data = data_buffer[:MAX_DATUM_COUNT_PER_TRANSFER]
		data_buffer = data_buffer[MAX_DATUM_COUNT_PER_TRANSFER:]
	register_spi_answer(spi_stm_data)

register_spi_answer(spi_stm_data)
register_spi_callback(spi_received)

def esb_received(type, satellite_id, data):
	if ESB_TX_CONNECT:
		if satellite_id in my_satellite_ids:
			satellites += {satellite_id = satellite_id, pending_messages = []}
		else:
			return make_esb_packet(ESB_RS_NOT_YOU)
	elif ESB_TX_DATA:
		data_buffer += data
	if len(data) >= BUFFER_REDLINE:
		return make_esb_packet(ESB_RS_SHUT_UP)
	elif not active:
		return make_esb_packet(ESB_RS_STOP)
	elif len(satellites[satellite_id].pending_messages):
		message = satellites[satellite_id].pending_messages[0]
		satellites[satellite_id].pending_messages = satellites[satellite_id].pending_messages[1:]
		return make_esb_packet(message)
	else:
		return make_esb_packet(ESB_RS_START)

while True:
	_WFE()