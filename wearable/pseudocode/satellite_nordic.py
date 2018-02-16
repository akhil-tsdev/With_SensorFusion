constants:
	CHANNELS
	ESB_TRANSMIT_TYPES = [ ESB_TX_CONNECT, ESB_TX_DATA ]
	ESB_RESPONSE_TYPES = [ ESB_RS_NOT_YOU, ESB_RS_STOP, ESB_RS_START, ESB_RS_SHUT_UP, ESB_RS_CALIBRATE, ESB_RS_SET_RTC, ERROR ]
	SPI_REQUEST_TYPES = [ SPI_RQ_ACTIVE, SPI_RQ_INACTIVE, SPI_RQ_SHUT_UP, SPI_RQ_CALIBRATE, SPI_RQ_SET_RTC ]
	HUB_TIMEOUT
	NAP_LENGTH

globals:
	hub_channel = None
	active = False
	should_go_to_sleep = False
	hub_swamped = False
	new_rtc_value = None
	should_calibrate = False
	sensor_data_buffer = [] #circular

def esb_callback(response_type):
	response = response_type

def send_to_hub(channel, transmit_type):
	start_time = now()
	esb_send(channel, esb_packet_for_transmit_type(transmit_type), esb_callback)
	while response is None and start_time + HUB_TIMEOUT < now():
		pass
	# TODO fill out auto response handling to populate flags (hub_channel, active, sleep, hub_swamped, &c)

def send_to_kinetis(request):
	response_data = spi_communicate(spi_packet_for_request_type(request))
	if response_data:
		sensor_data_buffer += response_data
	if len(sensor_data_buffer) and not hub_swamped:
		send_to_hub(hub_channel, ESB_TX_DATA)

def find_hub():
	for channel in CHANNELS:
		send_to_hub(channel, ESB_TX_CONNECT)
		if hub_channel is not None:
			return
	
while True:
	while(hub_channel is None):
		find_hub()
	if buffer_almost_full:
		send_to_kinetis(SPI_RQ_SHUT_UP)
	elif should_go_to_sleep:
		send_to_kinetis(SPI_RQ_INACTIVE)
		should_go_to_sleep = False
	elif should_calibrate:
		send_to_kinetis(SPI_RQ_CALIBRATE)
		should_calibrate = False
	elif new_rtc_value is not None:
		send_to_kinetis(SPI_RQ_SET_RTC)
		new_rtc_value = None
	elif active:
		send_to_kinetis(SPI_RQ_ACTIVE)
	if not active:
		sys_sleep(NAP_LENGTH)