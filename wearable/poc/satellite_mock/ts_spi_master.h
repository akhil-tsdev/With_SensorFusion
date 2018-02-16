#include <stdbool.h>
#include "sat_spi_types.h"

// Parameters of the simulation.
#define REQUESTS_PER_SECOND 200
#define REQUESTS_BETWEEN_SHUSHES 4000
#define REQUESTS_PER_SHUSH 500
#define REQUESTS_BETWEEN_STOPS 11000
#define REQUESTS_PER_STOP 1300

#define DELAY_MS                 (int)(1000.0/REQUESTS_PER_SECOND)  /**< Timer Delay in milli-seconds. */
typedef void (*spi_callback)(sat_spi_data_packet_t);

/// True IFF the command was actually sent.
bool spi_send_command(sat_control_packet_t);
bool spi_master_pull_tick(void);
bool spi_master_check_tick(void);
void start_spi(spi_callback);
