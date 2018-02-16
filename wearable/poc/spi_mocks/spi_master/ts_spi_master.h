#include <stdbool.h>

// Parameters of the simulation.
#define REQUESTS_PER_SECOND 400
#define REQUESTS_BETWEEN_SHUSHES 800
#define REQUESTS_PER_SHUSH 6
#define REQUESTS_BETWEEN_STOPS 2500
#define REQUESTS_PER_STOP 6

#define DELAY_MS                 (int)(1000.0/REQUESTS_PER_SECOND)  /**< Timer Delay in milli-seconds. */
bool spi_master_pull_tick(void);
bool spi_master_pull(void);
bool spi_master_check_tick(void);
void spi_init(void);
void start_spi(void);
