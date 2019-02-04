#include "stdint.h"
#include "stdbool.h"
#include "stdio.h"

#include <time.h>
#include <sys/time.h>
#include <unistd.h>

#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>

#define ESC_HZ 250

#define PWM_CHANS_USED 8

static const uint8_t chan_pru_map[]= {0,1,5,6,2,3,4,7,8,9,10,11};
/* Ch1: PRU1.0 - P8.45, Ch2: PRU1.1 P8.46,
 * Ch3: PRU1.2 - P8.43, Ch4: PRU1.3 P8.44,
 * Ch5: PRU1.4 - P8.41, Ch6: PRU1.5 P8.42
 */

#define RCOUT_PRUSS_SHAREDRAM_BASE     0x4a310000
#define MAX_PWMS                 12
#define PWM_CMD_MAGIC            0xf00fbaaf
#define PWM_REPLY_MAGIC          0xbaaff00f
#define PWM_CMD_CONFIG	         0	/* full configuration in one go */
#define PWM_CMD_ENABLE	         1	/* enable a pwm */
#define PWM_CMD_DISABLE	         2	/* disable a pwm */
#define PWM_CMD_MODIFY	         3	/* modify a pwm */
#define PWM_CMD_SET	         4	/* set a pwm output explicitly */
#define PWM_CMD_CLR	         5	/* clr a pwm output explicitly */
#define PWM_CMD_TEST	         6	/* various crap */

static const int TICK_PER_US = 200;
static const int TICK_PER_S = 200000000;

struct pwm_cmd
{
	uint32_t magic;
	uint32_t enmask;     /* enable mask */
	uint32_t offmsk;     /* state when pwm is off */
	uint32_t periodhi[MAX_PWMS][2];
	uint32_t hilo_read[MAX_PWMS][2];
	uint32_t enmask_read;
};

volatile struct pwm_cmd *sharedMem_cmd;

void timer_tick (void);
void set_freq (uint32_t chmask, uint16_t freq_hz);
void enable_ch (uint8_t ch);
void disable_ch (uint8_t ch);
void write_ch (uint8_t ch, uint16_t period_us);
uint16_t read_ch (uint8_t ch);
void read2 (uint16_t* period_us, uint8_t len);
void cork(void);


bool corked = false;
uint16_t pending[MAX_PWMS];
uint32_t pending_mask;


int main(int argc, char **argv, char **envp)
{
	int mem_fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC);
	if (mem_fd == -1)
	{
		printf("Unable to open /dev/mem");
		return -1;
	}

	sharedMem_cmd = (struct pwm_cmd *) mmap(0, 0x1000, PROT_READ|PROT_WRITE,
                                            MAP_SHARED, mem_fd, RCOUT_PRUSS_SHAREDRAM_BASE);
	if (sharedMem_cmd == NULL)
	{
		printf("Alta-RCOutput_PRU::init() PRU memory map failed.\n");
		return -1;
	}

//	close(mem_fd);

    	// All outputs default to 50Hz, the top level vehicle code
	// overrides this when necessary
    	set_freq (0xFFFFFFFF, 50);

	printf("Alta-RCOutput_PRU::init() PRU memory map complete.\n");

	set_freq (0xFF, ESC_HZ);

	for (uint8_t i = 0; i< PWM_CHANS_USED; i++)
	{
        	enable_ch (i);
	}

	uint32_t loops = 25000;
	while(loops--)
	{
		usleep (50000);
		timer_tick ();
	};

	close (mem_fd);
}

void timer_tick (void)
{
	static uint16_t pwm = 1500;
	static int8_t delta = 1;
	uint16_t read_pwm;

	for (uint8_t i=0; i < PWM_CHANS_USED; i++)
	{
		write_ch (i, pwm);
		pwm += delta;
		if (delta > 0 && pwm >= 2000)
		{
			delta = -1;
			printf ("reversing\n");
		}
		else if (delta < 0 && pwm <= 1000)
		{
			delta = 1;
			printf ("reversing\n");
		}
	}
//	read_pwm = 0;
//	read_pwm = read_ch (2);
//	printf("Channel[2] read back = %d\n", read_pwm);
//	read_pwm = 0;
//	read_pwm = read_ch (6);
//	printf("Channel[6] read back = %d\n", read_pwm);
}

void set_freq (uint32_t chmask, uint16_t freq_hz)            // LSB corresponds to CHAN_1
{
	printf ("Alta-RCOutput_PRU::set_freq(%d, %d).\n", chmask, freq_hz);
	uint8_t i;
	unsigned long tick = TICK_PER_S/(unsigned long)freq_hz;

	for (i=0;i<PWM_CHANS_USED;i++)
	{
		if (chmask & (1U<<i))
		{
			sharedMem_cmd->periodhi[chan_pru_map[i]][0] = tick;
		}
	}
}

void enable_ch (uint8_t ch)
{
	printf ("Alta-RCOutput_PRU::enable_ch(%d).\n", ch);
	sharedMem_cmd->enmask |= 1U<<chan_pru_map[ch];
}

void disable_ch (uint8_t ch)
{
	printf ("Alta-RCOutput_PRU::disable_ch(%d).\n", ch);
	sharedMem_cmd->enmask &= !(1U<<chan_pru_map[ch]);
}

void write_ch (uint8_t ch, uint16_t period_us)
{
//	if (corked)
//	{
//		pending[ch] = period_us;
//		pending_mask |= (1U << ch);
//	}
//	else
//	{
		sharedMem_cmd->periodhi[chan_pru_map[ch]][1] = TICK_PER_US*period_us;
//	}
}

uint16_t read_ch (uint8_t ch)
{
	return (sharedMem_cmd->hilo_read[chan_pru_map[ch]][1]/TICK_PER_US);
}

void read2 (uint16_t* period_us, uint8_t len)
{
	uint8_t i;
	if (len>PWM_CHANS_USED)
	{
        	len = PWM_CHANS_USED;
	}
	for (i=0;i<len;i++)
	{
		period_us[i] = sharedMem_cmd->hilo_read[chan_pru_map[i]][1]/TICK_PER_US;
	}
}


void cork(void)
{
	corked = true;
}
