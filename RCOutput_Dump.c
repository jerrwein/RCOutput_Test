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

//static const uint8_t chan_pru_map[]= {0,1,5,6,2,3,4,7,8,9,10,11};
/* Ch1: PRU1.0 - P8.45, Ch2: PRU1.1 P8.46,
 * Ch3: PRU1.2 - P8.43, Ch4: PRU1.3 P8.44,
 * Ch5: PRU1.4 - P8.41, Ch6: PRU1.5 P8.42
 */

#define RCOUT_PRU0_SHAREDRAM_BASE     0x4a312000
#define RCOUT_PRU1_SHAREDRAM_BASE     0x4a310000
#define MAX_CMD_PWMS             12
#define PWM_CMD_MAGIC            0xf00fbaaf
#define PWM_REPLY_MAGIC          0xbaaff00f
#define PWM_CMD_CONFIG	         0	/* full configuration in one go */
#define PWM_CMD_ENABLE	         1	/* enable a pwm */
#define PWM_CMD_DISABLE	         2	/* disable a pwm */
#define PWM_CMD_MODIFY	         3	/* modify a pwm */
#define PWM_CMD_SET	         4	/* set a pwm output explicitly */
#define PWM_CMD_CLR	         5	/* clr a pwm output explicitly */
#define PWM_CMD_TEST	         6	/* various crap */

//static const int TICK_PER_US = 200;
//static const int TICK_PER_S = 200000000;

struct pwm_cmd
{
	uint32_t magic;
	uint32_t enmask;     /* enable mask */
	uint32_t offmsk;     /* state when pwm is off */
	uint32_t periodhi[MAX_CMD_PWMS][2];
	uint32_t hilo_read[MAX_CMD_PWMS][2];
	uint32_t enmask_read;
};

struct ring_buf
{
	uint16_t ring_head;
 	uint16_t ring_tail;
	struct
	{
		uint16_t pin_value;
		uint16_t delta_t;
	} buffer[25];
};


volatile struct pwm_cmd *sharedMem_cmd;
volatile struct ring_buf *sharedMem_ring_buf;

void timer_tick (void);
void set_freq (uint32_t chmask, uint16_t freq_hz);
void enable_ch (uint8_t ch);
void disable_ch (uint8_t ch);
void write_ch (uint8_t ch, uint16_t period_us);
uint16_t read_ch (uint8_t ch);
void read2 (uint16_t* period_us, uint8_t len);
void cork(void);


bool corked = false;
uint16_t pending[MAX_CMD_PWMS];
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
                                            MAP_SHARED, mem_fd, RCOUT_PRU1_SHAREDRAM_BASE);
	if (sharedMem_cmd == NULL)
	{
		printf("Unable to map PRU-1 shared memory");
	}

	sharedMem_ring_buf = (struct ring_buf *) mmap(0, 0x1000, PROT_READ|PROT_WRITE,
                                            MAP_SHARED, mem_fd, RCOUT_PRU0_SHAREDRAM_BASE);
	if (sharedMem_ring_buf == NULL)
        {
                printf("Unable to map PRU-0 shared memory");
        }

//	close(mem_fd);

    	// All outputs default to 50Hz, the top level vehicle code
	// overrides this when necessary

	printf("Alta-RCOutput_PRU::init() PRU memory map complete.\n");

	uint32_t mask;
	uint32_t period;
	uint32_t period_hi;

	mask = sharedMem_cmd->enmask;
	printf("Alta-RCOutput_Dump(): Enable Mask = 0x%08X\n", mask);
	mask = sharedMem_cmd->offmsk;
	printf("Alta-RCOutput_Dump(): Off Mask = 0x%08X\n", mask);

	for (uint8_t i = 0; i< MAX_CMD_PWMS; i++)
	{
 		period = sharedMem_cmd->periodhi[i][0];
 		period_hi = sharedMem_cmd->periodhi[i][1];
		// sharedMem_cmd->hilo_read[chan_pru_map[i]][1]/TICK_PER_US;
		printf("Alta-RCOutput_Dump(%d): period[0] = %d, period_hi[1] = %d\n", i, period, period_hi);
	}

	for (uint8_t i = 0; i< MAX_CMD_PWMS; i++)
	{
 		period = sharedMem_cmd->hilo_read[i][0];
 		period_hi = sharedMem_cmd->hilo_read[i][1];
		printf("Alta-RCOutput_Dump(%d): read_period[0] = %d, read_period_hi[1] = %d\n", i, period, period_hi);
	}


    	uint16_t head = sharedMem_ring_buf->ring_head;
    	uint16_t tail = sharedMem_ring_buf->ring_tail;
    	uint16_t pin_a;	//  = sharedMem_ring_buf->buffer[0].pin_value;
    	uint16_t delta_a; //  = sharedMem_ring_buf->buffer[0].delta_t;

	printf("Alta-RCOutput_Dump(44): ring_buf.head = %d, ring_buf.tail = %d\n", head, tail);
//	printf("Alta-RCOutput_Dump(45): ring_buf.buf[0].pin = %d, ring_buf.buf[0].delta = %d\n", pin_a, delta_a);

	for (uint8_t i = 0; i<40; i++)
	{
		pin_a  = sharedMem_ring_buf->buffer[i].pin_value;
		delta_a  = sharedMem_ring_buf->buffer[i].delta_t;
		printf("Alta-RCOutput_Dump(%d): ring_buf.buf[%d].pin = %d, ring_buf.buf[%d].delta = %d\n", i, i, pin_a, i, delta_a);
	}

	close (mem_fd);
}

#if 0
void enable_ch (uint8_t ch)
{
	printf("Alta-RCOutput_PRU::enable_ch(%d).\n", ch);
	sharedMem_cmd->enmask |= 1U<<chan_pru_map[ch];
}

void disable_ch (uint8_t ch)
{
	printf("Alta-RCOutput_PRU::disable_ch(%d).\n", ch);
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
#endif
