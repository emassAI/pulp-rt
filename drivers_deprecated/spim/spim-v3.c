/*
 * Copyright (C) 2018 ETH Zurich, University of Bologna and GreenWaves Technologies
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


/* 
 * Authors: Germain Haugou, ETH (germain.haugou@iis.ee.ethz.ch)
 *          Jie Chen, GreenWaves Technologies (jie.chen@greenwaves-technologies.com)
 */

#include "rt/rt_api.h"
#include <stdint.h>

static int __rt_spim_open_count[ARCHI_UDMA_NB_SPIM];

static RT_FC_TINY_DATA rt_periph_spim_t __rt_spim_periph[ARCHI_UDMA_NB_SPIM];

typedef struct {
    unsigned int cmd[4];
} rt_spim_cmd_t;

static inline int __rt_spim_id(int periph_id)
{
  return periph_id - ARCHI_UDMA_SPIM_ID(0);
}

static int __rt_spi_get_div(int spi_freq)
{
  int periph_freq = __rt_freq_periph_get();

  if (spi_freq >= periph_freq) return 0;

  return ((periph_freq + spi_freq) / spi_freq - 1) >> 1;
}

static inline int __rt_spim_get_byte_align(int wordsize, int big_endian)
{
  return wordsize == RT_SPIM_WORDSIZE_32 && big_endian;
}

rt_spim_t *rt_spim_open(char *dev_name, rt_spim_conf_t *conf, rt_event_t *event)
{
  int irq = rt_irq_disable(); // critical section

  rt_spim_conf_t def_conf;

  if (conf == NULL)
  {
    conf = &def_conf;
    rt_spim_conf_init(conf);
  }

  int channel = -1;

  if (conf->id != -1)
  {  
    rt_trace(RT_TRACE_DEV_CTRL, "[SPIM] Opening spim device (id: %d)\n", conf->id);
    channel = ARCHI_UDMA_SPIM_ID(conf->id);
  }
  else if (dev_name != NULL)
  {
    rt_trace(RT_TRACE_DEV_CTRL, "[SPIM] Opening spim device (name: %s)\n", dev_name);
  
    rt_dev_t *dev = rt_dev_get(dev_name);
    if (dev == NULL) goto error;

    channel = dev->channel;
  }

  if (channel == -1) goto error;

  rt_spim_t *spim = rt_alloc(RT_ALLOC_FC_DATA, sizeof(rt_spim_t)); // note: ALLOC_FC_DATA 
  if (spim == NULL) goto error;

  spim->channel = channel;

  spim->wordsize = conf->wordsize; // default (below) is WORDSIZE_8 !
  spim->big_endian = conf->big_endian; // default (below) is 0
  spim->polarity = conf->polarity;
  spim->phase = conf->phase;
  spim->max_baudrate = conf->max_baudrate;
  spim->cs = conf->cs;
  spim->byte_align = __rt_spim_get_byte_align(conf->wordsize, conf->big_endian);

  int div = __rt_spi_get_div(spim->max_baudrate);
  spim->div = div;

  spim->cfg = SPI_CMD_CFG(div, conf->polarity, conf->phase);

  int id = channel - ARCHI_UDMA_SPIM_ID(0);

  __rt_spim_open_count[id]++;

  if (__rt_spim_open_count[id] == 1) // if this is the first open, then init more things!
  {
    plp_udma_cg_set(plp_udma_cg_get() | (1<<channel));
    __rt_udma_callback[channel] = __rt_spim_handle_eot;
    __rt_udma_callback_data[channel] = &__rt_spim_periph[id];
    soc_eu_fcEventMask_setEvent(ARCHI_SOC_EVENT_PERIPH_EVT_BASE(channel) + ARCHI_UDMA_SPIM_EOT_EVT);
  }

  rt_irq_restore(irq); // end of critical section

  return spim;

error:
  rt_warning("[SPIM] Failed to open spim device\n");
  return NULL;
}

void __rt_spim_control(rt_spim_t *handle, rt_spim_control_e cmd, uint32_t arg)
{
  int irq = rt_irq_disable();

  int polarity = (cmd >> __RT_SPIM_CTRL_CPOL_BIT) & 3;
  int phase = (cmd >> __RT_SPIM_CTRL_CPHA_BIT) & 3;
  int set_freq = (cmd >> __RT_SPIM_CTRL_SET_MAX_BAUDRATE_BIT) & 1;
  int wordsize = (cmd >> __RT_SPIM_CTRL_WORDSIZE_BIT) & 3;
  int big_endian = (cmd >> __RT_SPIM_CTRL_ENDIANNESS_BIT) & 3;

  if (set_freq)
  {
    handle->max_baudrate = arg;
    handle->div = __rt_spi_get_div(arg);
  }

  if (polarity) handle->polarity = polarity >> 1;
  if (phase) handle->phase = phase >> 1;
  if (wordsize) handle->wordsize = wordsize >> 1;
  if (big_endian) handle->big_endian = big_endian >> 1;


  handle->cfg = SPI_CMD_CFG(handle->div, handle->polarity, handle->phase);
  handle->byte_align = __rt_spim_get_byte_align(handle->wordsize, handle->big_endian);

  rt_irq_restore(irq);
}

void rt_spim_close(rt_spim_t *handle, rt_event_t *event)
{
  int irq = rt_irq_disable();

  int channel = handle->channel;
  int id = channel - ARCHI_UDMA_SPIM_ID(0);

  __rt_spim_open_count[id]--;

  if (__rt_spim_open_count[id] == 0)
  {
    plp_udma_cg_set(plp_udma_cg_get() & ~(1<<(channel)));

    __rt_udma_callback[channel] = NULL;
    soc_eu_fcEventMask_clearEvent(ARCHI_SOC_EVENT_PERIPH_EVT_BASE(channel) + ARCHI_UDMA_SPIM_EOT_EVT);
  }

  rt_free(RT_ALLOC_FC_DATA, handle, sizeof(handle));

  rt_irq_restore(irq);
}

static inline int __rt_spim_periph_push_waiting(rt_periph_spim_t *periph, rt_periph_copy_t *copy)
{
  if (periph->first_waiting)
    periph->last_waiting->next = copy;
  else
    periph->first_waiting = copy;
  copy->next = NULL;
  periph->last_waiting = copy;
  return 0;
}

static inline __attribute__((always_inline)) int __rt_spim_periph_push(rt_periph_spim_t *periph, rt_periph_copy_t *copy)
{
  if (likely(periph->pendings[1] == NULL))
  {
    if (periph->pendings[0] == NULL)
      periph->pendings[0] = copy;
    else
      periph->pendings[1] = copy;
    return 1;
  }
  else
  {
    return __rt_spim_periph_push_waiting(periph, copy);
  }
}

void __rt_spim_send_async(rt_spim_t *handle, void *data, size_t len, int qspi, rt_spim_cs_e cs_mode, rt_event_t *event)
{
  int spim_id = __rt_spim_id(handle->channel); // spim_id computed / based on channel ?
  int periph_id = handle->channel; // is the periph_id == channel (tx) ?
  int periph_base = hal_udma_periph_base(periph_id); // udma peripheral base ?
  int cmd_base = periph_base + ARCHI_SPIM_CMD_OFFSET; // SPI commands: sequence of 32-bit microcode ops, fetched with DMA
  int channel_base = periph_base + UDMA_CHANNEL_TX_OFFSET; // tx channel base
  int buffer_size = len/8; // # of 8-bit bytes of data to send from 'data' buffer
  rt_periph_copy_t *copy = &event->implem.copy; // copy the event ?
  rt_spim_cmd_t *cmd = (rt_spim_cmd_t *)copy->periph_data; // copy the cmd, that is, the address of spi microcode array ?
  rt_periph_spim_t *periph = &__rt_spim_periph[spim_id]; // which spi device are we using ?

  rt_trace(RT_TRACE_SPIM, "[SPIM] Send bitstream (handle: %p, buffer: %p, len: 0x%x, qspi: %d, keep_cs: %d, event: %p)\n", handle, data, len, qspi, cs_mode, event);

  int irq = rt_irq_disable(); // critical section

  copy->event = event; // copy event (again) ?

  cmd->cmd[0] = handle->cfg;  // copy configuration for spi microcode 
  cmd->cmd[1] = SPI_CMD_SOT(handle->cs); // copy configuration for spi microcode 

  /*
	Not documented in datasheet

	SPI_CMD_TX_DATA(words, wordspertrans, bitsperword, qpi, lsbfirst)

	SPI_CMD_TX_DATA_ID << 28 |
	qpi << 27 |
	lsbfirst << 26 |
	wordspertrans << 21 |
	(bitsperword - 1) << 16 | (words-1) // what is 'words'? why words-1? CHECK!

	wordspertrans: how many x-bit sized words are transmitted in a burst
		4 = 4_WORD_PER_TRANSF; 1 = 2_WORD_PER_TRANSF; 0 = 1_WORD_PER_TRANSF

	bitsperword: how many bits wide is a word transmitted over MOSI. In a 5-bit bitfield
		32 bit wide words are encoded as 0x11111 = 31 (+1) = 32;
		 8 bit wide words are encoded as 0x00111 =  7 (+1) =  8;

        qpi: 0 = spi, 1 = qspi // use 1 bit or use quad-spi, but the MOSI lines are 3, not 4: CHECK!

        lsbfirst: 0 = MSB_FIRST, 1 = LSB_FIRST
  */

  // cmd->cmd[2] = SPI_CMD_TX_DATA(len/32, SPI_CMD_1_WORD_PER_TRANSF, 32, qspi, SPI_CMD_MSB_FIRST);
  cmd->cmd[2] = SPI_CMD_TX_DATA(
	len/32,                    /* 32-bit words to transfer */ \
	SPI_CMD_1_WORD_PER_TRANSF, /* how many words per transfer (1 word == '00') */ \
	32,                        /* size of MOSI word: 8 is stored as %00111 = 7, 32 as %11111 = 31 */ \
	qspi,                      /* '0' == 1-bit spi; '1' == 3-4 bits qspi */ \
	SPI_CMD_MSB_FIRST          /* first MOSI word bit to send: 0 = MSB_FIRST, 1 = LSB_FIRST */ \
	);

  cmd->cmd[3] = SPI_CMD_EOT(1, cs_mode == RT_SPIM_CS_KEEP); // what is this command doing with CS?! CHECK!

  if (likely(__rt_spim_periph_push(periph, copy)))
  {
    int cfg = UDMA_CHANNEL_CFG_SIZE_32 | UDMA_CHANNEL_CFG_EN; // 'cfg' = 32-bit + enable, same for cmd microcode and buffer data DMAs
    plp_udma_enqueue(cmd_base, (int)cmd, 4*4, cfg); // spi base, microcode commands array, size of array in bytes, 'cfg'.
    plp_udma_enqueue(channel_base, (int)data, buffer_size, cfg); // channel_base: tx channel registers; data: pointer to tx buffer.
								 // buffer_size: lenght of data to transfer in bytes, converted from bits.
  }
  else // to be retrieved and submitted later?
  {
    copy->u.raw.val[1] = periph_base;	// the SPI registers
    copy->u.raw.val[2] = (int)cmd;	// where the microcode array is
    copy->u.raw.val[3] = 4*4;		// size of SPI cmd array of microcode: 4x 4-bytes words
    copy->u.raw.val[4] = buffer_size;	// how many 8-bit bytes of data shall SPI send
    copy->u.raw.val[5] = 0;		// what is this 0 for? is the CS pin #?
    copy->u.raw.val[6] = (int)data;	// pointer to data to send
  }

  rt_irq_restore(irq);
}

void __rt_spim_send(rt_spim_t *handle, void *data, size_t len, int qspi, rt_spim_cs_e cs_mode, rt_event_t *event)
{
  int irq = rt_irq_disable();

  rt_event_t *call_event = __rt_wait_event_prepare(event);

  __rt_spim_send_async(handle, data, len, qspi, cs_mode, call_event);

  __rt_wait_event_check(event, call_event);

  rt_irq_restore(irq);
}

void __rt_spim_receive_async(rt_spim_t *handle, void *data, size_t len, int qspi, rt_spim_cs_e cs_mode, rt_event_t *event)
{
  int spim_id = __rt_spim_id(handle->channel);
  int periph_id = handle->channel;
  int periph_base = hal_udma_periph_base(periph_id);
  int cmd_base = periph_base + ARCHI_SPIM_CMD_OFFSET;
  int channel_base = periph_base + UDMA_CHANNEL_RX_OFFSET;
  int buffer_size = len/8;
  rt_periph_copy_t *copy = &event->implem.copy;
  rt_spim_cmd_t *cmd = (rt_spim_cmd_t *)copy->periph_data;
  rt_periph_spim_t *periph = &__rt_spim_periph[spim_id];

  rt_trace(RT_TRACE_SPIM, "[SPIM] Receive bitstream (handle: %p, buffer: %p, len: 0x%x, qspi: %d, keep_cs: %d, event: %p)\n", handle, data, len, qspi, cs_mode, event);

  int irq = rt_irq_disable();

  copy->event = event;

  cmd->cmd[0] = handle->cfg;
  cmd->cmd[1] = SPI_CMD_SOT(handle->cs);
  cmd->cmd[2] = SPI_CMD_RX_DATA(len/32, SPI_CMD_1_WORD_PER_TRANSF, 32, qspi, SPI_CMD_MSB_FIRST);
  cmd->cmd[3] = SPI_CMD_EOT(1, cs_mode == RT_SPIM_CS_KEEP);

  if (__rt_spim_periph_push(periph, copy))
  {
    int cfg = UDMA_CHANNEL_CFG_SIZE_32 | UDMA_CHANNEL_CFG_EN;
    plp_udma_enqueue(channel_base, (int)data, buffer_size, cfg);
    plp_udma_enqueue(cmd_base, (int)cmd, 4*4, cfg);
  }
  else
  {
    copy->u.raw.val[1] = periph_base;
    copy->u.raw.val[2] = (int)cmd;
    copy->u.raw.val[3] = 4*4;
    copy->u.raw.val[4] = buffer_size;
    copy->u.raw.val[5] = (int)data;
    copy->u.raw.val[6] = 0;
  }

  rt_irq_restore(irq);
}

void __rt_spim_receive(rt_spim_t *handle, void *data, size_t len, int qspi, rt_spim_cs_e cs_mode, rt_event_t *event)
{
  int irq = rt_irq_disable();

  rt_event_t *call_event = __rt_wait_event_prepare(event);

  __rt_spim_receive_async(handle, data, len, qspi, cs_mode, call_event);

  __rt_wait_event_check(event, call_event);

  rt_irq_restore(irq);
}

void rt_spim_transfer_async(rt_spim_t *handle, void *tx_data, void *rx_data, size_t len, rt_spim_cs_e mode, rt_event_t *event)
{
  int spim_id = __rt_spim_id(handle->channel);
  int periph_id = handle->channel;
  int periph_base = hal_udma_periph_base(periph_id);
  int cmd_base = periph_base + ARCHI_SPIM_CMD_OFFSET;
  int rx_channel_base = periph_base + UDMA_CHANNEL_RX_OFFSET;
  int tx_channel_base = periph_base + UDMA_CHANNEL_TX_OFFSET;
  int buffer_size = len/8;
  rt_periph_copy_t *copy = &event->implem.copy;
  rt_spim_cmd_t *cmd = (rt_spim_cmd_t *)copy->periph_data;
  rt_periph_spim_t *periph = &__rt_spim_periph[spim_id];

  rt_trace(RT_TRACE_SPIM, "[SPIM] Transfering bitstream (handle: %p, tx_buffer: %p, rx_buffer: %p, len: 0x%x, keep_cs: %d, event: %p)\n", handle, tx_data, rx_data, len, mode, event);

  int irq = rt_irq_disable();

  copy->event = event;

  cmd->cmd[0] = handle->cfg;
  cmd->cmd[1] = SPI_CMD_SOT(handle->cs);
  cmd->cmd[2] = SPI_CMD_FUL(len/32, SPI_CMD_1_WORD_PER_TRANSF, 32, SPI_CMD_MSB_FIRST);
  cmd->cmd[3] = SPI_CMD_EOT(1, mode == RT_SPIM_CS_KEEP);

  if (__rt_spim_periph_push(periph, copy))
  {
    int cfg = UDMA_CHANNEL_CFG_SIZE_32 | UDMA_CHANNEL_CFG_EN;
    plp_udma_enqueue(cmd_base, (int)cmd, 4*4, cfg);
    plp_udma_enqueue(rx_channel_base, (int)rx_data, buffer_size, cfg);
    plp_udma_enqueue(tx_channel_base, (int)tx_data, buffer_size, cfg);
  }
  else
  {
    copy->u.raw.val[1] = periph_base;
    copy->u.raw.val[2] = (int)cmd;
    copy->u.raw.val[3] = 4*4;
    copy->u.raw.val[4] = buffer_size;
    copy->u.raw.val[5] = (int)rx_data;
    copy->u.raw.val[6] = (int)tx_data;
  }

  rt_irq_restore(irq);
}

void rt_spim_transfer(rt_spim_t *handle, void *tx_data, void *rx_data, size_t len, rt_spim_cs_e mode, rt_event_t *event)
{
  int irq = rt_irq_disable();

  rt_event_t *call_event = __rt_wait_event_prepare(event);

  rt_spim_transfer_async(handle, tx_data, rx_data, len, mode, call_event);

  __rt_wait_event_check(event, call_event);

  rt_irq_restore(irq);
}

void rt_spim_conf_init(rt_spim_conf_t *conf)
{
  conf->wordsize = RT_SPIM_WORDSIZE_8; // note: WORDSIZE_8? and how is this used for send/receive?
  conf->big_endian = 0;
  conf->max_baudrate = 10000000;
  conf->cs_gpio = -1;
  conf->cs = -1;
  conf->id = -1;
  conf->polarity = 0;
  conf->phase = 0;
}

RT_FC_BOOT_CODE void __attribute__((constructor)) __rt_spim_init()
{
  for (int i=0; i<ARCHI_UDMA_NB_SPIM; i++)
  {
    __rt_spim_open_count[i] = 0;    
    rt_periph_spim_t *periph = &__rt_spim_periph[i];
    periph->first_waiting = NULL;
    periph->pendings[0] = NULL;
    periph->pendings[1] = NULL;
  }
}
