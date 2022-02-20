/**
 * Copyright (c) 2015 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic"
static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

#define DISPLAY_RX_BUSY_FLAG (0)
#if defined(DISPLAY_RX_BUSY_FLAG) && (DISPLAY_RX_BUSY_FLAG)
static uint32_t      m_rx_wait_loops = 0;
#endif

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.");
    if (m_rx_buf[0] != 0)
    {
        NRF_LOG_INFO(" Received:");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}

/* -------------------------------------------------------- */
/* EA DOGS164B-A LCD MOD 64DIG 16X4 TRANSMISV WHT */

static int display_xfer(uint8_t rw, uint8_t rs, uint8_t data)
{
    memset(m_tx_buf, 0, 3);
    m_tx_buf[0] = 0xf8 | ((rw&1) << 2) |( (rs&1) << 1);
    m_tx_buf[1] = ((data&1) << 7) | ((data&2) << 5) | ((data&4) << 3) | ((data&8) << 1);
    m_tx_buf[2] = ((data&0x10) << 3) | ((data&0x20) << 1) | ((data&0x40) >> 1) | ((data&0x80) >> 3);

        spi_xfer_done = false;
        uint8_t sz = 3;
#if defined(DISPLAY_RX_BUSY_FLAG) && (DISPLAY_RX_BUSY_FLAG)
        if ( rw ) { /* for read */
            m_tx_buf[1] = 0; /* when reading, the mosi must be 0 on the data byte */
            m_rx_buf[0] = 0;
            m_rx_wait_loops = 0;
            sz = 2;
        }
#endif
        ret_code_t rc = nrf_drv_spi_transfer(&spi, m_tx_buf, sz, m_rx_buf, sz);
        if (rc != NRF_SUCCESS) {
            return 1;
        }

    #define LED2_BLUE  BSP_BOARD_LED_3  // 0.12
        uint32_t inner_loop_count = 0;
        while (!spi_xfer_done)
        {
            /* blink blue if it hits here by end of 200ms */
            if ( ((++inner_loop_count)%200) == 0 ) {
                bsp_board_led_invert(LED2_BLUE);
                if ( (inner_loop_count%2000) == 0 ) {
                    NRF_LOG_INFO("SPI example inner loop blink %u", inner_loop_count);
                    NRF_LOG_PROCESS();
                }
            }
#if !(defined(DISPLAY_RX_BUSY_FLAG) && (DISPLAY_RX_BUSY_FLAG))
            nrf_delay_ms(1); /* tx delay 1ms when waiting */
        }
#else
            #if 0 /* no tx delay 1ms when waiting */
            ;
            #elif 1 /* tx delay 1ms when waiting */
            if ( rw == 0 ) /* delay by 1ms only when writing */
                nrf_delay_ms(1); /* tx delay 1ms when waiting */
            #elif 1 /* rx delay 1ms when waiting too */
            nrf_delay_ms(1); /* tx delay 1ms when waiting */
            #endif
        }
        if ( rw ) m_rx_wait_loops = inner_loop_count;
#endif
    #undef LED2_BLUE

    return 0;
}

static int display_sequence_repeat()
{
    /* LED1 does not blink. LED2 ok. */
    #define LED2_GREEN BSP_BOARD_LED_2  // 1.9

    uint8_t seqd[] = {0x80};
    uint32_t seq_sz = sizeof(seqd);

    int rc1 = 0;

  while(1) {
    uint32_t seq_idx = 0;
    for ( seq_idx = 0; seq_idx < seq_sz; seq_idx ++)
    {
        int rc2 = display_xfer(0, 0, seqd[seq_idx]);
        if ( rc2 != 0 ) {
            rc1 = 1;
            NRF_LOG_INFO("SPI sequence  %u  rc2 %d  failed", seq_idx, rc2);
            break;
        }
        bsp_board_led_invert(LED2_GREEN);
        nrf_delay_ms(200);
        bsp_board_led_invert(LED2_GREEN);
        nrf_delay_ms(200);

        NRF_LOG_INFO("SPI sequqnce  %u  rc2 %d", seq_idx, rc2);
        NRF_LOG_PROCESS();
    }
  }

    #undef LED2_GREEN

    return rc1;
}

static int display_sequence_run(uint8_t *seqd, uint32_t seq_sz)
{
    int rc1 = 0;
    /* LED1 does not blink. LED2 ok. */
    #define LED2_GREEN BSP_BOARD_LED_2  // 1.9
    uint32_t seq_idx = 0;
    for ( seq_idx = 0; seq_idx < seq_sz; seq_idx ++) {
        int rc2 = display_xfer(0, 0, seqd[seq_idx]);
        if ( rc2 != 0 ) {
            rc1 = 1;
            NRF_LOG_INFO("SPI sequence  %u  wr  rc2 %d  failed", seq_idx, rc2);
            NRF_LOG_FLUSH();
            break;
        }

#if defined(DISPLAY_RX_BUSY_FLAG) && (DISPLAY_RX_BUSY_FLAG)
        /* read busy flag */
        uint32_t rd_loop = 0;
        int found_rd_ready = 0; /* not found yet */
        for (rd_loop = 0; rd_loop < 1000; rd_loop ++ ) { /* loop max 1000 * 1ms delay = 1 sec */
            int rc3 = display_xfer(1/*read*/, 0/*rs=0*/, seqd[seq_idx]/* dummy data for read */);
            if ( rc3 != 0 ) {
                rc1 = 1;
                NRF_LOG_INFO("SPI sequence  %u  rd  rc3 %d  failed", seq_idx, rc3);
                NRF_LOG_FLUSH();
                break;
            }
            NRF_LOG_INFO("SPI sequence  %u  rd  rc3 %d  data 0x%08x xfer rx wait loops %u", 
                            seq_idx, rc3, m_rx_buf[0], m_rx_wait_loops);
                            /* at freq 1M, no tx delay 1ms when waiting, rx wait loops is about 29. 
                             * at freq 125K, no tx delay 1ms when waiting, rx wait loops is about 237.
                             * add back tx delay 1ms when waiting, rx wait loops is about 202.
                             # enable rx delay 1ms when waiting, rx wait loops is about 1.
                             */
            NRF_LOG_PROCESS();
            if ( (m_rx_buf[0] & 1) == 0 ) { /* b7 busy flag, check b0 on little-endian spi rx */
                found_rd_ready = 1; /* controller finished processing */
                break;
            }
            nrf_delay_ms(1);
        }
        if ( rc1 ) break;
        if ( found_rd_ready == 0 ) {
            rc1 = 1;
            NRF_LOG_INFO("SPI sequence  %u  rd  wait  failed", seq_idx);
            NRF_LOG_FLUSH();
            break;
        }
        NRF_LOG_INFO("SPI sequence  %u  rd  wait  loop %u", seq_idx, rd_loop);
        NRF_LOG_FLUSH();
#endif

        bsp_board_led_invert(LED2_GREEN);
        nrf_delay_ms(10);
        bsp_board_led_invert(LED2_GREEN);
        nrf_delay_ms(10);

        NRF_LOG_INFO("SPI sequqnce  %u  rc2 %d", seq_idx, rc2);
        NRF_LOG_FLUSH();
    }
    #undef LED2_GREEN
    return rc1;
}

static int display_sequence_init()
{
    uint8_t seqd[] = {0x3a, 0x09, 0x06, 0x1e, 0x39, 0x1b, 0x6c, 
                        0x56, //0x56, /* bit 1:0==c5:c4 */
                        0x7a, /* bit 3:0==c3:c0 */
                        0x38, 
                            /* rotate by 180 degree */
                            //0x3a, 
                            //0x05, /* 0x06==bottom_view, 0x05==top_view */
                            //0x38, 
                        0x0f};
    uint32_t seq_sz = sizeof(seqd);
    return display_sequence_run(seqd, seq_sz);
}

static int display_sequence_contrast_set(uint8_t val)
{
    uint8_t seqd[] = {0x39, 
                        0x54 | ((val & 0x30) >> 4), //0x56, /* bit 1:0==c5:c4 */
                        0x70 | ((val & 0x0f)     ), //0x7a, /* bit 3:0==c3:c0 */
                        0x38};
    uint32_t seq_sz = sizeof(seqd);
    return display_sequence_run(seqd, seq_sz);
}

static int display_sequence_rom_a_select()
{
    uint8_t seqd[] = {0x3a, 0x72, 
                        0x00, /* 0x00 rom a, 0x04 rom b, 0x08/0x0c rom c */ 
                        0x38};
    uint32_t seq_sz = sizeof(seqd);
    return display_sequence_run(seqd, seq_sz);
}

static int display_sequence_return_home()
{
    uint8_t seqd[] = {0x39, 0x2, 0x38, 0x0f};
    uint32_t seq_sz = sizeof(seqd);
    return display_sequence_run(seqd, seq_sz);
}

static int display_sequence_contrast_sweep()
{
    int rc = 0;
    NRF_LOG_INFO("  CONTRAST sweep ...");
    NRF_LOG_PROCESS();
    uint8_t val = 0;
    for ( val = 0; val < 32; val ++ ) {
        int rc2 = display_sequence_contrast_set(val);
        NRF_LOG_INFO("  CONTRAST set      val %u  rc2 %d", val, rc2);
        NRF_LOG_PROCESS();
        if ( rc2 != 0 ) { 
            rc = 1;
            break;
        }
        nrf_delay_ms(999);
        NRF_LOG_PROCESS();
    }
    NRF_LOG_INFO("  CONTRAST sweep ... done");
    NRF_LOG_PROCESS();
    return rc;
}

static int display_sequence()
{
    int rc = 0;

    /* init */
    NRF_LOG_INFO("    SEQUENCE init ... ");
    NRF_LOG_PROCESS();
    rc = display_sequence_init();
    NRF_LOG_INFO("    SEQUENCE init ... done rc %d ", rc);
    NRF_LOG_PROCESS();
    if ( rc != 0 ) return 1;

    /* contrast  val 31 */
    uint8_t val = 31;
    rc = display_sequence_contrast_set(val);
    NRF_LOG_INFO("  CONTRAST set      val %u  rc %d", val, rc);
    NRF_LOG_PROCESS();

    /* display function set: 
     *      0x38  :  RE=0, IS=0
     *      0x39  :  RE=0, IS=1
     *      0x3a  :  RE=1, REV=0
     *      0x3b  :  RE=1, REV=1
     * clear display:
     *      RE/IS=x/x rs/rw=0/0 data=1
     * return home:
     *      RE/IS=0/x rs/rw=0/0 data=2
     * entry mode:
     *      RE/IS=0/x rs/rw=0/0 data bit7:3=0, bit2=1, bit1=(high=move_right), bit0=(0=no_shift)
     * set ddram address:
     *      RE/IS=0/x rs/rw=0/0 data bit7=1, bit6:0=ac6:0
     * write data:
     *      RE/IS=x/x rs/rw=1/0 data=data
     * rom selection
     *      RE/IS=x/x rs/rw=1/0 data=(0x00 select rom A)
     */

    nrf_delay_ms(100); NRF_LOG_PROCESS();

    //rc = display_sequence_rom_a_select();
    //NRF_LOG_INFO("  ROM-A             rc %d", rc); NRF_LOG_PROCESS();
    //nrf_delay_ms(10);
    //if ( rc != 0 ) return 1;

    rc = display_sequence_return_home();
    NRF_LOG_INFO("  HOME              rc %d", rc); NRF_LOG_PROCESS();
    nrf_delay_ms(10);
    if ( rc != 0 ) return 1;

    //rc = display_xfer(0, 0, 6);
    //NRF_LOG_INFO("  ENTRY             rc %d", rc); NRF_LOG_PROCESS();
    //nrf_delay_ms(10);
    //if ( rc != 0 ) return 1;

    /* clear screen */
    rc = display_xfer(0, 0, 1);
    NRF_LOG_INFO("  CLEAR             rc %d", rc); NRF_LOG_PROCESS();
    nrf_delay_ms(10);
    if ( rc != 0 ) return 1;

    /* ddram 0 */
    rc = display_xfer(0, 0, 0x80);
    NRF_LOG_INFO("  DDRAM 0x00        rc %d", rc); NRF_LOG_PROCESS();
    nrf_delay_ms(10);
    if ( rc != 0 ) return 1;

    NRF_LOG_INFO("  WRITE data line 1"); NRF_LOG_PROCESS();

    int wr_idx = 0;
    for ( wr_idx = 0; wr_idx < 4; wr_idx ++ ) {
        rc = display_xfer(0, 1/*rs*/, 'A' + wr_idx);
        NRF_LOG_INFO("  WRITE data %d", wr_idx); NRF_LOG_PROCESS();
        nrf_delay_ms(10);
        if ( rc != 0 ) {
            return 1;
        }
        NRF_LOG_PROCESS();
    }

    /* ddram 0x20 */
    rc = display_xfer(0, 0, 0x80 + 0x20);
    NRF_LOG_INFO("  DDRAM 0x20        rc %d", rc); NRF_LOG_PROCESS();
    nrf_delay_ms(10);
    if ( rc != 0 ) return 1;

    NRF_LOG_INFO("  WRITE data line 2"); NRF_LOG_PROCESS();

    for ( wr_idx = 0; wr_idx < 4; wr_idx ++ ) {
        rc = display_xfer(0, 1/*rs*/, 'a' + wr_idx);
        NRF_LOG_INFO("  WRITE data %d", wr_idx); NRF_LOG_PROCESS();
        nrf_delay_ms(10);
        if ( rc != 0 ) {
            return 1;
        }
        NRF_LOG_PROCESS();
    }

    /* ddram 0x48 */
    rc = display_xfer(0, 0, 0x80 + 0x48);
    NRF_LOG_INFO("  DDRAM 0x20        rc %d", rc); NRF_LOG_PROCESS();
    nrf_delay_ms(10);
    if ( rc != 0 ) return 1;

    NRF_LOG_INFO("  WRITE data line 3"); NRF_LOG_PROCESS();

    for ( wr_idx = 0; wr_idx < 4; wr_idx ++ ) {
        rc = display_xfer(0, 1/*rs*/, 'a' + wr_idx);
        NRF_LOG_INFO("  WRITE data %d", wr_idx); NRF_LOG_PROCESS();
        nrf_delay_ms(10);
        if ( rc != 0 ) {
            return 1;
        }
        NRF_LOG_PROCESS();
    }

    /* ddram 0x60 */
    rc = display_xfer(0, 0, 0x80 + 0x60);
    NRF_LOG_INFO("  DDRAM 0x20        rc %d", rc); NRF_LOG_PROCESS();
    nrf_delay_ms(10);
    if ( rc != 0 ) return 1;

    NRF_LOG_INFO("  WRITE data line 4"); NRF_LOG_PROCESS();

    return 0;

    /* repeat sweep contrast */
    int i = 0;
    for ( i=0; i<1000; i++ ) {
        NRF_LOG_INFO("    SEQUENCE contrast sweep %d ... ", i);
        NRF_LOG_PROCESS();
        rc = display_sequence_contrast_sweep();
        NRF_LOG_INFO("    SEQUENCE contrast sweep %d ... done rc %d ", i, rc);
        NRF_LOG_PROCESS();
        if ( rc != 0 ) return 1;
    }
    return 0;
}
/* -------------------------------------------------------- */

int main(void)
{
    bsp_board_init(BSP_INIT_LEDS);

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    spi_config.mode     = NRF_DRV_SPI_MODE_3;
    spi_config.frequency = NRF_SPI_FREQ_125K;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

    NRF_LOG_INFO("SPI example started.");
    NRF_LOG_INFO("SPI pins 20 32 31 13: %d %d %d %d", SPI_SS_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SCK_PIN);
    NRF_LOG_PROCESS();

    /* use nfc pins */
    nrf_gpio_cfg_output(9);
    nrf_gpio_cfg_output(10);
    nrf_gpio_pin_set(9);  /* init to high */
    nrf_gpio_pin_set(10); /* init to high */

        nrf_delay_ms(50); /* +50ms after power up */
        nrf_gpio_pin_clear(9);
        nrf_gpio_pin_clear(10);

        nrf_delay_ms(20); /* reset negative pulse 20ms */
        nrf_gpio_pin_set(9);  /* set high again */
        nrf_gpio_pin_set(10); /* set high again */

    uint32_t loop_count = 0;
    NRF_LOG_INFO("SPI example loop count %u", ++loop_count);
    NRF_LOG_PROCESS();

    /* LED1 does not blink. LED2 ok. */
    #define LED1_GREEN BSP_BOARD_LED_0  // 0.6
    #define LED2_RED   BSP_BOARD_LED_1  // 0.8
    #define LED2_GREEN BSP_BOARD_LED_2  // 1.9
    #define LED2_BLUE  BSP_BOARD_LED_3  // 0.12

    int rc = display_sequence();
    while (1)
    {
        if ( rc == 0 ) {
            bsp_board_led_invert(LED2_GREEN);
        } else {
            bsp_board_led_invert(LED2_RED);
        }
        nrf_delay_ms(800);
        if ( rc == 0 ) {
            bsp_board_led_invert(LED2_GREEN);
        } else {
            bsp_board_led_invert(LED2_RED);
        }
        nrf_delay_ms(800);
        NRF_LOG_FLUSH();
    }

    while (1)
    {
        // Reset rx buffer and transfer done flag
        memset(m_rx_buf, 0, m_length);
        spi_xfer_done = false;

        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));

        uint32_t inner_loop_count = 0;
        while (!spi_xfer_done)
        {
            //__WFE();
            /* blink blue if it hits here by end of 200ms */
            if ( ((++inner_loop_count)%20) == 0 ) {
                bsp_board_led_invert(LED2_BLUE);
                if ( (inner_loop_count%200) == 0 ) {
                    NRF_LOG_INFO("SPI example inner loop blink %u", inner_loop_count);
                    NRF_LOG_PROCESS();
                }
            }
            nrf_delay_ms(10);
        }

        NRF_LOG_INFO("SPI example loop count %u", ++loop_count);
        NRF_LOG_FLUSH();

        //bsp_board_led_invert(BSP_BOARD_LED_0);
        bsp_board_led_invert(LED2_RED);
        nrf_delay_ms(200);
        /* also blink green */
        bsp_board_led_invert(LED2_GREEN);
    }
}
