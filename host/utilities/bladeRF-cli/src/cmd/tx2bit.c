/*
 * This file is part of the bladeRF project
 *
 * Copyright (C) 2013 Nuand LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "conversions.h"
#include "host_config.h"
#include "minmax.h"
#include "parse.h"
#include "rel_assert.h"
#include "rxtx_impl.h"


/* The DAC range is [-2048, 2047] */
#define SC16Q11_IQ_MIN (-2048)
#define SC16Q11_IQ_MAX (2047)

/* SCALE_NN_BFR scales the maximum expected value of 3 to the fixed point
 *   2048 / 682 = 3! */
#define SCALE_NN_BRF 682

int16_t sm_lu_table[] = {-1*SCALE_NN_BRF, -3*SCALE_NN_BRF, +1*SCALE_NN_BRF, +3*SCALE_NN_BRF};



/* 'rsm_to_bladerf' takes the 16 real two-bit sign magnitude samples that 
 * are packed into a single 32-bit 'if_samples' word and converts them 
 * to 16 individual I/Q SC16Q11 samples.
 * This conversion simply sets the imaginary portion of the sample to 
 * zero. */
#if 0
static int rsm_to_bladerf(u_int32_t if_samples, int16_t *tx_buffer_current)
{
    int         i;
    int         lu_index;
    int16_t     *tx_buf, real;

    tx_buf = tx_buffer_current;
    for ( i=0; i<16; i++ ) {
        lu_index = (if_samples >> 30) & 0x3;
        if_samples <<= 2;
        
        /*  Signed, Complex 16-bit Q11. This is the native format of the DAC data.
         *   Values in the range [-2048, 2048) are used to represent [-1.0, 1.0). N
         *   ote that the lower bound here is inclusive, and the upper bound is 
         *   exclusive. Ensure that provided samples stay within [-2048, 2047].
         *
         *   Samples consist of interleaved IQ value pairs, with I being the first 
         *   value in the pair. Each value in the pair is a right-aligned, little-endian 
         *   int16_t. The FPGA ensures that these values are sign-extended.
         *
         *  .--------------.--------------.
         *  | Bits 31...16 | Bits 15...0  |
         *  +--------------+--------------+
         *  |   Q[15..0]   |   I[15..0]   |
         *  `--------------`--------------`
         * */
       real = sm_lu_table[lu_index];         

       *tx_buf++ = 0;
       *tx_buf++ = real;
    }
    return 16;
}
#endif

/* 'rsm_to_bladerf' takes the 16 real 2-bit samples that are packed into 
 * a single 32-bit 'if_samples' word and converts them to 16 individual 
 * I/Q SC16Q11 samples.
 * This conversion simply sets the imaginary portion of the sample to 
 * zero. */

static int rsm_to_bladerf(uint32_t wcount, u_int32_t *if_samples, int16_t *tx_buffer_current)
{
    register int16_t 	*tx_buf;
    register int8_t 	j, lu_index;
    register int16_t    real;
    register uint32_t	i, samples;

		/* Each word contains 16 2-bit samples that need to be converted to 
		   16 16-bit I&Q samples. */
    tx_buf = tx_buffer_current;
    memset(tx_buf, 0, sizeof(int16_t)*2*16*wcount);

    /* This arrangement matched the arrangement of Ben's conversion
     * C program 'int8tosc16q11.c', excluding the scaling factor. */
    tx_buf++;                               /* Skip Q component */
    for ( i=0; i<wcount; i++ ) {
    	samples = *if_samples++;

        for ( j=0; j<16; j++ ) {
            lu_index = (samples >> 30) & 0x3;
            samples <<= 2;
        
            /*  Signed, Complex 16-bit Q11. This is the native format of the DAC data.
            *   Values in the range [-2048, 2048) are used to represent [-1.0, 1.0). N
            *   ote that the lower bound here is inclusive, and the upper bound is 
            *   exclusive. Ensure that provided samples stay within [-2048, 2047].
            *
            *   Samples consist of interleaved IQ value pairs, with I being the first 
            *   value in the pair. Each value in the pair is a right-aligned, little-endian 
            *   int16_t. The FPGA ensures that these values are sign-extended.
            *
            *  .--------------.--------------.
            *  | Bits 31...16 | Bits 15...0  |
            *  +--------------+--------------+
            *  |   Q[15..0]   |   I[15..0]   |
            *  `--------------`--------------`
            * */
            real = sm_lu_table[lu_index];
            *tx_buf = real;
            tx_buf += 2;
        }
    }
    return 16*wcount;
}

/* 'csm_to_bladerf' takes the 8 complex two-bit sign magnitude samples that 
 * are packed into a single 32-bit 'if_samples' word and converts them 
 * to 4 individual I/Q SC16Q11 samples. */

#if 0
static int csm_to_bladerf(u_int32_t if_samples, int16_t *tx_buffer_current)
{
    int         i;
    int         lu_index;
    int16_t     *tx_buf, real, imag;

    tx_buf = tx_buffer_current;
    for ( i=0; i<8; i++ ) {
        lu_index = (if_samples >> 30) & 0x3;
        if_samples <<= 2;
        imag = sm_lu_table[lu_index];
        lu_index = (if_samples >> 30) & 0x3;
        if_samples <<= 2;
        real = sm_lu_table[lu_index];

       *tx_buf++ = imag;         
       *tx_buf++ = real;
        
    }
    return 8;
}
#endif

/* 'csm_to_bladerf' takes the 8 complex 2-bit samples that are packed into 
 * a single 32-bit 'if_samples' word and converts them to 8 individual 
 * I/Q SC16Q11 samples. */

static int csm_to_bladerf(uint32_t wcount, u_int32_t *if_samples, int16_t *tx_buffer_current)
{
    register int16_t 	*tx_buf;
    register int8_t 	j, lu_index;
    register int16_t    real, imag;
    register uint32_t	i, samples;

		/* Each word contains 8 complex 2-bit samples that need to be converted to 
		   8 16-bit I&Q samples. */
    tx_buf = tx_buffer_current;
    memset(tx_buf, 0, sizeof(int16_t)*2*8*wcount);

    /* This arrangement matched the arrangement of Ben's conversion
     * C program 'int8tosc16q11.c', excluding the scaling factor. */
    for ( i=0; i<wcount; i++ ) {
    	samples = *if_samples++;

        for ( j=0; j<8; j++ ) {
            lu_index = (samples >> 30) & 0x3;
            samples <<= 2;
            real = sm_lu_table[lu_index];

            lu_index = (samples >> 30) & 0x3;
            samples <<= 2;
            imag = sm_lu_table[lu_index];
        
            /*  Signed, Complex 16-bit Q11. This is the native format of the DAC data.
            *   Values in the range [-2048, 2048) are used to represent [-1.0, 1.0). N
            *   ote that the lower bound here is inclusive, and the upper bound is 
            *   exclusive. Ensure that provided samples stay within [-2048, 2047].
            *
            *   Samples consist of interleaved IQ value pairs, with I being the first 
            *   value in the pair. Each value in the pair is a right-aligned, little-endian 
            *   int16_t. The FPGA ensures that these values are sign-extended.
            *
            *  .--------------.--------------.
            *  | Bits 31...16 | Bits 15...0  |
            *  +--------------+--------------+
            *  |   Q[15..0]   |   I[15..0]   |
            *  `--------------`--------------`
            * */
            *tx_buf++ = imag;
            *tx_buf++ = real;
        }
    }
    return 8*wcount;
}


/* 'r8_to_bladerf' takes the 4 real 8-bit samples that are packed into 
 * a single 32-bit 'if_samples' word and converts them to 4 individual 
 * I/Q SC16Q11 samples.
 * This conversion simply sets the imaginary portion of the sample to 
 * zero. 
 * REMARK: THE BYTE ORDER MAY REQUIRE MANGLING FOR THIS FUNCTION. */
#if 0
static int r8_to_bladerf_old(u_int32_t if_samples, int16_t *tx_buffer_current)
{
    register int16_t 	*tx_buf;
    register int8_t 	r1, r2, r3, r4;
    

    tx_buf = tx_buffer_current;
   
    //r1 = ((if_samples & 0xFF000000)>>24);
    //r2 = ((if_samples & 0x00FF0000)>>16);
    //r3 = ((if_samples & 0x0000FF00)>>8);
    //r4 = ((if_samples & 0x000000FF));
    r4 = (if_samples & 0x000000FF);
    if_samples >>= 8;
    r3 = (if_samples & 0x000000FF);
    if_samples >>= 8;
    r2 = (if_samples & 0x000000FF);
    if_samples >>= 8;
    r1 = (if_samples & 0x000000FF);
       
    /* This arrangement matched the arrangement of Ben's conversion
     * C program 'int8tosc16q11.c', excluding the scaling factor. */
    *tx_buf++ = 0;
    //*tx_buf++ = ((int16_t)r4*SCALE_NN_BRF);
    *tx_buf++ = (int16_t) r4 << 8;
    *tx_buf++ = 0;    
    //*tx_buf++ = ((int16_t)r3*SCALE_NN_BRF);
    *tx_buf++ = (int16_t) r3 << 8;
    *tx_buf++ = 0;
    //*tx_buf++ = ((int16_t)r2*SCALE_NN_BRF);
    *tx_buf++ = (int16_t) r2 << 8;    
    *tx_buf++ = 0;
    //*tx_buf++ = ((int16_t)r1*SCALE_NN_BRF);
    *tx_buf++ = (int16_t) r1 << 8;
    
    return 4;
}
#endif

int16_t sim_to_sc16q11(int8_t value) 
{
    int16_t output;

    if ( value==-1 )
        output = -3*SCALE_NN_BRF;
    else if ( value==-3 )
        output = -1*SCALE_NN_BRF;
    else if ( value==1 )
        output = 1*SCALE_NN_BRF;
    else /* if ( value==3 ) */
        output = 3*SCALE_NN_BRF;
    return output;
}

int16_t int8_to_sc16q11(int8_t value) 
{
    int16_t output;

    if ( value==-1 )
        output = -1*SCALE_NN_BRF;
    else if ( value==-3 )
        output = -3*SCALE_NN_BRF;
    else if ( value==1 )
        output = 1*SCALE_NN_BRF;
    else /* if ( value==3 ) */
        output = 3*SCALE_NN_BRF;
    return output;
}

/* 'r8_to_bladerf' takes the 4 real 8-bit samples that are packed into 
 * a single 32-bit 'if_samples' word and converts them to 4 individual 
 * I/Q SC16Q11 samples.
 * This conversion simply sets the imaginary portion of the sample to 
 * zero.
 * This code has been tested and works well. */

static int r8_to_bladerf(uint32_t wcount, u_int32_t *if_samples, int16_t *tx_buffer_current)
{
    register int16_t 	*tx_buf;
    register int8_t 	r1, r2, r3, r4;
    register uint32_t	i, samples;

		/* Each word contains 4 8-bit samples that need to be converted to 
		   4 16-bit I&Q samples. */
    tx_buf = tx_buffer_current;
    memset(tx_buf, 0, sizeof(int16_t)*2*4*wcount);

    /* This arrangement matched the arrangement of Ben's conversion
     * C program 'int8tosc16q11.c', excluding the scaling factor. 
     * Here a function 'sim_to_sc16q11' has been created to fix the
     * -1,-3 reversal fault that the NordNav file conversion program
     * introduces. */
    tx_buf++;
    for ( i=0; i<wcount; i++ ) {
    	samples = *if_samples++;
    	
    	r4 = (int8_t)(samples & 0x000000FF);
    	samples >>= 8;
    	r3 = (int8_t)(samples & 0x000000FF);
    	samples >>= 8;
    	r2 = (int8_t)(samples & 0x000000FF);
    	samples >>= 8;
    	r1 = (int8_t)(samples & 0x000000FF);
    	
    	#if 0
    	*tx_buf = (int16_t) r4 << 8;
    	tx_buf += 2;
    	*tx_buf = (int16_t) r3 << 8;
    	tx_buf += 2;
    	*tx_buf = (int16_t) r2 << 8;
    	tx_buf += 2;
    	*tx_buf = (int16_t) r1 << 8;
    	tx_buf += 2;
    	#else
    	*tx_buf = sim_to_sc16q11(r4);
    	tx_buf += 2;
    	*tx_buf = sim_to_sc16q11(r3);
    	tx_buf += 2;
    	*tx_buf = sim_to_sc16q11(r2);
    	tx_buf += 2;
    	*tx_buf = sim_to_sc16q11(r1);
    	tx_buf += 2;
    	#endif    	
    }
    
    return 4*wcount;
}

/* 'c8_to_bladerf' takes the 2 complex 8-bit I/Q samples that are packed 
 * into a single 32-bit 'if_samples' word and converts them to 2 individual 
 * I/Q SC16Q11 samples.
 * REMARK: THE BYTE ORDER MAY REQUIRE MANGLING FOR THIS FUNCTION. */

#if 0
static int c8_to_bladerf(u_int32_t if_samples, int16_t *tx_buffer_current)
{
    int16_t *tx_buf;
    int8_t  r1, r2, r3, r4;

    tx_buf = tx_buffer_current;
    r1 = ((if_samples & 0xFF000000)>>24);
    r2 = ((if_samples & 0x00FF0000)>>16);
    r3 = ((if_samples & 0x0000FF00)>>8);
    r4 = ((if_samples & 0x000000FF));
    
    *tx_buf++ = ((int16_t)r3 * SCALE_NN_BRF);
    *tx_buf++ = ((int16_t)r4 * SCALE_NN_BRF);
    *tx_buf++ = ((int16_t)r1 * SCALE_NN_BRF);
    *tx_buf++ = ((int16_t)r2 * SCALE_NN_BRF);
    
    return 2;
}
#endif


/* 'c8_to_bladerf' takes the 2 complex 8-bit samples that are packed into 
 * a single 32-bit 'if_samples' word and converts them to 2 individual 
 * I/Q SC16Q11 samples. */

static int c8_to_bladerf(uint32_t wcount, u_int32_t *if_samples, int16_t *tx_buffer_current)
{
    register int16_t 	*tx_buf;
    register int8_t 	r1, r2, r3, r4;
    register uint32_t	i, samples;

		/* Each word contains 2 complex 8-bit samples (I&Q) that need 
		   to be converted to 2 16-bit I&Q samples. */
    tx_buf = tx_buffer_current;
    memset(tx_buf, 0, sizeof(int16_t)*2*2*wcount);

    /* This arrangement matched the arrangement of Ben's conversion
     * C program 'int8tosc16q11.c', excluding the scaling factor. 
     * Here a function 'sim_to_sc16q11' has been created to fix the
     * -1,-3 reversal fault that the NordNav file conversion program
     * introduces. */
    for ( i=0; i<wcount; i++ ) {
    	samples = *if_samples++;
    	
    	r4 = (int8_t)(samples & 0x000000FF);
    	samples >>= 8;
    	r3 = (int8_t)(samples & 0x000000FF);
    	samples >>= 8;
    	r2 = (int8_t)(samples & 0x000000FF);
    	samples >>= 8;
    	r1 = (int8_t)(samples & 0x000000FF);
    	
    	/* SC16Q11 are organised as as Q:I whereas the input file
    	 * is assumed to be ordered as I followed by Q. 
    	 * If an IF format with a sample rate of 16.368 MHz and an
    	 * IF of 4.092 MHz is read, then successive samples will be
    	 * similar to an I,Q arrangement. In this case, the samples
    	 * can be interpreted as complex samples with 1/2 the samling
    	 * rate. This reduced the output data rate to the SDR by a 
    	 * factor of two. It should also avoid the double sideband
    	 * structure that can be seen with the spectrum analyser. */
    	*tx_buf++ = sim_to_sc16q11(r3);    	
    	*tx_buf++ = sim_to_sc16q11(r4);
    	*tx_buf++ = sim_to_sc16q11(r1);    	
    	*tx_buf++ = sim_to_sc16q11(r2);
    }
    
    return 2*wcount;
}


static int tx2_task_exec_running(struct rxtx_data *tx, struct cli_state *s)
{
    int status = 0;
    unsigned int samples_per_buffer;
    int16_t *tx_buffer;
    u_int32_t *raw_if_buffer;
    struct tx_params *tx_params = tx->params;
    unsigned int repeats_remaining;
    unsigned int delay_us;
    unsigned int delay_samples;
    unsigned int delay_samples_remaining;
    bool repeat_infinite;
    unsigned int timeout_ms;
    bladerf_sample_rate sample_rate = 0;
    int i;
    int tx_buffer_size_bytes;
    int rawsamples_per_outsample;
    int raw_if_buffer_size_bytes;
    unsigned int k, words_read;

    enum state { INIT, READ_FILE, DELAY, PAD_TRAILING, DONE };
    enum state state = INIT;

    /* Fetch the parameters required for the TX operation */
    MUTEX_LOCK(&tx->param_lock);
    repeats_remaining = tx_params->repeat;
    delay_us          = tx_params->repeat_delay;
    MUTEX_UNLOCK(&tx->param_lock);

    repeat_infinite = (repeats_remaining == 0);

    MUTEX_LOCK(&tx->data_mgmt.lock);
    samples_per_buffer = (unsigned int)tx->data_mgmt.samples_per_buffer;
    timeout_ms         = tx->data_mgmt.timeout_ms;
    MUTEX_UNLOCK(&tx->data_mgmt.lock);

    for (i = 0; i < RXTX_MAX_CHANNELS; ++i) {
        if (tx->channel_enable[i]) {
            status = bladerf_get_sample_rate(s->dev, BLADERF_CHANNEL_TX(i),
                                             &sample_rate);
            if (status != 0) {
                set_last_error(&tx->last_error, ETYPE_BLADERF, status);
                return CLI_RET_LIBBLADERF;
            }
            break;
        }
    }

    if (0 == sample_rate) {
        cli_err(s, "tx2", "Could not read sample rate\n");
        return CLI_RET_UNKNOWN;
    }

    /* Compute delay time as a sample count */
    delay_samples = (unsigned int)((uint64_t)sample_rate * delay_us / 1000000);
    delay_samples_remaining = delay_samples;

    /* To support different input file formats, this function needs to be changed
     * so that the raw input file is read into a temporary buffer after which a
     * conversion to the bladeRF format is performed.
     * The 'samples_per_buffer' variable gives the number of actual samples 
     * to be transmitted in the bladeRF SC16Q11 format. This is an inefficient 
     * format and reading from the raw input file requires a smaller read than would
     * otherwise be needed.
     * SC16Q11 represents each sample as a signed 16-bit number with a fixed point
     * at bit 11. The scaling is therefore 2^11 or 2048 giving values in the range
     * of -16.000 to +15.999511718. The bladeRF uses SC16Q11 with both I & Q samples
     * and therefore requires 32-bits for each input samples.
     * The NordNav stores its IF in real 2-bit format with values of +/-3 and +/-1.
     * The Q value could be set to 0 or a pseudo complex sample produced with 
     * successive samples. The first technique retains the original sample rate
     * and the latter halves the sample rate. However, the NordNav raw input format
     * is proprietary and it has been necessary to convert this to byte format
     * in which each sample consumes 8-bits rather than 2 bits.
     * Our own GPSR sensor is able to record 2 bit real samples directly at a 
     * high sample rate and 2-bit complex baseband signals at a lower sample rate.
     *  
     * 
     * 
     * */
 
    /* Allocate a buffer to hold each block of samples to transmit */
    switch ( tx->file_mgmt.if_fmt ) {
        case IFTX_FMT_RSM:  	rawsamples_per_outsample = 16;
                            	break;
        case IFTX_FMT_CSM:		rawsamples_per_outsample = 8;
                            	break;
        case IFTX_FMT_R8:   	rawsamples_per_outsample = 4;
                            	break;
        case IFTX_FMT_C8:   	rawsamples_per_outsample = 2;
                            	break;
        case IFTX_FMT_SC16Q11:  rawsamples_per_outsample = 1;
        						break;	
        case IFTX_FMT_LS2:  
        default:            rawsamples_per_outsample = -1;                       
    }
   if (-1 == rawsamples_per_outsample) {
        cli_err(s, "tx2", "Unsupported if_format\n");
        return CLI_RET_UNKNOWN;
    }
    tx_buffer_size_bytes = samples_per_buffer * 2 * sizeof(int16_t);
    tx_buffer = (int16_t *)malloc(tx_buffer_size_bytes);
    raw_if_buffer_size_bytes = tx_buffer_size_bytes/rawsamples_per_outsample;
    raw_if_buffer = (u_int32_t *)malloc(raw_if_buffer_size_bytes);
    if (tx_buffer == NULL) {
        status = CLI_RET_MEM;
        set_last_error(&tx->last_error, ETYPE_ERRNO,
                       errno == 0 ? ENOMEM : errno);
    }
    if (raw_if_buffer == NULL) {
        status = CLI_RET_MEM;
        set_last_error(&tx->last_error, ETYPE_ERRNO,
                       errno == 0 ? ENOMEM : errno);
    }

    /* Keep writing samples while there is more data to send and no failures
     * have occurred */
    while (state != DONE && status == 0) {
        unsigned char requests;
        unsigned int buffer_samples_remaining   = samples_per_buffer;
        unsigned int buffer_samples_remaining_b = samples_per_buffer;
        unsigned int raw_buffer_samples_remaining;
        int16_t *tx_buffer_current              = tx_buffer;
        int16_t *tx_buffer_current_b            = tx_buffer;
        u_int32_t *raw_if_buffer_current        = raw_if_buffer;
    
        /* Stop stream on STOP or SHUTDOWN, but only clear STOP. This will keep
         * the SHUTDOWN request around so we can read it when determining
         * our state transition */
        requests = rxtx_get_requests(tx, RXTX_TASK_REQ_STOP);
        if (requests & (RXTX_TASK_REQ_STOP | RXTX_TASK_REQ_SHUTDOWN)) {
            break;
        }

        /* Keep adding to the buffer until it is full or a failure occurs */
        buffer_samples_remaining = samples_per_buffer;
        buffer_samples_remaining_b = samples_per_buffer;
        raw_buffer_samples_remaining = samples_per_buffer / rawsamples_per_outsample;
        while (buffer_samples_remaining > 0 && status == 0 && state != DONE) {
            size_t samples_populated = 0;

            switch (state) {
                case INIT:
                case READ_FILE:

                    MUTEX_LOCK(&tx->file_mgmt.file_lock);

                    /* Read from the input file
                     * In the original 'tx' code, this attempted to fill the entire buffer
                     * with a single 'fread' call. Each sample requires 2 SC16Q11 samples
                     * and the read attempts to read the same number of samples as the size
                     * of the buffer.
                     * In the new arrangement, a single byte could have 4 samples or even 8
                     * samples if 1 bit sampling was used. In fact, even with 8 bit samples
                     * there will be two byte samples for each bladeRF sample.
                     * 
                     * To support different input file formats, this portion needs to 
                     * be changed so that 'samples_populated' reflects the actual number
                     * of samples being added each time and a conversion to bladeRF
                     * format needs to be performed. 
                    */
                    #if 0
                    //samples_populated =
                    //    fread(tx_buffer_current, 2 * sizeof(int16_t),
                    //          buffer_samples_remaining, tx->file_mgmt.file);
                    #endif
                    // words_read =
                    //    fread(raw_if_buffer_current, sizeof(u_int32_t),
                    //          raw_if_buffer_size_bytes/sizeof(u_int32_t), 
                    //          tx->file_mgmt.file);
                    words_read =
                        fread(raw_if_buffer_current, sizeof(u_int32_t),
                              raw_buffer_samples_remaining, 
                              tx->file_mgmt.file);
                        
                    assert(words_read <= UINT_MAX);

                    /* Convert the 'raw_if_buffer' elements to bladeRF samples 
                     *  placing the result in 'tx_buffer_current'. */
                   k = words_read;
                   while ( k>0 ) {
                       switch (tx->file_mgmt.if_fmt ) {
                            case IFTX_FMT_RSM:      /* VALIDATED */

                                samples_populated += rsm_to_bladerf(words_read, raw_if_buffer_current, 
                                    tx_buffer_current_b);
                                buffer_samples_remaining_b -= (16*words_read);
                                tx_buffer_current_b += (16*2*words_read);    /* '16 * words_read' BladeRF samples */
                                raw_if_buffer_current += words_read;
                                k -= words_read;
                                break;

                            case IFTX_FMT_CSM:  
                                samples_populated += csm_to_bladerf(words_read, raw_if_buffer_current, 
                                    tx_buffer_current_b);
                                buffer_samples_remaining_b -= (8*words_read);
                                tx_buffer_current_b += (8*2*words_read);    /* '8 * words_read' BladeRF samples */
                                raw_if_buffer_current += words_read;
                                k -= words_read;
                                break;

                            case IFTX_FMT_R8:       /* VALIDATED */
                                samples_populated += r8_to_bladerf(words_read, raw_if_buffer_current,
                                    tx_buffer_current_b);
                                buffer_samples_remaining_b -= (4*words_read);
                                tx_buffer_current_b += (4*2*words_read);    /* '4 * words_read' BladeRF samples */
                                raw_if_buffer_current += words_read;
                                k -= words_read;
                                break;

                            case IFTX_FMT_C8:
                                samples_populated += c8_to_bladerf(words_read, raw_if_buffer_current,
                                    tx_buffer_current_b);
                                buffer_samples_remaining_b -= (2*words_read);
                                tx_buffer_current_b += (2*2*words_read);    /* '2 * words_read' BladeRF samples */
                                raw_if_buffer_current += words_read;
                                k -= words_read;                            	   
                                break;
                                
                            case IFTX_FMT_SC16Q11:
                            	memcpy(tx_buffer_current_b, raw_if_buffer_current, words_read*sizeof(u_int32_t));
                            	samples_populated = words_read;

                                /* Advance the buffer pointer.
                                 * Remember, two int16_t's make up 1 sample in the SC16Q11 
                                 * format */
                                buffer_samples_remaining_b -= words_read;
                                tx_buffer_current_b += (2*words_read);    /* 'words_read' BladeRF samples */
                                raw_if_buffer_current += words_read;
                                k -= words_read;
 								break;	
                            	
                            case IFTX_FMT_LS2:  
                            default:            break;
                  		
                   		}
                   }

                    /* If the end of the file was reached, determine whether
                     * to delay, re-read from the file, or pad the rest of the
                     * buffer and finish */
                    if (feof(tx->file_mgmt.file)) {
                        repeats_remaining--;

                        if ((repeats_remaining > 0) || repeat_infinite) {
                            if (delay_samples != 0) {
                                delay_samples_remaining = delay_samples;
                                state                   = DELAY;
                            }
                        } else {
                            state = PAD_TRAILING;
                        }

                        /* Clear the EOF condition and rewind the file */
                        clearerr(tx->file_mgmt.file);
                        rewind(tx->file_mgmt.file);
                    }

                    /* Check for errors */
                    else if (ferror(tx->file_mgmt.file)) {
                        status = errno;
                        set_last_error(&tx->last_error, ETYPE_ERRNO, status);
                    }

                    MUTEX_UNLOCK(&tx->file_mgmt.file_lock);
                    break;

                case DELAY:
                    /* Insert as many zeros as are necessary to realize the
                     * specified repeat delay */
                    samples_populated = uint_min(buffer_samples_remaining,
                                                 delay_samples_remaining);

                    memset(tx_buffer_current, 0,
                           samples_populated * 2 * sizeof(uint16_t));

                    delay_samples_remaining -= (unsigned int)samples_populated;

                    if (delay_samples_remaining == 0) {
                        state = READ_FILE;
                    }
                    break;

                case PAD_TRAILING:
                    /* Populate the remainder of the buffer with zeros */
                    memset(tx_buffer_current, 0,
                           buffer_samples_remaining * 2 * sizeof(uint16_t));
                    state = DONE;
                    break;

                case DONE:
                default:
                    break;
            }
            
            /* Advance the buffer pointer.
             * Remember, two int16_t's make up 1 sample in the SC16Q11 format */
            buffer_samples_remaining -= (unsigned int)samples_populated;
            tx_buffer_current += (2 * samples_populated);
        }

        /* If there were no errors, transmit the data buffer */
        if (status == 0) {
            bladerf_sync_tx(s->dev, tx_buffer, samples_per_buffer, NULL,
                            timeout_ms);
        }
    }

    /* Flush zero samples through the device to ensure samples reach the RFFE
     * before we exit and then disable the TX channel.
     *
     * This is a bit excessive, but sufficient for the time being. */
    if (status == 0) {
        const unsigned int num_buffers = tx->data_mgmt.num_buffers;
        unsigned int i;

        memset(tx_buffer, 0, samples_per_buffer * 2 * sizeof(int16_t));
        for (i = 0; i < (num_buffers + 1) && status == 0; i++) {
            status = bladerf_sync_tx(s->dev, tx_buffer, samples_per_buffer,
                                     NULL, timeout_ms);
        }
    }

    free(tx_buffer);
    return status;
}

/* Create a temp (binary) file from a CSV so we don't have to waste time
 * parsing it in between sending samples.
 *
 * Postconditions: TX cfg's file descriptor, filename, and format will be
 *                 changed. (On success they'll be set to the binary file,
 *                 and on failure the csv will be closed.)
 *
 * return 0 on success, CLI_RET_* on failure
 */
#if 0
static int tx2_csv_to_sc16q11(struct cli_state *s)
{
    struct rxtx_data *tx = s->tx;
    char buf[81]         = { 0 };
    FILE *bin            = NULL;
    FILE *csv            = NULL;
    char *bin_name       = NULL;
    int16_t *tmp_iq      = NULL;
    int line             = 1;
    size_t n_clamped     = 0;

    int status;

    assert(tx->file_mgmt.path != NULL);

    status = expand_and_open(tx->file_mgmt.path, "r", &csv);
    if (status != 0) {
        goto tx2_csv_to_sc16q11_out;
    }

    bin_name = strdup(TMP_FILE_NAME);
    if (!bin_name) {
        status = CLI_RET_MEM;
        goto tx2_csv_to_sc16q11_out;
    }

    status = expand_and_open(bin_name, "wb+", &bin);
    if (status != 0) {
        goto tx2_csv_to_sc16q11_out;
    }

    while (fgets(buf, sizeof(buf), csv)) {
        int i, cols, tmp_int;
        int **args = NULL;

        cols = csv2int(buf, &args);

        if (cols < 0) {
            cli_err(s, "tx", "Line (%d): Parsing failed.\n", line);
            status = CLI_RET_INVPARAM;
            break;
        }

        if (cols == 0) {
            // empty line...?
            continue;
        }

        tmp_iq = realloc(tmp_iq, cols * sizeof(int16_t));

        for (i = 0; i < cols; ++i) {
            tmp_int = *args[i];

            if (tmp_int < SC16Q11_IQ_MIN) {
                tmp_int = SC16Q11_IQ_MIN;
                n_clamped++;
            } else if (tmp_int > SC16Q11_IQ_MAX) {
                tmp_int = SC16Q11_IQ_MAX;
                n_clamped++;
            }

            tmp_iq[i] = tmp_int;
        }

        free_csv2int(cols, args);

        if (cols % 2 != 0) {
            cli_err(
                s, "tx",
                "Line (%d): Encountered %d value%s (values must be in pairs)\n",
                line, cols, 1 == cols ? "" : "s");
            status = CLI_RET_INVPARAM;
            break;
        }

        if ((int)fwrite(tmp_iq, sizeof(tmp_iq[0]), cols, bin) != cols) {
            status = CLI_RET_FILEOP;
            break;
        }

        line++;
    }

    if (status == 0) {
        if (feof(csv)) {
            tx->file_mgmt.format = RXTX_FMT_BIN_SC16Q11;
            free(tx->file_mgmt.path);
            tx->file_mgmt.path = bin_name;

            if (n_clamped != 0) {
                printf("  Warning: %zu value%s clamped within DAC SC16 Q11 "
                       "range of [%d, %d].\n",
                       n_clamped, 1 == n_clamped ? "" : "s", SC16Q11_IQ_MIN,
                       SC16Q11_IQ_MAX);
            }
        } else {
            status = CLI_RET_FILEOP;
        }
    }

tx2_csv_to_sc16q11_out:
    if (status != 0) {
        free(bin_name);
    }

    free(tmp_iq);

    if (csv) {
        fclose(csv);
    }

    if (bin) {
        fclose(bin);
    }

    return status;
}
#endif

void *tx2_task(void *cli_state_arg)
{
    int status = 0;
    int disable_status;
    unsigned char requests;
    enum rxtx_state task_state;
    struct cli_state *cli_state = (struct cli_state *)cli_state_arg;
    struct rxtx_data *tx        = cli_state->tx;

    /* We expect to be in the IDLE state when this is kicked off. We could
     * also get into the shutdown state if the program exits before we
     * finish up initialization */
    task_state = rxtx_get_state(tx);
    assert(task_state == RXTX_STATE_INIT);

    set_last_error(&tx->last_error, ETYPE_BLADERF, 0);
    requests = 0;

    while (task_state != RXTX_STATE_SHUTDOWN) {
        task_state = rxtx_get_state(tx);
        switch (task_state) {
            case RXTX_STATE_INIT:
                rxtx_set_state(tx, RXTX_STATE_IDLE);
                break;

            case RXTX_STATE_IDLE:
                rxtx_task_exec_idle(tx, &requests);
                break;

            case RXTX_STATE_START: {
                enum error_type err_type = ETYPE_BUG;

                /* Clear out the last error */
                set_last_error(&tx->last_error, ETYPE_ERRNO, 0);

                /* Bug catcher */
                MUTEX_LOCK(&tx->file_mgmt.file_meta_lock);
                assert(tx->file_mgmt.file != NULL);
                MUTEX_UNLOCK(&tx->file_mgmt.file_meta_lock);

                /* Initialize the TX synchronous data configuration */
                status = bladerf_sync_config(
                    cli_state->dev, tx->data_mgmt.layout,
                    BLADERF_FORMAT_SC16_Q11, tx->data_mgmt.num_buffers,
                    tx->data_mgmt.samples_per_buffer,
                    tx->data_mgmt.num_transfers, tx->data_mgmt.timeout_ms);

                if (status < 0) {
                    err_type = ETYPE_BLADERF;
                }

                if (status == 0) {
                    rxtx_set_state(tx, RXTX_STATE_RUNNING);
                } else {
                    set_last_error(&tx->last_error, err_type, status);
                    rxtx_set_state(tx, RXTX_STATE_IDLE);
                }
            } break;

            case RXTX_STATE_RUNNING:
                status = rxtx_apply_channels(cli_state, tx, true);

                if (status < 0) {
                    set_last_error(&tx->last_error, ETYPE_BLADERF, status);
                } else {
                    status = tx2_task_exec_running(tx, cli_state);

                    if (status < 0) {
                        set_last_error(&tx->last_error, ETYPE_BLADERF, status);
                    }

                    disable_status = rxtx_apply_channels(cli_state, tx, false);

                    if (status == 0 && disable_status < 0) {
                        set_last_error(&tx->last_error, ETYPE_BLADERF,
                                       disable_status);
                    }
                }

                rxtx_set_state(tx, RXTX_STATE_STOP);
                break;

            case RXTX_STATE_STOP:
                rxtx_task_exec_stop(cli_state, tx, &requests);
                break;

            case RXTX_STATE_SHUTDOWN:
                break;

            default:
                /* Bug */
                assert(0);
                rxtx_set_state(tx, RXTX_STATE_IDLE);
        }
    }

    return NULL;
}

static int tx2_cmd_start(struct cli_state *s)
{
    int status = 0;

    /* Check that we're able to start up in our current state */
    status = rxtx_cmd_start_check(s, s->tx, "tx2");
    if (status != 0) {
        return status;
    }

    /* Perform file conversion (if needed) and open input file */
    MUTEX_LOCK(&s->tx->file_mgmt.file_meta_lock);

    #if 0
    /* The 'tx2' command does not support CSV formatted files so there is 
     * no need to perform this conversion. */
    if (s->tx->file_mgmt.format == RXTX_FMT_CSV_SC16Q11) {
        status = tx2_csv_to_sc16q11(s);

        if (status == 0) {
            printf("  Converted CSV to SC16 Q11 file and "
                   "switched to converted file.\n\n");
        }
    }
    #endif

    if (status == 0) {
        MUTEX_LOCK(&s->tx->file_mgmt.file_lock);

        assert(s->tx->file_mgmt.format == RXTX_FMT_BIN_SC16Q11);
        status = expand_and_open(s->tx->file_mgmt.path, "rb",
                                 &s->tx->file_mgmt.file);
        MUTEX_UNLOCK(&s->tx->file_mgmt.file_lock);
    }

    MUTEX_UNLOCK(&s->tx->file_mgmt.file_meta_lock);

    if (status != 0) {
        return status;
    }

    /* Request thread to start running */
    rxtx_submit_request(s->tx, RXTX_TASK_REQ_START);
    status = rxtx_wait_for_state(s->tx, RXTX_STATE_RUNNING, 3000);

    /* This should never occur. If it does, there's likely a defect
     * present in the tx task */
    if (status != 0) {
        cli_err(s, "tx2", "TX2 did not start up in the alloted time\n");
        status = CLI_RET_UNKNOWN;
    }

    return status;
}

void tx2_print_file_if_fmt(struct rxtx_data *rxtx,
                            const char *prefix,
                            const char *suffix)
{
    enum iftx_fmt if_fmt;

    MUTEX_LOCK(&rxtx->file_mgmt.file_meta_lock);
    if_fmt = rxtx->file_mgmt.if_fmt;
    MUTEX_UNLOCK(&rxtx->file_mgmt.file_meta_lock);

    switch (if_fmt) {
        case IFTX_FMT_RSM:      /* Real binary, 2-bit sign/mag */
            printf("%sReal 2-bit sign/mag binary%s", prefix, suffix);
            break;
        case IFTX_FMT_CSM:      /* Complex binary, 2-bit sign/mag */
            printf("%sComplex 2-bit sign/mag binary%s", prefix, suffix);
            break;
        case IFTX_FMT_R8:       /* Real binary, 1-byte */
            printf("%sReal 8-bit binary%s", prefix, suffix);
            break;
        case IFTX_FMT_C8:       /* Complex binary, 1-byte */
            printf("%sComplex 8-bit binary%s", prefix, suffix);
            break;
        case IFTX_FMT_LS2:      /* Labsat 2 format */
            printf("%sLabsat2%s", prefix, suffix);
            break;
        case IFTX_FMT_SC16Q11:  /* SC16Q11 format */
            printf("%sSC16Q11%s", prefix, suffix);
            break;

        default:
            printf("%sNot configured%s", prefix, suffix);
    }
}

static void tx2_print_config(struct rxtx_data *tx)
{
    unsigned int repetitions, repeat_delay;
    struct tx_params *tx_params = tx->params;

    MUTEX_LOCK(&tx->param_lock);
    repetitions  = tx_params->repeat;
    repeat_delay = tx_params->repeat_delay;
    MUTEX_UNLOCK(&tx->param_lock);

    printf("\n");
    rxtx_print_state(tx, "  State: ", "\n");
    rxtx_print_channel(tx, "  Channels: ", "\n");
    rxtx_print_error(tx, "  Last error: ", "\n");
    rxtx_print_file(tx, "  File: ", "\n");
    rxtx_print_file_format(tx, "  File format: ", "\n");
    

    if (repetitions) {
        printf("  Repetitions: %u\n", repetitions);
    } else {
        printf("  Repetitions: infinite\n");
    }

    if (repeat_delay) {
        printf("  Repetition delay: %u us\n", repeat_delay);
    } else {
        printf("  Repetition delay: none\n");
    }

    rxtx_print_stream_info(tx, "  ", "\n");

    tx2_print_file_if_fmt(tx, "  IF file format: ", "\n");

    printf("\n");
}

enum iftx_fmt iftx_str2fmt(const char *str)
{
    enum iftx_fmt ret = IFTX_FMT_INVALID;

    if (!strcasecmp("rsm", str)) {
        ret = IFTX_FMT_RSM;
    } else if (!strcasecmp("csm", str)) {
        ret = IFTX_FMT_CSM;
    } else if (!strcasecmp("r8", str)) {
        ret = IFTX_FMT_R8;
    } else if (!strcasecmp("c8", str)) {
        ret = IFTX_FMT_C8;
    } else if (!strcasecmp("ls2", str)) {
        ret = IFTX_FMT_LS2;
    } else if (!strcasecmp("sc16q11", str)) {
        ret = IFTX_FMT_SC16Q11;
    } else if (!strcasecmp("bin", str)) {
        ret = IFTX_FMT_SC16Q11;
    }
    return ret;
}


void tx2_set_file_if_fmt(struct rxtx_data *rxtx, enum iftx_fmt if_fmt)
{
    MUTEX_LOCK(&rxtx->file_mgmt.file_meta_lock);
    rxtx->file_mgmt.if_fmt = if_fmt;
    rxtx->file_mgmt.format = RXTX_FMT_BIN_SC16Q11;
    MUTEX_UNLOCK(&rxtx->file_mgmt.file_meta_lock);
}

static int tx2_config(struct cli_state *s, int argc, char **argv)
{
    int i;
    char *val;
    int status;
    struct tx_params *tx_params = s->tx->params;

    assert(argc >= 2);

    if (argc == 2) {
        tx2_print_config(s->tx);
        
        return 0;
    }

    for (i = 2; i < argc; i++) {
        status = rxtx_handle_config_param(s, s->tx, argv[0], argv[i], &val);

        if (status < 0) {
            return status;
        } else if (status == 0) {
            if (!strcasecmp("repeat", argv[i])) {
                /* Configure the number of transmission repetitions to use */
                unsigned int tmp;
                bool ok;

                tmp = str2uint(val, 0, UINT_MAX, &ok);
                if (ok) {
                    MUTEX_LOCK(&s->tx->param_lock);
                    tx_params->repeat = tmp;
                    MUTEX_UNLOCK(&s->tx->param_lock);
                } else {
                    cli_err(s, argv[0], RXTX_ERRMSG_VALUE(argv[1], val));
                    return CLI_RET_INVPARAM;
                }
            } else if (!strcasecmp("delay", argv[i])) {
                /* Configure the number of useconds between each repetition  */
                unsigned int tmp;
                bool ok;

                tmp = str2uint(val, 0, UINT_MAX, &ok);

                if (ok) {
                    MUTEX_LOCK(&s->tx->param_lock);
                    tx_params->repeat_delay = tmp;
                    MUTEX_UNLOCK(&s->tx->param_lock);
                } else {
                    cli_err(s, argv[0], RXTX_ERRMSG_VALUE(argv[i], val));
                    return CLI_RET_INVPARAM;
                }
            } else if (!strcasecmp("channel", argv[i])) {
                /* Configure TX channels */
                status = rxtx_handle_channel_list(s, s->tx, val);
                if (status < 0) {
                    if (CLI_RET_INVPARAM == status) {
                        cli_err(s, argv[0], RXTX_ERRMSG_VALUE(argv[i], val));
                    }
                    return status;
                }
            } else if (!strcasecmp("if_format", argv[i])) {

                enum iftx_fmt if_fmt;
                if_fmt = iftx_str2fmt(val);

                if (if_fmt == IFTX_FMT_INVALID) {
                    cli_err(s, argv[0], RXTX_ERRMSG_VALUE(argv[i], val));
                    status = CLI_RET_INVPARAM;
                    return status;
                } else {
                    tx2_set_file_if_fmt(s->tx, if_fmt);
                    status = 1;
                }
            } else {
                cli_err(s, argv[0], "Unrecognized config parameter: %s\n",
                        argv[i]);
                return CLI_RET_INVPARAM;
            }
        }
    }

    return 0;
}

int cmd_tx2(struct cli_state *s, int argc, char **argv)
{
    int status;

    assert(argc > 0);

    if (argc == 1) {
        tx2_print_config(s->tx);
        status = 0;
    } else if (!strcasecmp(argv[1], RXTX_CMD_START)) {
        status = tx2_cmd_start(s);
    } else if (!strcasecmp(argv[1], RXTX_CMD_STOP)) {
        status = rxtx_cmd_stop(s, s->tx);
    } else if (!strcasecmp(argv[1], RXTX_CMD_CONFIG)) {
        status = tx2_config(s, argc, argv);
    } else if (!strcasecmp(argv[1], RXTX_CMD_WAIT)) {
        status = rxtx_handle_wait(s, s->tx, argc, argv);
    } else {
        cli_err(s, argv[0], "Invalid command: \"%s\"\n", argv[1]);
        status = CLI_RET_INVPARAM;
    }

    return status;
}
