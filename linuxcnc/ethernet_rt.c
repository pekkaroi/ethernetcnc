/*
 * Copyright 2015 Pekka Roivainen
 * Parts of this file are derived from LinuxCNC project with following copyrights:
 * Copyright 2013,2014 Michael Geszkiewicz <micges@wp.pl>, Jeff Epler <jepler@unpythonic.net>
 * Copyright (C) 2007-2009 Sebastian Kuzminsky
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef ULAPI
#include <rtapi_slab.h>
#include <rtapi_ctype.h>
#include <rtapi_list.h>

#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_string.h"
#include "rtapi_math.h"
#endif

#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/sockios.h>
#include <net/if_arp.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <errno.h>

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <signal.h>

#include "hal.h"


char *filename="ethernet.conf";

#ifndef ULAPI
#define SEND_TIMEOUT_US 1000
#define RECV_TIMEOUT_US 1000
MODULE_LICENSE("GPL");
RTAPI_MP_STRING(filename,"Configuration file");
#define LL_PRINT(fmt, args...)    rtapi_print("ethernet_rt: " fmt, ## args);
#else
#define SEND_TIMEOUT_US 5000
#define RECV_TIMEOUT_US 100000
#define LL_PRINT(fmt, args...)    printf("ethernet_uspace: " fmt, ## args);
#endif


#define READ_PCK_DELAY_NS 10000


#define f_period_s ((double)(l_period_ns * 1e-9))

void sendConfiguration();




static int comp_id;

#ifndef ULAPI
char *modname = "ethernet_rt";
#else 
char *modname = "ethernet_uspace";
#endif

static int sockfd = -1;
static struct sockaddr_in local_addr;
static struct sockaddr_in server_addr;

//these are the supported io types
int number_of_inputs = 0;
int number_of_outputs = 0;
int number_of_pwms = 0;
int number_of_encs = 0;
int number_of_stepgens = 0;



int bank_directions[] = {-1,-1,-1,-1,-1,-1,-1,-1};
char bank_dir_out;
char board_ip[50];


typedef struct {
	char name[30];
	int pin;
	hal_bit_t *inverted;
	hal_bit_t *state;
} input_t;

typedef struct {
	char name[30];
	int pin;
	hal_bit_t *inverted;
	hal_bit_t *state;
} output_t;

typedef struct {
	char name[30];
	int pin;
	int dirPin;
	uint16_t frequency;
	hal_float_t *dutycycle;
} pwm_t;

typedef struct {
	char name[30];
	int pin1;
	int pin2;
	hal_s32_t *count;
	hal_float_t scale;
	hal_float_t *pos;
} enc_t;

typedef struct {
	char name[30];
	int steppin;
	int dirpin;
	int step_len;
	int delay_dir;
	int delay_step;
	
	hal_bit_t *enable;
	hal_float_t *pos_cmd;
	hal_float_t old_pos_cmd;
	hal_float_t *pos_fb;
	hal_float_t pos_scale;
	hal_float_t *velocity_fb;
	hal_float_t maxvel;
	hal_float_t maxaccel;
	
	hal_float_t *dbg_ff_vel;
	hal_float_t *dbg_vel_error;
	hal_float_t *dbg_s_to_match;
	hal_float_t *dbg_err_at_match;
	hal_float_t *dbg_pos_minus_prev_cmd;
	hal_u32_t *dbg_accum;
	hal_bit_t *dbg_dir;
	hal_bit_t *pkg_failure;
	
	hal_u32_t period;
	hal_u32_t *dbg_period;
	hal_bit_t dir;
#ifndef ULAPI
	rtapi_u32 prev_accu;
	rtapi_s64 subcounts;
#else
	uint32_t prev_accu;
	int64_t subcounts;
#endif
} stepgen_t;


typedef struct {
	hal_bit_t	*enable;
	 hal_s32_t     *read_looptime; //for rt looptime
	 hal_s32_t     *write_looptime; //for rt looptime
	hal_bit_t	*comm_ok;
	output_t	*outputs;				
	input_t		*inputs;
	pwm_t		*pwms;
	enc_t		*encs;				
	stepgen_t   *stepgens;
	
    hal_float_t     looptime; //for userspace looptime
   


} ethernetcnc_t;

ethernetcnc_t *ethcnc;

/*
 * The function  stepgen_instance_position_control is largely based on function hm2_stepgen_instance_position_control from LinuxCNC's hostmot2 stepgen driver written by Sebastian Kuzminsky
 * 
 */
static void stepgen_instance_position_control(ethernetcnc_t *h, long l_period_ns, int i) {
   
    
    double ff_vel;
    double velocity_error;
    double match_accel;
    double seconds_to_vel_match;
    double position_at_match;
    double position_cmd_at_match;
    double error_at_match;
    double velocity_cmd;
	uint32_t new_period;
    stepgen_t *s = &h->stepgens[i];
	
	if(!(*s->enable))
	{
		s->period = 0;
		s->old_pos_cmd = (*s->pos_cmd);
		*s->velocity_fb = 0;
		return;
	}
    (*s->dbg_pos_minus_prev_cmd) = (*s->pos_fb) - s->old_pos_cmd;
	

    // calculate feed-forward velocity in machine units per second
    ff_vel = ((*s->pos_cmd) - s->old_pos_cmd) / f_period_s;
    (*s->dbg_ff_vel) = ff_vel;

    s->old_pos_cmd = (*s->pos_cmd);

    velocity_error = (*s->velocity_fb) - ff_vel;
    (*s->dbg_vel_error) = velocity_error;

    // Do we need to change speed to match the speed of position-cmd?
    // If maxaccel is 0, there's no accel limit: fix this velocity error
    // by the next servo period!  This leaves acceleration control up to
    // the trajectory planner.
    // If maxaccel is not zero, the user has specified a maxaccel and we
    // adhere to that.
    if (velocity_error > 0.0) {
        if (s->maxaccel == 0) {
            match_accel = -velocity_error / f_period_s;
        } else {
            match_accel = -s->maxaccel;
        }
    } else if (velocity_error < 0.0) {
        if (s->maxaccel == 0) {
            match_accel = velocity_error / f_period_s;
        } else {
            match_accel = s->maxaccel;
        }
    } else {
        match_accel = 0;
    }

    if (match_accel == 0) {
        // vel is just right, dont need to accelerate
        seconds_to_vel_match = 0.0;
    } else {
        seconds_to_vel_match = -velocity_error / match_accel;
    }
   *s->dbg_s_to_match = seconds_to_vel_match;

    // compute expected position at the time of velocity match
    // Note: this is "feedback position at the beginning of the servo period after we attain velocity match"
    {
        double avg_v;
        avg_v = (ff_vel + *s->velocity_fb) * 0.5;
        position_at_match = *s->pos_fb + (avg_v * (seconds_to_vel_match + f_period_s));
    }

    // Note: this assumes that position-cmd keeps the current velocity
    position_cmd_at_match = *s->pos_cmd + (ff_vel * seconds_to_vel_match);
    error_at_match = position_at_match - position_cmd_at_match;

    *s->dbg_err_at_match = error_at_match;

    if (seconds_to_vel_match <= f_period_s) {
        // we can match velocity in one period
        // try to correct whatever position error we have
        
        //if we are within one step, just match velocity
        if(fabs(error_at_match) < 1/s->pos_scale && fabs(ff_vel)<0.0001)
        {
			velocity_cmd = ff_vel;
		}
		else
		{
			velocity_cmd = ff_vel - (0.5 * error_at_match / f_period_s);
		}
		// apply accel limits?
		if (s->maxaccel > 0) {
			if (velocity_cmd > (*s->velocity_fb + (s->maxaccel * f_period_s))) {
				velocity_cmd = *s->velocity_fb + (s->maxaccel * f_period_s);
			} else if (velocity_cmd < (*s->velocity_fb - (s->maxaccel * f_period_s))) {
				velocity_cmd = *s->velocity_fb - (s->maxaccel * f_period_s);
			}
		}
		
	

    } else {
		
        // we're going to have to work for more than one period to match velocity
        // FIXME: I dont really get this part yet

        double dv;
        double dp;

        // calculate change in final position if we ramp in the opposite direction for one period 
        dv = -2.0 * match_accel * f_period_s;
        dp = dv * seconds_to_vel_match;

        // decide which way to ramp 
        if (fabs(error_at_match + (dp * 2.0)) < fabs(error_at_match)) {
            match_accel = -match_accel;
        }

        // and do it 
        velocity_cmd = *s->velocity_fb + (match_accel * f_period_s);
    }

    
    *s->velocity_fb = velocity_cmd;

	if(velocity_cmd < 0) s->dir = 0;
	else s->dir = 1;
	*s->dbg_dir = s->dir;

    if(fabs(velocity_cmd)<0.00001) new_period = 0;

    else 
    {
		new_period = 1/(fabs(velocity_cmd)*s->pos_scale)*1.68e6; //1.68MHz is the counter clock
		if(new_period<3) new_period = 3;
		if(new_period>UINT16_MAX-1) new_period=0;
    
	}
	*s->dbg_period = new_period;
    s->period = new_period;
}



static void ethernetcnc_read(void *void_haldata, long period) {
	long long t0, t1, t2;
#ifndef ULAPI
	t0 = rtapi_get_time();		
#endif
	ethernetcnc_t *haldata = void_haldata;
	int retval;

	static int pkgId=0;
	int pkgId_recv;
	
	int i,j=0;
	char c[sizeof(hal_float_t)];
	uint32_t tmp=0;
	int32_t tmp_s;
	uint8_t buf[1500];
	uint8_t *recvbuf = buf;
	struct timeval start;
	struct timeval stop;
	
//do a dummy read to make sure there is nothing in buffer

	retval = recv(sockfd, recvbuf, 1500, MSG_DONTWAIT);//,NULL,NULL);

	pkgId++;
	if(pkgId>255) pkgId=0;


	buf[0] = 1;
	buf[1] = pkgId;
	buf[2] = '\0';

	retval = sendto(sockfd, buf, 3, 0, (struct sockaddr *) &server_addr, sizeof(struct sockaddr_in));
	if(retval < 0)
	{
		
		
		return;
	}
#ifndef ULAPI
	t1 = rtapi_get_time();	
	do
	{
		rtapi_delay(READ_PCK_DELAY_NS);
		retval = recv(sockfd, recvbuf, 150, 0);//,NULL,NULL);
		t2 = rtapi_get_time();			
	} while ((retval < 0) && ((t2 - t1) < 1000*1000));

	 
#else
	retval = recv(sockfd, recvbuf, 150, 0);//,NULL,NULL);
#endif
	
	if(retval < 0)
	{
		*(haldata->comm_ok)=0;
		LL_PRINT("%s: failed to receive %d\n",modname,pkgId);
		return;
	}
	*(haldata->comm_ok)=1;
	j=0;
	if(recvbuf[0] == 0)
	{
		j++;
		pkgId_recv = recvbuf[j++];
		if(pkgId_recv == pkgId)
		{
			tmp = (recvbuf[j++]<<16);
			tmp +=(recvbuf[j++]<<8); 
			tmp += recvbuf[j++];
			for(i=0;i<number_of_inputs;i++)
			{
				if(!*(haldata->inputs[i].inverted))
					*(haldata->inputs[i].state) = (tmp>>i) & 1;		
				else
					*(haldata->inputs[i].state) = ((~(tmp>>i)) & 1);		
			}
		
			for(i=0;i<number_of_encs;i++)
			{
				tmp_s = (recvbuf[j++]<<24);
				tmp_s += (recvbuf[j++]<<16);
				tmp_s +=(recvbuf[j++]<<8); 
				tmp_s += recvbuf[j++];
			
				*(haldata->encs[i].count) = tmp_s;
				*(haldata->encs[i].pos) = ((haldata->encs[i].scale))*tmp_s;
			}
			uint32_t accum;
	#ifndef ULAPI
			rtapi_s64 acc_delta;
	#else
			int64_t acc_delta;
	#endif
			for(i=0;i<number_of_stepgens;i++)
			{
			
				accum = (recvbuf[j++]<<24);
				accum += (recvbuf[j++]<<16);
				accum +=(recvbuf[j++]<<8); 
				accum += recvbuf[j++];
			
	#ifndef ULAPI
				acc_delta = (rtapi_s64)accum - (rtapi_s64)(haldata->stepgens[i].prev_accu);
	#else
				acc_delta = (int64_t)accum - (int64_t)(haldata->stepgens[i].prev_accu);
	#endif

				if (acc_delta > INT32_MAX) {
					acc_delta -= UINT32_MAX;
				} else if (acc_delta < INT32_MIN) {
					acc_delta += UINT32_MAX;
				}
				*(haldata->stepgens[i].dbg_accum) = acc_delta;
				haldata->stepgens[i].subcounts += acc_delta;
								
				haldata->stepgens[i].prev_accu = accum;
				*(haldata->stepgens[i].pos_fb) = (double)haldata->stepgens[i].subcounts/65536.0/(float)haldata->stepgens[i].pos_scale;
			}
		}
		else
		{
			LL_PRINT("Error: Sent package id %d, received %d\n", pkgId, pkgId_recv);	//lets read for all the queued packets:
			retval = recv(sockfd, recvbuf, 150, 0);//,NULL,NULL);
			*(haldata->comm_ok)=0;
		}

	}
	else if(recvbuf[0] == 4)
	{
		LL_PRINT("%s: NOT CONFIGURED - resending configuration packet\n",modname);
		sendConfiguration();
	
	}
	else
	{
		LL_PRINT("%s: Wrong packet start byte received: %x\n",modname, recvbuf[0]);
		*(haldata->comm_ok)=0;

	}
	t2 = rtapi_get_time();			
	*(haldata->read_looptime) = t2-t0;


}

static void ethernetcnc_write(void *void_haldata, long period) {

	ethernetcnc_t *haldata = void_haldata;
	long long t0, t2;
#ifndef ULAPI
	t0 = rtapi_get_time();	
#endif
	int retval;
	
	int i,j=0;
	char c[sizeof(hal_float_t)];
	uint32_t tmp=0;
	int32_t tmp_s;
	uint8_t buf[150];
	uint8_t recvbuf[150];

	buf[j++] = 0; // We are always sending command packets 
	for(i=number_of_outputs-1;i>=0;i--)
	{
		if(!(*(haldata->outputs[i].inverted)))
			tmp = (tmp<<1)+ (*(haldata->outputs[i].state));
		else
			tmp = (tmp<<1)+ (~(*(haldata->outputs[i].state)))&1;
	} 
	buf[j++]= (tmp>>16)&0xFF;
	buf[j++]= (tmp>>8)&0xFF;
	buf[j++]= (tmp>>0)&0xFF;
	for(i=0;i<number_of_pwms;i++)
	{
		float f =*(haldata->pwms[i].dutycycle);
		
		memcpy(c,&f,sizeof(hal_float_t*));
		buf[j++] = c[3];
		buf[j++] = c[2];
		buf[j++] = c[1];
		buf[j++] = c[0];

	}
	for(i=0;i<number_of_stepgens;i++)
	{
		stepgen_instance_position_control(haldata,period, i);
		tmp = (haldata->stepgens[i].period);
		buf[j++]=(tmp>>8)&0xFF;
		buf[j++] = tmp&0xFF;
		buf[j++] = (uint8_t)haldata->stepgens[i].dir;
	}


	recv(sockfd, recvbuf, 50, MSG_DONTWAIT);

	retval = sendto(sockfd, buf, j, 0, (struct sockaddr *) &server_addr, sizeof(struct sockaddr_in));
	if(retval < 0)
	{
		LL_PRINT("%s: failed to send\n",modname);
		return;
	}
#ifndef ULAPI
	t2 = rtapi_get_time();			
#endif
	*(haldata->write_looptime) = t2-t0;
}


static int init_net()
{
	
	int ret;
	struct timeval timeout;
	
	sockfd = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
	if (sockfd < 0) {
		printf("%s: ERROR: can't open socket: %s\n", modname, strerror(errno));
		return -1;
	}
	
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(20202);
	server_addr.sin_addr.s_addr = inet_addr(board_ip);
	local_addr.sin_family = AF_INET;
	local_addr.sin_port = htons(20202);
	local_addr.sin_addr.s_addr = INADDR_ANY;
	ret = bind(sockfd, (struct sockaddr *) &local_addr, sizeof(struct sockaddr_in));
	if (ret < 0) {
		printf("%s: ERROR: can't connect: %s\n", modname, strerror(errno));
		return -1;
	}
	
	timeout.tv_sec = 0;
	timeout.tv_usec = RECV_TIMEOUT_US;
	ret = setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));
	if (ret < 0) {
		printf("%s: ERROR: can't set socket option: %s\n", modname, strerror(errno));
		return -errno;
	}
	timeout.tv_usec = SEND_TIMEOUT_US;
	setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(timeout));
	if (ret < 0) {
		printf("%s: ERROR: can't set socket option: %s\n", modname, strerror(errno));
		return -errno;
	}

	


	return ret;
}
static int close_net()
{
	int ret = close(sockfd);
	if (ret < 0)
        LL_PRINT("ERROR: can't close socket: %s\n", strerror(errno));

    return ret < 0 ? -errno : 0;
}

static int export_to_hal()
{
	int retval,i;
	hal_pin_bit_newf(HAL_IN, &ethcnc->enable, comp_id, "%s.enable", modname);	
	hal_pin_s32_newf(HAL_OUT, &ethcnc->read_looptime, comp_id, "%s.read_looptime", modname);	
	hal_pin_s32_newf(HAL_OUT, &ethcnc->write_looptime, comp_id, "%s.write_looptime", modname);	
	
	hal_pin_bit_newf(HAL_OUT, &ethcnc->comm_ok, comp_id, "%s.comm_ok", modname);	

	for(i=0;i<number_of_inputs;i++)
	{
		hal_pin_bit_newf(HAL_OUT, &(ethcnc->inputs[i].state), comp_id, "%s.%s", modname,ethcnc->inputs[i].name);	
		hal_pin_bit_newf(HAL_IN, &(ethcnc->inputs[i].inverted), comp_id, "%s.%s-invert", modname,ethcnc->inputs[i].name);	
	}

	for(i=0;i<number_of_outputs;i++)
	{
		hal_pin_bit_newf(HAL_IN, &ethcnc->outputs[i].state, comp_id, "%s.%s", modname,ethcnc->outputs[i].name);	
		hal_pin_bit_newf(HAL_IN, &(ethcnc->outputs[i].inverted), comp_id, "%s.%s-invert", modname,ethcnc->outputs[i].name);	
	}

	for(i=0;i<number_of_pwms;i++)
	{
		hal_pin_float_newf(HAL_IN, &ethcnc->pwms[i].dutycycle, comp_id, "%s.%s", modname,ethcnc->pwms[i].name);	
	}

	for(i=0;i<number_of_encs;i++)
	{
		hal_pin_s32_newf(HAL_OUT, &ethcnc->encs[i].count, comp_id, "%s.%s-count", modname,ethcnc->encs[i].name);
		hal_pin_float_newf(HAL_OUT, &ethcnc->encs[i].pos, comp_id, "%s.%s-pos", modname,ethcnc->encs[i].name);		
		hal_param_float_newf(HAL_RW, &ethcnc->encs[i].scale, comp_id, "%s.%s-scale", modname,ethcnc->encs[i].name);		
	}
	
	for(i=0;i<number_of_stepgens;i++)
	{
		
		hal_pin_bit_newf(HAL_IN,  &ethcnc->stepgens[i].enable, comp_id, "%s.%s-enable", modname,ethcnc->stepgens[i].name);	
		hal_pin_float_newf(HAL_IN,  &ethcnc->stepgens[i].pos_cmd, comp_id, "%s.%s-cmd", modname,ethcnc->stepgens[i].name);	
		hal_pin_float_newf(HAL_OUT, &ethcnc->stepgens[i].pos_fb, comp_id, "%s.%s-fb", modname,ethcnc->stepgens[i].name);	
		hal_pin_float_newf(HAL_OUT, &ethcnc->stepgens[i].velocity_fb, comp_id, "%s.%s-vel_fb", modname,ethcnc->stepgens[i].name);	
		hal_pin_float_newf(HAL_OUT, &ethcnc->stepgens[i].dbg_ff_vel, comp_id, "%s.%s-dbg_ff_vel", modname,ethcnc->stepgens[i].name);	
		hal_pin_float_newf(HAL_OUT, &ethcnc->stepgens[i].dbg_vel_error, comp_id, "%s.%s-dbg_vel_error", modname,ethcnc->stepgens[i].name);	
		hal_pin_float_newf(HAL_OUT, &ethcnc->stepgens[i].dbg_s_to_match, comp_id, "%s.%s-dbg_s_to_match", modname,ethcnc->stepgens[i].name);	
		hal_pin_float_newf(HAL_OUT, &ethcnc->stepgens[i].dbg_err_at_match, comp_id, "%s.%s-dbg_err_at_match", modname,ethcnc->stepgens[i].name);	
		hal_pin_float_newf(HAL_OUT, &ethcnc->stepgens[i].dbg_pos_minus_prev_cmd, comp_id, "%s.%s-dbg_pos_minus_prev_cmd", modname,ethcnc->stepgens[i].name);	
		hal_pin_bit_newf(HAL_OUT, &ethcnc->stepgens[i].dbg_dir, comp_id, "%s.%s-dbg_dir", modname,ethcnc->stepgens[i].name);	
		hal_pin_bit_newf(HAL_OUT, &ethcnc->stepgens[i].pkg_failure, comp_id, "%s.%s-dbg_pkg_fail", modname,ethcnc->stepgens[i].name);	
		hal_pin_u32_newf(HAL_OUT, &ethcnc->stepgens[i].dbg_period, comp_id, "%s.%s-dbg_period", modname,ethcnc->stepgens[i].name);	
		hal_pin_u32_newf(HAL_OUT, &ethcnc->stepgens[i].dbg_accum, comp_id, "%s.%s-dbg_accum", modname,ethcnc->stepgens[i].name);	
		hal_param_float_newf(HAL_RW, &ethcnc->stepgens[i].maxaccel, comp_id, "%s.%s-maxaccel",modname,ethcnc->stepgens[i].name);
		hal_param_float_newf(HAL_RW, &ethcnc->stepgens[i].maxvel, comp_id, "%s.%s-maxvel",modname,ethcnc->stepgens[i].name);
		hal_param_float_newf(HAL_RW, &ethcnc->stepgens[i].pos_scale, comp_id, "%s.%s-pos_scale",modname,ethcnc->stepgens[i].name);
		ethcnc->stepgens[i].old_pos_cmd = 0;
		ethcnc->stepgens[i].subcounts = 0;

		*ethcnc->stepgens[i].pos_cmd=0;
		*ethcnc->stepgens[i].pos_fb=0;
		*ethcnc->stepgens[i].velocity_fb=0;
		

	}
	
	
#ifndef ULAPI
	char name[50];
	rtapi_snprintf(name, sizeof(name), "%s.read", modname);
	retval = hal_export_funct(name, ethernetcnc_read, ethcnc, 1, 0, comp_id);
	if (retval < 0) {
             LL_PRINT("error adding param '%s', aborting\n", name);
             return -EINVAL;
	}
	rtapi_snprintf(name, sizeof(name), "%s.write", modname);
	retval = hal_export_funct(name, ethernetcnc_write, ethcnc, 1, 0, comp_id);
	if (retval < 0) {
             LL_PRINT("error adding param '%s', aborting\n", name);
             return -EINVAL;
	}
#endif
	out_close:
		return retval;
}
int startsWith(const char *pre, const char *str)
{
   return strncmp(pre, str, strlen(pre)) == 0;
}

static int parse_config()
{
	FILE *fp;
	char line[100];
	char *lineptr;
	LL_PRINT("Loading configuration file %s\n", filename);
	fp = fopen(filename, "r");
	if (fp == NULL) {
		LL_PRINT("Can't open input file!\n");
		return -1;
	}
	//first round, only get count of different types of IO
	while(fgets(line, 100, fp) != NULL)
	{
		lineptr = strtok (line," \t;");
		if(!strcmp("INPUT", lineptr))
			number_of_inputs++;
		else if(!strcmp("OUTPUT", lineptr))
			number_of_outputs++;
		else if(!strcmp("PWM", lineptr))
			number_of_pwms++;
		else if(!strcmp("ENCODER", lineptr))
			number_of_encs++;
		else if(!strcmp("STEPGEN", lineptr))
			number_of_stepgens++;
	}
	
	//allocate memory
	ethcnc = hal_malloc(sizeof(ethernetcnc_t));

	if (ethcnc == NULL) 
	{
		LL_PRINT("ERROR: out of memory:\n");
		return -ENOMEM;
	}
	memset(ethcnc, 0, sizeof(ethernetcnc_t));
	
	//allocate shared memory to all devices based on the number of them
	
	ethcnc->inputs = hal_malloc(sizeof(input_t)*number_of_inputs);
	if ((ethcnc->inputs == 0)) {
		LL_PRINT("%s: ERROR: unable to allocate shared memory\n", modname);
		goto out_close;
	}
	memset(ethcnc->inputs, 0, sizeof(input_t)*number_of_inputs);

	ethcnc->outputs = hal_malloc(sizeof(output_t)*number_of_outputs);
	if ((ethcnc->outputs == 0)) {
		LL_PRINT("%s: ERROR: unable to allocate shared memory\n", modname);
		goto out_close;
	} 
	memset(ethcnc->outputs, 0, sizeof(output_t)*number_of_outputs);

	ethcnc->pwms = hal_malloc(sizeof(pwm_t)*number_of_pwms);
	if ((ethcnc->pwms == 0)) {
		LL_PRINT("%s: ERROR: unable to allocate shared memory\n", modname);
		goto out_close;
	} 
	memset(ethcnc->pwms, 0, sizeof(pwm_t)*number_of_pwms);

	ethcnc->encs = hal_malloc(sizeof(enc_t)*number_of_encs);
	if ((ethcnc->encs == 0)) {
		LL_PRINT("%s: ERROR: unable to allocate shared memory\n", modname);
		goto out_close;
	} 
	memset(ethcnc->encs, 0, sizeof(enc_t)*number_of_encs);

	ethcnc->stepgens = hal_malloc(sizeof(stepgen_t)*number_of_stepgens);
	if ((ethcnc->stepgens == 0)) {
		LL_PRINT("%s: ERROR: unable to allocate shared memory\n", modname);
		goto out_close;
	} 
	memset(ethcnc->stepgens, 0, sizeof(stepgen_t)*number_of_stepgens);
	
	//temporarily reset counters
	number_of_inputs=0;
	number_of_outputs=0;
	number_of_encs=0;
	number_of_pwms=0;
	number_of_stepgens=0;
	
	rewind(fp); //back to beginning
	while(fgets(line, 100, fp) != NULL)
	{
		lineptr = strtok (line," \t;");
	
		if(lineptr[0] == '#')
			continue; //comment
		if(!strcmp("IP", lineptr))
	    {
			
			lineptr = strtok (NULL," \t;");   
			if(lineptr == NULL)
			{
				LL_PRINT("Too few parameters\n");
				return -1;
			}
			memcpy(board_ip,lineptr,sizeof(board_ip));
			LL_PRINT("IP address is: %s\n", board_ip);
	    }
	    else if(!strcmp("INPUT", lineptr))
	    {
			
			lineptr = strtok (NULL," \t;");   
			if(lineptr == NULL)
			{
				LL_PRINT("Too few parameters\n");
				return -1;
			}
			memcpy(ethcnc->inputs[number_of_inputs].name,lineptr,sizeof(ethcnc->inputs[number_of_inputs].name));
			lineptr = strtok (NULL," \t;");   
			if(lineptr == NULL)
			{
				LL_PRINT("Too few parameters\n");
				return -1;
			}
			ethcnc->inputs[number_of_inputs].pin = atoi(lineptr);
			
			if(bank_directions[ethcnc->inputs[number_of_inputs].pin/8] == 1)
				LL_PRINT("WARNING: Both inputs and outputs defined in the same bank. Last definition will be selected\n");
			bank_directions[ethcnc->inputs[number_of_inputs].pin/8] = 0;
			
			
			LL_PRINT("Input %s in pin %i\n", ethcnc->inputs[number_of_inputs].name, ethcnc->inputs[number_of_inputs].pin);
			number_of_inputs++;
			
		}
		else if(!strcmp("OUTPUT", lineptr))
	    {
			
			lineptr = strtok (NULL," \t;");   
			if(lineptr == NULL)
			{
				LL_PRINT("Too few parameters\n");
				return -1;
			}
			memcpy(ethcnc->outputs[number_of_outputs].name,lineptr,sizeof(ethcnc->outputs[number_of_outputs].name));
			lineptr = strtok (NULL," \t;");   
			if(lineptr == NULL)
			{
				LL_PRINT("Too few parameters\n");
				return -1;
			}
			ethcnc->outputs[number_of_outputs].pin = atoi(lineptr);
			
			
			if(bank_directions[ethcnc->outputs[number_of_outputs].pin/8] == 0)
				LL_PRINT("WARNING: Both inputs and outputs defined in the same bank. Last definition will be selected\n");
			bank_directions[ethcnc->outputs[number_of_outputs].pin/8] = 1;
			
			
			LL_PRINT("Output %s in pin %i\n", ethcnc->outputs[number_of_outputs].name, ethcnc->outputs[number_of_outputs].pin);
			number_of_outputs++;
			
		}
		else if(!strcmp("PWM", lineptr))
	    {
			
			lineptr = strtok (NULL," \t;");   
			if(lineptr == NULL)
			{
				LL_PRINT("Too few parameters\n");
				return -1;
			}
			memcpy(ethcnc->pwms[number_of_pwms].name,lineptr,sizeof(ethcnc->pwms[number_of_pwms].name));
			//NAME
			lineptr = strtok (NULL," \t;");   
			if(lineptr == NULL)
			{
				LL_PRINT("Too few parameters\n");
				return -1;
			}
			//PWM PIN
			ethcnc->pwms[number_of_pwms].pin = atoi(lineptr);
			
			if(ethcnc->pwms[number_of_pwms].pin%8>3)
				LL_PRINT("WARNING: Illegal PWM pin\n");
			
			if(bank_directions[ethcnc->pwms[number_of_pwms].pin/8] == 0)
				LL_PRINT("WARNING: Both inputs and outputs defined in the same bank. Last definition will be selected\n");
			bank_directions[ethcnc->pwms[number_of_pwms].pin/8] = 1;

			//DIR PIN
			lineptr = strtok (NULL," \t;");   
			if(lineptr == NULL)
			{
				LL_PRINT("Too few parameters\n");
				return -1;
			}
			ethcnc->pwms[number_of_pwms].dirPin = atoi(lineptr);
			
			
			lineptr = strtok (NULL," \t;");   
			if(lineptr == NULL)
			{
				LL_PRINT("Too few parameters\n");
				return -1;
			}
			ethcnc->pwms[number_of_pwms].frequency = atoi(lineptr);
			
			
			
			LL_PRINT("PWM %s in pin %i\n", ethcnc->pwms[number_of_pwms].name, ethcnc->pwms[number_of_pwms].pin);
			number_of_pwms++;
			
		}
		else if(!strcmp("ENCODER", lineptr))
	    {
			
			lineptr = strtok (NULL," \t;");   
			if(lineptr == NULL)
			{
				LL_PRINT("Too few parameters\n");
				return -1;
			}
			memcpy(ethcnc->encs[number_of_encs].name,lineptr,sizeof(ethcnc->encs[number_of_encs].name));
			lineptr = strtok (NULL," \t;");   
			if(lineptr == NULL)
			{
				LL_PRINT("Too few parameters\n");
				return -1;
			}
			ethcnc->encs[number_of_encs].pin1 = atoi(lineptr);
			lineptr = strtok (NULL," \t;");   
			if(lineptr == NULL)
			{
				LL_PRINT("Too few parameters\n");
				return -1;
			}
			ethcnc->encs[number_of_encs].pin2 = atoi(lineptr);
			int tmp;
			if(abs(ethcnc->encs[number_of_encs].pin2 - ethcnc->encs[number_of_encs].pin1)>1 || ethcnc->encs[number_of_encs].pin2%8>3 || ethcnc->encs[number_of_encs].pin1%8>3)
				LL_PRINT("WARNING: Illegal encoder pin\n");
			if(bank_directions[ethcnc->encs[number_of_encs].pin1/8] == 1)
				LL_PRINT("WARNING: Both inputs and outputs defined in the same bank. Last definition will be selected\n");
			bank_directions[ethcnc->encs[number_of_encs].pin1/8] = 0;
			
			LL_PRINT("ENCODER %s in pins %i and %i\n", ethcnc->encs[number_of_encs].name, ethcnc->encs[number_of_encs].pin1, ethcnc->encs[number_of_encs].pin2);
			number_of_encs++;
			
		}
		else if(!strcmp("STEPGEN", lineptr))
	    {
			
			lineptr = strtok (NULL," \t;");   
			if(lineptr == NULL)
			{
				LL_PRINT("Too few parameters\n");
				return -1;
			}
			memcpy(ethcnc->stepgens[number_of_stepgens].name,lineptr,sizeof(ethcnc->stepgens[number_of_stepgens].name));
			lineptr = strtok (NULL," \t;");   
			if(lineptr == NULL)
			{
				LL_PRINT("Too few parameters\n");
				return -1;
			}
			ethcnc->stepgens[number_of_stepgens].steppin= atoi(lineptr);
			lineptr = strtok (NULL," \t;");   
			if(lineptr == NULL)
			{
				LL_PRINT("Too few parameters\n");
				return -1;
			}
			ethcnc->stepgens[number_of_stepgens].dirpin= atoi(lineptr);
			
			
			if(bank_directions[ethcnc->stepgens[number_of_stepgens].steppin/8] == 0)
				LL_PRINT("WARNING: Both inputs and outputs defined in the same bank. Last definition will be selected\n");
			bank_directions[ethcnc->stepgens[number_of_stepgens].steppin/8] = 1;
			
			if(bank_directions[ethcnc->stepgens[number_of_stepgens].dirpin/8] == 0)
				LL_PRINT("WARNING: Both inputs and outputs defined in the same bank. Last definition will be selected\n");
			bank_directions[ethcnc->stepgens[number_of_stepgens].dirpin/8] = 1;
			
			
			
			
			lineptr = strtok (NULL," \t;");   
			if(lineptr == NULL)
			{
				ethcnc->stepgens[number_of_stepgens].step_len = 1;
			}
			else
			{
				ethcnc->stepgens[number_of_stepgens].step_len = atoi(lineptr);
			}
			
			lineptr = strtok (NULL," \t;");   
			if(lineptr == NULL)
			{
				ethcnc->stepgens[number_of_stepgens].delay_dir = 1;
			}
			else
			{
				ethcnc->stepgens[number_of_stepgens].delay_dir = atoi(lineptr);
			}
			
			lineptr = strtok (NULL," \t;");   
			if(lineptr == NULL)
			{
				ethcnc->stepgens[number_of_stepgens].delay_step = 1;
			}
			else
			{
				ethcnc->stepgens[number_of_stepgens].delay_step = atoi(lineptr);
			}
				
			LL_PRINT("STEPGEN %s in pins %i and %i\n", ethcnc->stepgens[number_of_stepgens].name, ethcnc->stepgens[number_of_stepgens].steppin, ethcnc->stepgens[number_of_stepgens].dirpin);
			number_of_stepgens++;
			
		}
	    
	}
	fclose(fp);
	bank_dir_out=0;
	int i;
	for(i = 0;i<8;i++)
	{
		if(bank_directions[i] > 0)
			bank_dir_out |= (1<<i);
	}
	//LL_PRINT("Bank directions: %x", bank_dir_out);
		
	return 0;
out_close:
	fclose(fp);
	return -1;
	
}

void sendConfiguration()
{
	char buf[100];
	int j=0;
	int i;
	buf[j++]=2; //configuration message
	buf[j++]=number_of_inputs;
	buf[j++]=number_of_outputs;
	buf[j++]=number_of_pwms;
	buf[j++]=number_of_encs;
	buf[j++]=number_of_stepgens;
	
	for(i=0;i<number_of_inputs;i++)
	{
		buf[j++] = ethcnc->inputs[i].pin;
	}
	for(i=0;i<number_of_outputs;i++)
	{
		buf[j++] = ethcnc->outputs[i].pin;
	}
	for(i=0;i<number_of_pwms;i++)
	{
		buf[j++] = ethcnc->pwms[i].pin;
		buf[j++] = ethcnc->pwms[i].dirPin;
		buf[j++] = (ethcnc->pwms[i].frequency>>8) & 0xFF;
		buf[j++] = (ethcnc->pwms[i].frequency) & 0xFF;
	}
	for(i=0;i<number_of_encs;i++)
	{
		buf[j++] = ethcnc->encs[i].pin1;
		buf[j++] = ethcnc->encs[i].pin2;
	}
	for(i=0;i<number_of_stepgens;i++)
	{
		buf[j++] = ethcnc->stepgens[i].steppin;
		buf[j++] = ethcnc->stepgens[i].dirpin;
		buf[j++] = ethcnc->stepgens[i].step_len;
		buf[j++] = ethcnc->stepgens[i].delay_dir;
		buf[j++] = ethcnc->stepgens[i].delay_step;
	}
	buf[j++]='\0';
	i = sendto(sockfd, buf, j, 0, (struct sockaddr *) &server_addr, sizeof(struct sockaddr_in));
	if(i < 0)
	{
		LL_PRINT("%s: failed to send\n",modname);
		return;
	}
	
}

#ifndef ULAPI
int rtapi_app_main(void) {
	int ret;
	ret  = hal_init("ethernet_rt");
    if (ret < 0)
        goto error0;
    comp_id = ret;
    ret = parse_config();
    if(ret<0)
		goto error0;

    ret = init_net();
    if (ret < 0)
        goto error1;
	
	sendConfiguration();
	
	export_to_hal();
	

	hal_ready(comp_id);
	return 0;
	
error1:
    close_net();
error0:
    hal_exit(comp_id);
    return ret;
}

void rtapi_app_exit(void) {
	close_net();
	hal_exit(comp_id);
	LL_PRINT("unloading EthernetCNC\n");

}
#else
int main(int argc, char **argv)
{
	int ret;
	int done=0;
	struct timespec loop_timespec, remaining;
	ret  = hal_init("ethernet_uspace");
	 if (ret < 0)
        goto error0;
    comp_id = ret;
    
	if (argc > 1)
	{
		filename = argv[1];
	}
	parse_config();
	init_net();
	sendConfiguration();
	export_to_hal();
	hal_ready(comp_id);
	ethcnc->looptime = 0.05; //TODO: Make this a parameter?
	while (done==0) {
		if(*ethcnc->enable)
		{
		ethernetcnc_read(ethcnc, ethcnc->looptime*1000000000l);
		ethernetcnc_write(ethcnc, ethcnc->looptime*1000000000l);
		}
		// don't want to scan too fast, and shouldn't delay more than a few seconds
		if (ethcnc->looptime < 0.001) ethcnc->looptime = 0.001;
		if (ethcnc->looptime > 1.0) ethcnc->looptime = 1.0;
		loop_timespec.tv_sec = (time_t)(ethcnc->looptime);
		loop_timespec.tv_nsec = (long)((ethcnc->looptime - loop_timespec.tv_sec) * 1000000000l);
		nanosleep(&loop_timespec, &remaining);

	}

	
error1:
    close_net();
error0:
    hal_exit(comp_id);
    return ret;
}
#endif
