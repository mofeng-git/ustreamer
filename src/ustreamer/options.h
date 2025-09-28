/*****************************************************************************
#                                                                            #
#    uStreamer - Lightweight and fast MJPEG-HTTP streamer.                   #
#                                                                            #
#    Copyright (C) 2018-2024  Maxim Devaev <mdevaev@gmail.com>               #
#                                                                            #
#    This program is free software: you can redistribute it and/or modify    #
#    it under the terms of the GNU General Public License as published by    #
#    the Free Software Foundation, either version 3 of the License, or       #
#    (at your option) any later version.                                     #
#                                                                            #
#    This program is distributed in the hope that it will be useful,         #
#    but WITHOUT ANY WARRANTY; without even the implied warranty of          #
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           #
#    GNU General Public License for more details.                            #
#                                                                            #
#    You should have received a copy of the GNU General Public License       #
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.  #
#                                                                            #
*****************************************************************************/


#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <limits.h>
#include <getopt.h>
#include <errno.h>
#include <assert.h>

#include "../libs/const.h"
#include "../libs/logging.h"
#include "../libs/process.h"
#include "../libs/frame.h"
#include "../libs/memsink.h"
#include "../libs/options.h"
#include "../libs/capture.h"
#if defined(WITH_DRM) || defined(WITH_V4P)
#	include "../libs/drm/drm.h"
#endif

#include "encoder.h"
#include "stream.h"
#include "http/server.h"
#ifdef WITH_GPIO
#	include "gpio/gpio.h"
#endif


typedef struct {
	unsigned		argc;
	char			**argv;
	char			**argv_copy;
	us_memsink_s	*jpeg_sink;
	us_memsink_s	*raw_sink;
	us_memsink_s	*h264_sink;
#	if defined(WITH_DRM) || defined(WITH_V4P)
	us_drm_s		*drm;
#	endif
} us_options_s;


us_options_s *us_options_init(unsigned argc, char *argv[]);
void us_options_destroy(us_options_s *options);

int options_parse(us_options_s *options, us_capture_s *cap, us_encoder_s *enc, us_stream_s *stream, us_server_s *server);
