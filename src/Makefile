R_DESTDIR ?=
PREFIX ?= /usr/local

CC ?= gcc
CFLAGS ?= -O3
LDFLAGS ?=


# =====
_USTR = ustreamer.bin
_DUMP = ustreamer-dump.bin
_V4P = ustreamer-v4p.bin

_CFLAGS = -MD -c -std=c17 -Wall -Wextra -D_GNU_SOURCE $(CFLAGS)

_USTR_LDFLAGS = $(LDFLAGS) -Wl,-Bstatic -ljpeg -Wl,-Bdynamic -lm -pthread  -latomic -levent -levent_pthreads  -lrt -lm
_DUMP_LDFLAGS = $(LDFLAGS) -Wl,-Bstatic -ljpeg -Wl,-Bdynamic -lm -pthread -lrt -latomic -lm
_V4P_LDFLAGS = $(LDFLAGS)  -ljpeg -pthread -lrt -latomic

_USTR_SRCS = $(shell ls \
	libs/*.c \
	ustreamer/*.c \
	ustreamer/http/*.c \
	ustreamer/data/*.c \
	ustreamer/encoders/cpu/*.c \
	ustreamer/encoders/hw/*.c \
	ustreamer/*.c \
)

_DUMP_SRCS = $(shell ls \
	libs/*.c \
	dump/*.c \
)

_V4P_SRCS = $(shell ls \
	libs/*.c \
	libs/drm/*.c \
	v4p/*.c \
)

_BUILD = build


_TARGETS = $(_USTR) $(_DUMP)
_OBJS = $(_USTR_SRCS:%.c=$(_BUILD)/%.o) $(_DUMP_SRCS:%.c=$(_BUILD)/%.o)


define optbool
$(filter $(shell echo $(1) | tr A-Z a-z), yes on 1)
endef


ifneq ($(call optbool,$(WITH_GPIO)),)
override _CFLAGS += -DWITH_GPIO $(shell pkg-config --atleast-version=2 libgpiod 2> /dev/null && echo -DHAVE_GPIOD2)
override _USTR_LDFLAGS += -Wl,-Bstatic -lgpiod -Wl,-Bdynamic
override _USTR_SRCS += $(shell ls ustreamer/gpio/*.c)
endif


ifneq ($(call optbool,$(WITH_SYSTEMD)),)
override _CFLAGS += -DWITH_SYSTEMD
override _USTR_LDFLAGS += -lsystemd
override _USTR_SRCS += $(shell ls ustreamer/http/systemd/*.c)
endif

ifneq ($(call optbool,$(WITH_LIBX264)),)
override _CFLAGS += -DWITH_LIBX264
override _USTR_LDFLAGS += -Wl,-Bstatic -lyuv -Wl,-Bdynamic -lx264
override _USTR_SRCS += $(shell ls ustreamer/encoders/libx264/*.c)
endif

ifneq ($(call optbool,$(WITH_MEDIACODEC)),)
override _CFLAGS += -DWITH_MEDIACODEC
override _USTR_LDFLAGS += -Wl,-Bstatic -lyuv -Wl,-Bdynamic -lavcodec -lavformat -lavutil -landroid-shmem
override _USTR_SRCS += $(shell ls ustreamer/encoders/android_mediacodec/*.c)
endif

WITH_PTHREAD_NP ?= 1
ifneq ($(call optbool,$(WITH_PTHREAD_NP)),)
override _CFLAGS += -DWITH_PTHREAD_NP
endif


WITH_SETPROCTITLE ?= 1
ifneq ($(call optbool,$(WITH_SETPROCTITLE)),)
override _CFLAGS += -DWITH_SETPROCTITLE
ifeq ($(shell uname -s | tr A-Z a-z),linux)
override _USTR_LDFLAGS += -lbsd
endif
endif


WITH_V4P ?= 0
ifneq ($(call optbool,$(WITH_V4P)),)
override _TARGETS += $(_V4P)
override _OBJS += $(_V4P_SRCS:%.c=$(_BUILD)/%.o)
override _CFLAGS += -DWITH_V4P $(shell pkg-config --cflags libdrm)
override _V4P_LDFLAGS += $(shell pkg-config --libs libdrm)
override _USTR_SRCS += $(shell ls libs/drm/*.c)
override _USTR_LDFLAGS += $(shell pkg-config --libs libdrm)
endif


# =====
all: $(_TARGETS)


install: all
	mkdir -p $(R_DESTDIR)$(PREFIX)/bin
	for i in $(subst .bin,,$(_TARGETS)); do \
		install -m755 $$i.bin $(R_DESTDIR)$(PREFIX)/bin/$$i; \
	done


install-strip: install
	for i in $(subst .bin,,$(_TARGETS)); do \
		strip $(R_DESTDIR)$(PREFIX)/bin/$$i; \
	done


$(_USTR): $(_USTR_SRCS:%.c=$(_BUILD)/%.o)
	$(info == LD $@)
	$(ECHO) $(CC) $^ -o $@ $(_USTR_LDFLAGS)


$(_DUMP): $(_DUMP_SRCS:%.c=$(_BUILD)/%.o)
	$(info == LD $@)
	$(ECHO) $(CC) $^ -o $@ $(_DUMP_LDFLAGS)


$(_V4P): $(_V4P_SRCS:%.c=$(_BUILD)/%.o)
	$(info == LD $@)
	$(ECHO) $(CC) $^ -o $@ $(_V4P_LDFLAGS)


$(_BUILD)/%.o: %.c
	$(info -- CC $<)
	$(ECHO) mkdir -p $(dir $@) || true
	$(ECHO) $(CC) $< -o $@ $(_CFLAGS)


clean:
	rm -rf $(_USTR) $(_DUMP) $(_V4P) $(_BUILD)


-include $(_OBJS:%.o=%.d)
