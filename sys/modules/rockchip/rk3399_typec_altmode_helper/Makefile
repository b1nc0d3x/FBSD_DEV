.PATH: ${SRCTOP}/sys/arm64/rockchip

KMOD=	rk3399_typec_altmode_helper
SRCS=	rk3399_typec_altmode_helper.c
SRCS+=	rk3399_typec_altmode_var.h

SRCS+=	\
	bus_if.h \
	device_if.h \
	opt_platform.h

.include <bsd.kmod.mk>
