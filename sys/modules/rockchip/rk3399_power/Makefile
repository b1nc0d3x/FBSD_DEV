.PATH: ${SRCTOP}/sys/arm64/rockchip

KMOD=	rk3399_power
SRCS=	rk3399_power.c
EXPORT_SYMS=	rk3399_power_enable_domain

SRCS+=	\
	bus_if.h \
	clknode_if.h \
	device_if.h \
	ofw_bus_if.h \
	opt_platform.h \
	syscon_if.h

.include <bsd.kmod.mk>
