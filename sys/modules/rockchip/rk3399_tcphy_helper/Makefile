.PATH: ${SRCTOP}/sys/arm64/rockchip

KMOD=	rk3399_tcphy_helper
CFLAGS+=	-DFDT
SRCS=	rk3399_tcphy_helper.c

SRCS+=	\
	bus_if.h \
	device_if.h \
	ofw_bus_if.h \
	opt_platform.h \
	syscon_if.h

.include <bsd.kmod.mk>
