.PATH: ${SRCTOP}/sys/arm64/rockchip

KMOD=	rk_drm
SRCS=	rk_drm.c rk_drm_hw.c

SRCS+=	\
	bus_if.h \
	device_if.h \
	fb_if.h \
	ofw_bus_if.h \
	pci_if.h \
	opt_drm.h \
	opt_platform.h \
	opt_syscons.h

.include <bsd.kmod.mk>
