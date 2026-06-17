
.PATH:	${SRCTOP}/sys/arm/broadcom/bcm2835

KMOD=	sdhci_iproc
SRCS=	sdhci_iproc.c \
	bus_if.h \
	device_if.h \
	mmcbr_if.h \
	ofw_bus_if.h \
	sdhci_if.h \
	opt_mmccam.h \
	opt_cam.h

.include <bsd.kmod.mk>
