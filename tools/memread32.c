#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

int
main(int argc, char **argv)
{
	long pagesz;
	uint64_t addr, page_base, page_off;
	volatile uint32_t *reg;
	void *map;
	char *end;
	int fd;

	if (argc != 2) {
		fprintf(stderr, "usage: %s <physaddr>\n", argv[0]);
		return (2);
	}

	errno = 0;
	addr = strtoull(argv[1], &end, 0);
	if (errno != 0 || end == argv[1] || *end != '\0') {
		fprintf(stderr, "invalid address: %s\n", argv[1]);
		return (2);
	}

	pagesz = sysconf(_SC_PAGESIZE);
	if (pagesz <= 0) {
		perror("sysconf");
		return (1);
	}

	page_base = addr & ~((uint64_t)pagesz - 1);
	page_off = addr - page_base;

	fd = open("/dev/mem", O_RDONLY | O_SYNC);
	if (fd < 0) {
		perror("open /dev/mem");
		return (1);
	}

	map = mmap(NULL, (size_t)pagesz, PROT_READ, MAP_SHARED, fd,
	    (off_t)page_base);
	if (map == MAP_FAILED) {
		perror("mmap");
		close(fd);
		return (1);
	}

	reg = (volatile uint32_t *)((volatile uint8_t *)map + page_off);
	printf("0x%08" PRIx64 " 0x%08" PRIx32 "\n", addr, *reg);

	munmap(map, (size_t)pagesz);
	close(fd);
	return (0);
}
