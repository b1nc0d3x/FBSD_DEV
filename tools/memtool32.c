#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

static void
usage(const char *prog)
{
	fprintf(stderr,
	    "usage: %s read <physaddr>\n"
	    "       %s write <physaddr> <value>\n",
	    prog, prog);
}

int
main(int argc, char **argv)
{
	long pagesz;
	uint64_t addr, page_base, page_off;
	uint32_t value;
	volatile uint32_t *reg;
	void *map;
	char *end;
	int fd, prot, flags, rv;
	bool do_write;

	if (argc != 3 && argc != 4) {
		usage(argv[0]);
		return (2);
	}

	if (strcmp(argv[1], "read") == 0 && argc == 3) {
		do_write = false;
	} else if (strcmp(argv[1], "write") == 0 && argc == 4) {
		do_write = true;
	} else {
		usage(argv[0]);
		return (2);
	}

	errno = 0;
	addr = strtoull(argv[2], &end, 0);
	if (errno != 0 || end == argv[2] || *end != '\0') {
		fprintf(stderr, "invalid address: %s\n", argv[2]);
		return (2);
	}

	value = 0;
	if (do_write) {
		errno = 0;
		value = (uint32_t)strtoul(argv[3], &end, 0);
		if (errno != 0 || end == argv[3] || *end != '\0') {
			fprintf(stderr, "invalid value: %s\n", argv[3]);
			return (2);
		}
	}

	pagesz = sysconf(_SC_PAGESIZE);
	if (pagesz <= 0) {
		perror("sysconf");
		return (1);
	}

	page_base = addr & ~((uint64_t)pagesz - 1);
	page_off = addr - page_base;
	prot = do_write ? (PROT_READ | PROT_WRITE) : PROT_READ;
	flags = do_write ? O_RDWR : O_RDONLY;

	fd = open("/dev/mem", flags | O_SYNC);
	if (fd < 0) {
		perror("open /dev/mem");
		return (1);
	}

	map = mmap(NULL, (size_t)pagesz, prot, MAP_SHARED, fd, (off_t)page_base);
	if (map == MAP_FAILED) {
		perror("mmap");
		close(fd);
		return (1);
	}

	reg = (volatile uint32_t *)((volatile uint8_t *)map + page_off);
	if (do_write) {
		uint32_t before = *reg;
		*reg = value;
		msync((void *)map, (size_t)pagesz, MS_SYNC);
		printf("0x%08" PRIx64 " 0x%08" PRIx32 " -> 0x%08" PRIx32 "\n",
		    addr, before, *reg);
	} else {
		printf("0x%08" PRIx64 " 0x%08" PRIx32 "\n", addr, *reg);
	}

	rv = munmap(map, (size_t)pagesz);
	if (rv != 0)
		perror("munmap");
	close(fd);
	return (rv == 0 ? 0 : 1);
}
