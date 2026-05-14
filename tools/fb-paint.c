#include <sys/types.h>
#include <sys/fbio.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <err.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static uint32_t
parse_color(const char *s)
{
	char *end;
	unsigned long v;

	v = strtoul(s, &end, 0);
	if (*s == '\0' || *end != '\0')
		errx(1, "invalid color: %s", s);
	return ((uint32_t)v);
}

static void
fill_solid(uint32_t *fb, uint32_t stride_words, uint32_t width,
    uint32_t height, uint32_t color)
{
	uint32_t x, y;

	for (y = 0; y < height; y++) {
		uint32_t *row = fb + y * stride_words;
		for (x = 0; x < width; x++)
			row[x] = color;
	}
}

static void
fill_bars(uint32_t *fb, uint32_t stride_words, uint32_t width,
    uint32_t height)
{
	static const uint32_t colors[] = {
		0x00ffffffu, 0x00ff0000u, 0x0000ff00u, 0x000000ffu,
		0x00ffff00u, 0x0000ffffu, 0x00ff00ffu, 0x00000000u
	};
	uint32_t x, y, bar_w, idx;

	bar_w = width / (sizeof(colors) / sizeof(colors[0]));
	if (bar_w == 0)
		bar_w = 1;

	for (y = 0; y < height; y++) {
		uint32_t *row = fb + y * stride_words;
		for (x = 0; x < width; x++) {
			idx = x / bar_w;
			if (idx >= sizeof(colors) / sizeof(colors[0]))
				idx = (sizeof(colors) / sizeof(colors[0])) - 1;
			row[x] = colors[idx];
		}
	}
}

int
main(int argc, char **argv)
{
	struct fbtype fb;
	uint32_t *map;
	uint32_t stride_words;
	uint32_t color;
	int fd;

	if (argc > 3)
		errx(1, "usage: %s [bars|solid] [color]", argv[0]);

	fd = open("/dev/fb0", O_RDWR);
	if (fd < 0)
		err(1, "open /dev/fb0");

	if (ioctl(fd, FBIOGTYPE, &fb) != 0)
		err(1, "FBIOGTYPE");
	if (fb.fb_depth != 32)
		errx(1, "unsupported depth %d", fb.fb_depth);
	if (fb.fb_height <= 0 || fb.fb_width <= 0 || fb.fb_size <= 0)
		errx(1, "bad fb geometry %dx%dx%d size=%d",
		    fb.fb_width, fb.fb_height, fb.fb_depth, fb.fb_size);

	map = mmap(NULL, (size_t)fb.fb_size, PROT_READ | PROT_WRITE,
	    MAP_SHARED, fd, 0);
	if (map == MAP_FAILED)
		err(1, "mmap");

	stride_words = (uint32_t)(fb.fb_size / fb.fb_height / sizeof(uint32_t));

	if (argc >= 2 && strcmp(argv[1], "solid") == 0) {
		color = (argc >= 3) ? parse_color(argv[2]) : 0x00ffffffu;
		fill_solid(map, stride_words, (uint32_t)fb.fb_width,
		    (uint32_t)fb.fb_height, color);
	} else {
		fill_bars(map, stride_words, (uint32_t)fb.fb_width,
		    (uint32_t)fb.fb_height);
	}

	if (msync(map, (size_t)fb.fb_size, MS_SYNC) != 0)
		err(1, "msync");

	printf("painted /dev/fb0: %dx%dx%d size=%d stride=%u bytes mode=%s\n",
	    fb.fb_width, fb.fb_height, fb.fb_depth, fb.fb_size,
	    stride_words * (unsigned)sizeof(uint32_t),
	    (argc >= 2 && strcmp(argv[1], "solid") == 0) ? "solid" : "bars");

	munmap(map, (size_t)fb.fb_size);
	close(fd);
	return (0);
}
