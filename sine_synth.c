#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <stdio.h>
#include "tableprint.h"

static const unsigned char static_hdr_portion[20] = {
   0x52, 0x49, 0x46, 0x46, 0xFF, 0xFF, 0xFF, 0x7F,
   0x57, 0x41, 0x56, 0x45, 0x66, 0x6D, 0x74, 0x20,
   0x10, 0x00, 0x00, 0x00
};

typedef struct _WAVHeader {
    uint16_t wav_id;
    uint16_t channels;
    uint32_t samplerate;
    uint32_t bitrate;
    uint32_t block_align;
    uint32_t pad0;
    uint32_t pad1;
} __attribute__((packed)) WAVHeader;

static void write_wav_header(int fd, uint32_t rate, uint8_t channels)
{
   WAVHeader w;
   write(fd, &static_hdr_portion, 20);

   w.wav_id = 3;
   w.channels = channels;
   w.samplerate = rate;
   w.bitrate = rate*channels*2;
   w.block_align = (channels << 1) | 0x00200000;
   w.pad0 = 0x61746164;
   w.pad1 = 0x7fffffff;
   write(fd, &w, sizeof(WAVHeader));
}
WRITE_1D_FUNC(float, "%.6f", 3)
WRITE_1D_FUNC(uint16_t, "0x%04x", 7)

static __attribute__((always_inline)) inline uint16_t av_bswap16(uint16_t x)
{
	x= (x>>8) | (x<<8);
	return x;
}

#define WORD2INT(x) ((x) < -2046.5f ? -2047 : ((x) > 2046.5f ? 2047 : lrintf(x)))
int main(int argc, char **argv) {
    float *buf;
	uint16_t buf2[44100];
    unsigned int i = 0;
    buf = malloc(44100 * sizeof(float));
    while (i < 44100) {
        //
        // Generate the wave for each channel:
        // Amplitude * sin( Sample * Frequency * 2PI / SamplesPerSecond )
        //
        float tmp = ((i - 1) * 440.0f / 44100.0f);
        
        buf[i] = sinf(tmp * (float)M_PI); // sine
#if 0
        buf[i] = -1 + tmp; // saw
        buf[i] = -1 + (tmp < 0.5f); // square
        // triangle
        if(tmp < 0.5f) {
            buf[i] = -1 + tmp*2.0f;
        } else {
            buf[i] = 1 - (tmp - 0.5f)/(1.0f - 0.5f);
        }
#endif
        i++;
    }
	i = 0;
	while (i < 44100) {
		buf2[i] = WORD2INT((buf[i]*2048.0f));
		buf2[i] += 2048;
		buf2[i] |= 0x8000;
		buf2[i] = av_bswap16(buf2[i]);
		i++;
	}
	//WRITE_ARRAY2("const", float, buf, 44100);
	WRITE_ARRAY("const", uint16_t, buf2);
    return 0;
}

