#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <time.h>
#include "emu6502.h"

#define BENCHMARK 0

void run_vm(const char* rom_path);

int main(int argc, char** argv)
{
    run_vm("test/rom.bin");
    return 0;
}

unsigned long long time_ns()
{
	struct timespec tv;
	clock_gettime(CLOCK_MONOTONIC, &tv);
	return tv.tv_nsec + tv.tv_sec * 1000000000ULL;
}

void run_vm(const char* rom_path)
{
    struct stat     rom_stat;
    FILE*           rom_file;
    uint8_t*        rom;
    cpu_state       cpu = cpu_reset();
    uint8_t*        ram = (uint8_t*)malloc(32768);
    uint32_t        counter = 0;

    size_t test = sizeof(cpu_state);

    uint8_t         display[256] = {0xff};
    int             update_display = 0;
    struct winsize  w;
    uint64_t        time = 0;
    
    stat(rom_path, &rom_stat);

    if (rom_stat.st_size < 32768)
    {
        puts("rom file less than 32Kb");
        exit(-1);
    }
    else if (rom_stat.st_size > 32768)
    {
        puts("rom file more than 32Kb, will be truncated.");
    }

    rom = (uint8_t*)malloc(32768);

    rom_file = fopen(rom_path, "rb");
    if (!rom_file)
    {
        printf("Failed to open rom: %s", rom_path);
        exit(-1);
    }

    fread(rom, 32768, 1, rom_file);
    fclose(rom_file);

    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);

    if (display[0] == 0xff) 
        memset(display, 0, sizeof(display));

    while(1)
    {
        uint8_t* memory = ram;
        uint16_t address = cpu.address;

        /* Chip select */
        if (address & 0x8000)
        {
            memory = rom;
            address = address & 0x7fff;
        }
        else if (address >= 0x4000 && address <= (0x4000 + sizeof(display)))
        {
            memory = display;
            address -= 0x4000;
        }
        
        /* Memory read/write */
        if (CPU_IS_WRITE(cpu))
        {
            memory[address] = cpu.data;
            if (memory == display)
                update_display = 1;
        }
        else
        {
            cpu.data = memory[address];
        }

        /* Run CPU */
        cpu = cpu_execute(cpu);

        if (++counter)
        {
#if BENCHMARK == 1
            if (counter == 1000000)
            {
                uint64_t sample_time = time_ns() - time;
                uint64_t ns_per_cycle = sample_time / counter;
                uint64_t cycles_per_mcs = 1000000ULL / ns_per_cycle;
                float freqMHz = ((float)cycles_per_mcs) / 1000.0f;
                counter = 0;
                printf("freq: %.3f MHz time: %u mcs\n", freqMHz, ((uint32_t)(sample_time /1000ULL)));
                fflush(stdout);
                usleep(10);
                time = time_ns();
            }
#else 
            if ((counter %5) == 0)
            {
                /* Display */ 
                if (update_display)
                {
                    update_display = 0;
                    int i;
                    printf("\e[?25l"); // hide cursor
                    printf("\33[2K\r");// delete line
                    for (i = 0; i < sizeof(display); ++i)
                    {
                        if (i >= w.ws_col) continue;

                        char c = display[i];
                        printf("%c", c ? c : ' ');
                    }

                    fflush(stdout);
                }
                usleep(10);
            }
#endif
        }
    }

    free(rom);
    free(ram);
}
