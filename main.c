#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include "emu6502.h"

void run_vm(const char* rom_path);

int main(int argc, char** argv)
{
    run_vm("test/rom.bin");
    return 0;
}

void run_vm(const char* rom_path)
{
    struct stat     rom_stat;
    FILE*           rom_file;
    uint8_t*        rom;
    cpu_state       cpu = cpu_reset();
    uint8_t*        ram = (uint8_t*)malloc(32768);

    uint8_t         display[256] = {0xff};
    int             update_display = 0;
    int             i;
    struct winsize  w;
    
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
            update_display = 1;
            memory = display;
            address -= 0x4000;
        }
        
        /* Memory read/write */
        if (cpu.rw == 0)
        {
            cpu.data = memory[address];
        }
        else
        {
            memory[address] = cpu.data;
            if (memory == display)
                update_display = 1;
        }

        /* Run CPU */
        cpu = cpu_execute(cpu);

        /* Display */ 
        if (update_display)
        {
            update_display = 0;
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

    free(rom);
    free(ram);
}
