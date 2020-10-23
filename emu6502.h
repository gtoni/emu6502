#ifndef _EMU6502_H_
#define _EMU6502_H_

#include "emu6502_opcodes.h"
#include <stdint.h>
#include <memory.h>

struct cpu_state_
{
    uint8_t  instruction;
    uint8_t  cycle;
    uint8_t  memory_addr_low;   //24 bits

    // registers
    uint8_t  A;
    uint8_t  X;
    uint8_t  Y;
    uint8_t  S;
    uint8_t  P; 
    uint16_t PC;                //56 bits 

    // pins
    uint8_t  ad1 : 1;
    uint8_t  ad2 : 1;

    uint8_t  out0 : 1;
    uint8_t  out1 : 1;
    uint8_t  out2 : 1;

    uint8_t  oe1 : 1;
    uint8_t  oe2 : 1;

    uint8_t  rw : 1;

    uint8_t  nmi : 1;
    uint8_t  irq : 1;           // 17 bits

    uint8_t  data;
    uint16_t address;           // 24 bits
};

typedef struct cpu_state_ cpu_state;


#define _CPU_END(cpu)               {cpu.cycle = 0;}
#define _CPU_FETCH_INSTRUCTION(cpu) {cpu.rw = 0; cpu.address = cpu.PC++;}
#define _CPU_FETCH_OPERAND(cpu)     {cpu.rw = 0; cpu.address = cpu.PC++;}
#define _CPU_COND_BRANCH(cpu, cond) cpu.PC += (cond)?(int8_t)cpu.data : 0

#define _CPU_UPDATE_NZ(cpu, v)       (cpu.P = (cpu.P & 0x7D)| ((v & 0x80) | (v?0:0x02)))
#define _CPU_SET_REG(cpu, reg, v)    {cpu.reg = v; _CPU_UPDATE_NZ(cpu, cpu.reg);} 
#define _CPU_SET_REG_A(cpu, v)        _CPU_SET_REG(cpu, A, v)
#define _CPU_SET_REG_X(cpu, v)        _CPU_SET_REG(cpu, X, v)
#define _CPU_SET_REG_Y(cpu, v)        _CPU_SET_REG(cpu, Y, v)
#define _CPU_SET_REG_S(cpu, v)        cpu.S = v
#define _CPU_SET_REG_P(cpu, v)        cpu.P = v

#define _CPU_BIT(cpu) cpu.P = (cpu.P & 0x3D) | (cpu.data & 0xC0) | (((cpu.A & cpu.data) == 0)?2:0)

#define _CPU_ADC(cpu) {\
    uint16_t tmp = ((uint16_t)cpu.A) + cpu.data + (cpu.P & 1);\
    _CPU_SET_REG_A(cpu, tmp & 0xFF);\
    _CPU_SET_REG_P(cpu, (cpu.P & 0xFE) | ((tmp >> 8) & 1));\
}

#define _CPU_SBC(cpu) {\
    uint16_t tmp = ((uint16_t)cpu.A) + ~cpu.data + (cpu.P & 1);\
    _CPU_SET_REG_A(cpu, tmp & 0xFF);\
    _CPU_SET_REG_P(cpu, (cpu.P & 0xFE) | ((~tmp >> 8) & 1));\
}

#define _CPU_ROL(cpu, v) {\
    uint8_t tmp = cpu.P & 1;\
    cpu.rw = 1;\
    _CPU_SET_REG_P(cpu, (cpu.P & 0xFE) | (v >> 7));\
    v = (v << 1) | tmp;\
    _CPU_UPDATE_NZ(cpu, v);\
}

#define _CPU_ROR(cpu, v) {\
    uint8_t tmp = cpu.P & 1;\
    cpu.rw = 1;\
    _CPU_SET_REG_P(cpu, (cpu.P & 0xFE) | (v & 1));\
    v = (v >> 1) | (tmp << 7);\
    _CPU_UPDATE_NZ(cpu, v);\
}

#define _CPU_ASL(cpu, v) {\
    cpu.rw = 1;\
    _CPU_SET_REG_P(cpu, (cpu.P & 0xFE) | (v >> 7 ));\
    v = (v << 1);\
    _CPU_UPDATE_NZ(cpu, v);\
}

#define _CPU_LSR(cpu, v) {\
    cpu.rw = 1;\
    _CPU_SET_REG_P(cpu, (cpu.P & 0xFE) | (v & 1));\
    v = (v >> 1);\
    cpu.P = (cpu.P & 0x7D) | (v?0:0x02);\
}


static cpu_state cpu_reset()
{
    cpu_state state;
    memset(&state, 0, sizeof(cpu_state));
    state.address = 0xfffc;
    state.S = 0xff;
    return state; 
}

static cpu_state cpu_execute(cpu_state state)
{
    uint8_t cycle = state.cycle++;

    if (state.instruction == 0)
    {
        if (cycle == 0)
        {
            if (state.address != 0xfffc)
                return state;
            state.address = 0xfffd;
            state.PC = state.data;
            return state;
        }
        else if (cycle == 1)
        {
            state.PC = (state.data << 8) | state.PC;
            _CPU_FETCH_INSTRUCTION(state);
            return state;
        }
        cycle = 0;
        state.cycle = 1;
    }

    if (cycle == 0)
    {
        state.instruction = state.data;

        switch (state.instruction)
        {
            case IC_PHP:
                state.rw = 1;
                state.address = ((uint8_t)state.S--) + 0x0100;
                state.data = state.P;
                return state;
            case IC_PHA:
                state.rw = 1;
                state.address = ((uint8_t)state.S--) + 0x0100;
                state.data = state.A;
                return state;
            case IC_RTS:
            case IC_RTI:
            case IC_PLP:
            case IC_PLA:
                state.address = ((uint8_t)++state.S) + 0x0100;
                return state;
            case IC_INX:
                _CPU_SET_REG_X(state, state.X + 1);
                return state;
            case IC_INY:
                _CPU_SET_REG_Y(state, state.Y + 1);
                return state;
            case IC_DEX:
                _CPU_SET_REG_X(state, state.X - 1);
                return state;
            case IC_DEY:
                _CPU_SET_REG_Y(state, state.Y - 1);
                return state;
            case IC_ROL_ACC:
                _CPU_ROL(state, state.A);
                return state;
            case IC_ROR_ACC:
                _CPU_ROR(state, state.A);
                return state;
            case IC_ASL_ACC:
                _CPU_ASL(state, state.A);
                return state;
            case IC_LSR_ACC:
                _CPU_LSR(state, state.A);
                return state;
            case IC_TAX:
                _CPU_SET_REG_X(state, state.A);
                return state;
            case IC_TAY:
                _CPU_SET_REG_Y(state, state.A);
                return state;
            case IC_TSX:
                _CPU_SET_REG_X(state, state.S);
                return state;
            case IC_TXA:
                _CPU_SET_REG_A(state, state.X);
                return state;
            case IC_TXS:
                _CPU_SET_REG_S(state, state.X);
                return state;
            case IC_TYA:
                _CPU_SET_REG_A(state, state.Y);
                return state;
            case IC_CLC:
                _CPU_SET_REG_P(state, state.P & 0xFE);
                return state;
            case IC_SEC:
                _CPU_SET_REG_P(state, state.P | 1);
                return state;
            case IC_CLI:
                _CPU_SET_REG_P(state, state.P & 0xFB);
                return state;
            case IC_SEI:
                _CPU_SET_REG_P(state, state.P | 0x04);
                return state;
            case IC_CLV:
                _CPU_SET_REG_P(state, state.P & 0xBF);
                return state;
            case IC_CLD:
                _CPU_SET_REG_P(state, state.P & 0xF7);
                return state;
            case IC_SED:
                _CPU_SET_REG_P(state, state.P | 0x08);
                return state;
            case IC_NOP: 
                return state;
            default: 
                _CPU_FETCH_OPERAND(state);
                return state;
        }
    }
    else if (cycle == 1)
    {
        switch (state.instruction)
        {
            case IC_ADC_IMM:  _CPU_ADC(state); break;
            case IC_AND_IMM:  _CPU_SET_REG_A(state, state.A & state.data); break;
            case IC_ORA_IMM:  _CPU_SET_REG_A(state, state.A | state.data); break;
            case IC_EOR_IMM:  _CPU_SET_REG_A(state, state.A ^ state.data); break;
            case IC_LDA_IMM:  _CPU_SET_REG_A(state, state.data); break;
            case IC_LDX_IMM:  _CPU_SET_REG_X(state, state.data); break;
            case IC_LDY_IMM:  _CPU_SET_REG_Y(state, state.data); break;
            case IC_SBC_IMM:  _CPU_SBC(state); break;
            case IC_BCC:      _CPU_COND_BRANCH(state, (state.P & 0x01) == 0); break;
            case IC_BCS:      _CPU_COND_BRANCH(state, (state.P & 0x01)); break;
            case IC_BNE:      _CPU_COND_BRANCH(state, (state.P & 0x02) == 0); break;
            case IC_BEQ:      _CPU_COND_BRANCH(state, (state.P & 0x02)); break;
            case IC_BVC:      _CPU_COND_BRANCH(state, (state.P & 0x40) == 0); break;
            case IC_BVS:      _CPU_COND_BRANCH(state, (state.P & 0x40)); break;
            case IC_BPL:      _CPU_COND_BRANCH(state, (state.P & 0x80) == 0); break;
            case IC_BMI:      _CPU_COND_BRANCH(state, (state.P & 0x80)); break;
            case IC_LDA_ABS:
            case IC_LDA_ABS_X:
            case IC_LDA_ABS_Y:
            case IC_LDX_ABS:
            case IC_LDX_ABS_Y:
            case IC_LDY_ABS:
            case IC_LDY_ABS_X:
            case IC_STA_ABS:
            case IC_STA_ABS_X:
            case IC_STA_ABS_Y:
            case IC_STX_ABS:
            case IC_STY_ABS:
            case IC_JMP:
            case IC_JMP_IND:
            case IC_ROL_ABS:
            case IC_ROL_ABS_X:
            case IC_ROR_ABS:
            case IC_ROR_ABS_X:
            case IC_ASL_ABS:
            case IC_ASL_ABS_X:
            case IC_LSR_ABS:
            case IC_LSR_ABS_X:
            case IC_ADC_ABS:
            case IC_ADC_ABS_X:
            case IC_ADC_ABS_Y:
            case IC_SBC_ABS:
            case IC_SBC_ABS_X:
            case IC_SBC_ABS_Y:
            case IC_AND_ABS:
            case IC_AND_ABS_X:
            case IC_AND_ABS_Y:
            case IC_ORA_ABS:
            case IC_ORA_ABS_X:
            case IC_ORA_ABS_Y:
            case IC_EOR_ABS:
            case IC_EOR_ABS_X:
            case IC_EOR_ABS_Y:
            case IC_BIT_ABS:
                state.memory_addr_low = state.data;
                _CPU_FETCH_OPERAND(state);
                return state;
            case IC_LDA_ZP:
            case IC_LDX_ZP:
            case IC_LDY_ZP:
            case IC_BIT_ZP:
            case IC_ROL_ZP:
            case IC_ROR_ZP:
            case IC_ASL_ZP:
            case IC_LSR_ZP:
            case IC_ADC_ZP:
            case IC_SBC_ZP:
            case IC_AND_ZP:
            case IC_ORA_ZP:
            case IC_EOR_ZP:
                state.address = state.data;
                return state;
            case IC_LDA_ZP_X:
            case IC_LDY_ZP_X:
            case IC_ROL_ZP_X:
            case IC_ROR_ZP_X:
            case IC_ASL_ZP_X:
            case IC_LSR_ZP_X:
            case IC_ADC_ZP_X:
            case IC_SBC_ZP_X:
            case IC_AND_ZP_X:
            case IC_ORA_ZP_X:
            case IC_EOR_ZP_X:
                state.address = state.data + state.X;
                return state; 
            case IC_LDX_ZP_Y:
                state.address = state.data + state.Y;
                return state;
            case IC_STA_ZP:
                state.rw = 1;
                state.address = state.data;
                state.data = state.A;
                return state;
            case IC_STA_ZP_X:
                state.rw = 1;
                state.address = state.data + state.X;
                state.data = state.A;
                return state;
            case IC_STX_ZP:
                state.rw = 1;
                state.address = state.data;
                state.data = state.X;
                return state;
            case IC_STX_ZP_Y:
                state.rw = 1;
                state.address = state.data + state.Y;
                state.data = state.X;
                return state;
            case IC_STY_ZP:
                state.rw = 1;
                state.address = state.data;
                state.data = state.Y;
                return state;
            case IC_STY_ZP_X:
                state.rw = 1;
                state.address = state.data + state.X;
                state.data = state.Y;
                return state;
            case IC_LDA_IND_X:
            case IC_STA_IND_X:
            case IC_ADC_IND_X:
            case IC_SBC_IND_X:
            case IC_AND_IND_X:
            case IC_ORA_IND_X:
            case IC_EOR_IND_X:
                state.address = state.data + state.X;
                return state;
            case IC_LDA_IND_Y:
            case IC_STA_IND_Y:
            case IC_ADC_IND_Y:
            case IC_SBC_IND_Y:
            case IC_AND_IND_Y:
            case IC_ORA_IND_Y:
            case IC_EOR_IND_Y:
                state.address = state.data;
                return state;
            case IC_JSR:
                state.memory_addr_low = state.data;
                return state;
            case IC_PHA:
            case IC_PHP:
                state.rw = 0;
                return state;
            case IC_RTS:
                // skip one cycle to align with RTI
                return state;
            case IC_RTI:
                _CPU_SET_REG_P(state, state.data);
                state.address = ((uint8_t)++state.S) + 0x0100;
                return state;
            case IC_PLA:
                _CPU_SET_REG_A(state, state.data);
                return state;
            case IC_PLP:
                _CPU_SET_REG_P(state, state.data);
                return state;
            default:
                break;
        }
    }
    else if (cycle == 2)
    {
        switch (state.instruction)
        {
            case IC_LDA_ABS:
            case IC_LDX_ABS:
            case IC_LDY_ABS:
            case IC_ROL_ABS:
            case IC_ROR_ABS:
            case IC_ASL_ABS:
            case IC_LSR_ABS:
            case IC_ADC_ABS:
            case IC_SBC_ABS:
            case IC_AND_ABS:
            case IC_ORA_ABS:
            case IC_EOR_ABS:
            case IC_BIT_ABS:
                state.address = (state.data << 8) | state.memory_addr_low;
                return state;
            case IC_LDA_ABS_X:
            case IC_LDY_ABS_X:
            case IC_ROL_ABS_X:
            case IC_ROR_ABS_X:
            case IC_ASL_ABS_X:
            case IC_LSR_ABS_X:
            case IC_ADC_ABS_X:
            case IC_SBC_ABS_X:
            case IC_AND_ABS_X:
            case IC_ORA_ABS_X:
            case IC_EOR_ABS_X:
                state.address = ((state.data << 8) | state.memory_addr_low) + state.X;
                return state;
            case IC_LDA_ABS_Y:
            case IC_LDX_ABS_Y:
            case IC_ADC_ABS_Y:
            case IC_SBC_ABS_Y:
            case IC_AND_ABS_Y:
            case IC_ORA_ABS_Y:
            case IC_EOR_ABS_Y:
                state.address = ((state.data << 8) | state.memory_addr_low) + state.Y;
                return state;
            case IC_STA_ABS:
                state.rw = 1;
                state.address = (state.data << 8) | state.memory_addr_low;
                state.data = state.A;
                return state;
            case IC_STA_ABS_X:
                state.rw = 1;
                state.address = ((state.data << 8) | state.memory_addr_low) + state.X;
                state.data = state.A;
                return state;
            case IC_STA_ABS_Y:
                state.rw = 1;
                state.address = ((state.data << 8) | state.memory_addr_low) + state.Y;
                state.data = state.A;
                return state;
            case IC_STX_ABS:
                state.rw = 1;
                state.address = (state.data << 8) | state.memory_addr_low;
                state.data = state.X;
                return state;
            case IC_STY_ABS:
                state.rw = 1;
                state.address = (state.data << 8) | state.memory_addr_low;
                state.data = state.Y;
                return state;
            case IC_LDA_IND_X:
            case IC_LDA_IND_Y:
            case IC_STA_IND_X:
            case IC_STA_IND_Y:
            case IC_ADC_IND_X:
            case IC_ADC_IND_Y:
            case IC_SBC_IND_X:
            case IC_SBC_IND_Y:
            case IC_AND_IND_X:
            case IC_AND_IND_Y:
            case IC_ORA_IND_X:
            case IC_ORA_IND_Y:
            case IC_EOR_IND_X:
            case IC_EOR_IND_Y:
                state.memory_addr_low = state.data;
                state.address +=1;
                return state;
            case IC_JMP:
                state.address = (state.data << 8) | state.memory_addr_low;
                state.PC = state.address;
                break;
            case IC_JMP_IND:
                state.address = (state.data << 8) | state.memory_addr_low;
                return state;
            case IC_BIT_ZP: _CPU_BIT(state); break;
            case IC_LDA_ZP: _CPU_SET_REG_A(state, state.data); break;
            case IC_LDX_ZP: _CPU_SET_REG_X(state, state.data); break;
            case IC_LDY_ZP: _CPU_SET_REG_Y(state, state.data); break;
            case IC_ADC_ZP: _CPU_ADC(state); break;
            case IC_SBC_ZP: _CPU_SBC(state); break;
            case IC_AND_ZP: _CPU_SET_REG_A(state, state.A & state.data); break;
            case IC_ORA_ZP: _CPU_SET_REG_A(state, state.A | state.data); break;
            case IC_EOR_ZP: _CPU_SET_REG_A(state, state.A ^ state.data); break;
            case IC_LDA_ZP_X:
                _CPU_SET_REG_A(state, state.data);
                return state; 
            case IC_LDY_ZP_X:
                _CPU_SET_REG_Y(state, state.data);
                return state;
            case IC_LDX_ZP_Y:
                _CPU_SET_REG_X(state, state.data);
                return state;
            case IC_JSR:
                state.rw = 1;
                state.data = (state.PC >> 8);
                state.address = ((uint8_t)state.S--) + 0x0100;
                return state;
            case IC_RTS:
            case IC_RTI:
                state.memory_addr_low = state.data;
                state.address = ((uint8_t)++state.S) + 0x0100;
                return state;
            case IC_PLA:
            case IC_PLP:
            case IC_ROL_ZP:
            case IC_ROL_ZP_X:
            case IC_ROR_ZP:
            case IC_ROR_ZP_X:
            case IC_ASL_ZP:
            case IC_ASL_ZP_X:
            case IC_LSR_ZP:
            case IC_LSR_ZP_X:
            case IC_STA_ZP_X:
            case IC_STX_ZP_Y:
            case IC_STY_ZP_X:
            case IC_ADC_ZP_X:
            case IC_SBC_ZP_X:
            case IC_AND_ZP_X:
            case IC_ORA_ZP_X:
            case IC_EOR_ZP_X:
                // empty cycles
                state.rw = 0;
                return state;
            default: break;
        }
    }
    else if (cycle == 3)
    {
        switch(state.instruction)
        {
            case IC_ADC_ABS:
            case IC_ADC_ABS_X:
            case IC_ADC_ABS_Y:
            case IC_ADC_ZP_X: 
                _CPU_ADC(state); 
                break;
            case IC_SBC_ABS:
            case IC_SBC_ABS_X:
            case IC_SBC_ABS_Y:
            case IC_SBC_ZP_X: 
                _CPU_SBC(state); 
                break;
            case IC_AND_ABS:
            case IC_AND_ABS_X:
            case IC_AND_ABS_Y:
            case IC_AND_ZP_X: 
                _CPU_SET_REG_A(state, state.A & state.data); 
                break;
            case IC_ORA_ABS:
            case IC_ORA_ABS_X:
            case IC_ORA_ABS_Y:
            case IC_ORA_ZP_X: 
                _CPU_SET_REG_A(state, state.A | state.data); 
                break;
            case IC_EOR_ABS:
            case IC_EOR_ABS_X:
            case IC_EOR_ABS_Y:
            case IC_EOR_ZP_X: 
                _CPU_SET_REG_A(state, state.A ^ state.data); 
                break;
            case IC_LDA_ABS:
            case IC_LDA_ABS_X:
            case IC_LDA_ABS_Y:
                _CPU_SET_REG_A(state, state.data);
                break;
            case IC_LDX_ABS:
            case IC_LDX_ABS_Y:
                _CPU_SET_REG_X(state, state.data);
                break;
            case IC_LDY_ABS:
            case IC_LDY_ABS_X:
                _CPU_SET_REG_Y(state, state.data);
                break;
            case IC_BIT_ABS:    _CPU_BIT(state); break;
            case IC_ROL_ABS:
            case IC_ROL_ABS_X:
            case IC_ROL_ZP:
            case IC_ROL_ZP_X:
                _CPU_ROL(state, state.data);
                state.rw = 1;
                return state;
            case IC_ROR_ABS:
            case IC_ROR_ABS_X:
            case IC_ROR_ZP:
            case IC_ROR_ZP_X:
                _CPU_ROR(state, state.data);
                state.rw = 1;
                return state;
            case IC_ASL_ABS:
            case IC_ASL_ABS_X:
            case IC_ASL_ZP:
            case IC_ASL_ZP_X:
                _CPU_ASL(state, state.data);
                state.rw = 1;
                return state;
            case IC_LSR_ABS:
            case IC_LSR_ABS_X:
            case IC_LSR_ZP:
            case IC_LSR_ZP_X:
                _CPU_LSR(state, state.data);
                state.rw = 1;
                return state;
            case IC_LDA_IND_X:
            case IC_ADC_IND_X:
            case IC_SBC_IND_X:
            case IC_AND_IND_X:
            case IC_ORA_IND_X:
            case IC_EOR_IND_X:
                state.address = (state.data << 8) | state.memory_addr_low;
                return state;
            case IC_LDA_IND_Y:
            case IC_ADC_IND_Y:
            case IC_SBC_IND_Y:
            case IC_AND_IND_Y:
            case IC_ORA_IND_Y:
            case IC_EOR_IND_Y:
                state.address = ((state.data << 8) | state.memory_addr_low) + state.Y;
                return state;
            case IC_STA_IND_X:
                state.rw = 1;
                state.address = (state.data << 8) | state.memory_addr_low;
                state.data = state.A;
                return state;
            case IC_STA_IND_Y:
                state.rw = 1;
                state.address = ((state.data << 8) | state.memory_addr_low) + state.Y;
                state.data = state.A;
                return state;
            case IC_JMP_IND:
                state.memory_addr_low = state.data;
                state.address += 1;
                return state;
            case IC_JSR:
                state.data = state.PC & 0xFF;
                state.address = ((uint8_t)state.S--) + 0x0100;
                return state;
           case IC_RTS:
           case IC_RTI:
                state.PC = (state.data << 8) | state.memory_addr_low;
                state.address = ++state.PC;
                return state;
            case IC_STA_ABS_X:
            case IC_STA_ABS_Y:
                // empty cycles
                return state;
           default: break;
        }
    }
    else if (cycle == 4)
    {
        switch (state.instruction)
        {
            case IC_LDA_IND_X: _CPU_SET_REG_A(state, state.data); return state;
            case IC_ADC_IND_X: _CPU_ADC(state); return state;
            case IC_SBC_IND_X: _CPU_SBC(state); return state;
            case IC_AND_IND_X: _CPU_SET_REG_A(state, state.A & state.data); return state;
            case IC_ORA_IND_X: _CPU_SET_REG_A(state, state.A | state.data); return state;
            case IC_EOR_IND_X: _CPU_SET_REG_A(state, state.A ^ state.data); return state;
            case IC_LDA_IND_Y: _CPU_SET_REG_A(state, state.data); break;
            case IC_ADC_IND_Y: _CPU_ADC(state); break;
            case IC_SBC_IND_Y: _CPU_SBC(state); break;
            case IC_AND_IND_Y: _CPU_SET_REG_A(state, state.A & state.data); break;
            case IC_ORA_IND_Y: _CPU_SET_REG_A(state, state.A | state.data); break;
            case IC_EOR_IND_Y: _CPU_SET_REG_A(state, state.A ^ state.data); break;
            case IC_JMP_IND:
                state.address = (state.data << 8) | state.memory_addr_low;
                state.PC = state.address;
                break;
            case IC_JSR:
                _CPU_FETCH_OPERAND(state);
                return state;
            case IC_ROL_ABS:
            case IC_ROL_ABS_X:
            case IC_ROL_ZP_X:
            case IC_ROR_ABS:
            case IC_ROR_ABS_X:
            case IC_ROR_ZP_X:
            case IC_ASL_ABS:
            case IC_ASL_ABS_X:
            case IC_ASL_ZP_X:
            case IC_LSR_ABS:
            case IC_LSR_ABS_X:
            case IC_LSR_ZP_X:
            case IC_STA_IND_X:
            case IC_STA_IND_Y:
            case IC_RTS:
            case IC_RTI:
                // empty cycles
                state.rw = 0;
                return state;
            default: break;
        }
    }
    else if (cycle == 5)
    {
        switch (state.instruction)
        {
            case IC_JSR:
                state.PC = (state.data << 8) | state.memory_addr_low;
                break;
            case IC_ROL_ABS_X:
            case IC_ROR_ABS_X:
            case IC_ASL_ABS_X:
            case IC_LSR_ABS_X:
                // empty cycle
                state.rw = 0;
                return state;
        }
    }

    _CPU_END(state);
    _CPU_FETCH_INSTRUCTION(state);
    return state;
}

#endif
