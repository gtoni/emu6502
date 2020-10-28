#ifndef _EMU6502_H_
#define _EMU6502_H_

#include "emu6502_opcodes.h"
#include <stdint.h>
#include <memory.h>
#include <unistd.h>

#include <stdio.h>

enum cpu_rw_mode
{
    CPU_RW_MODE_NONE    = 0,
    CPU_RW_MODE_READ    = 1,
    CPU_RW_MODE_WRITE   = 2
};

enum cpu_status_flags
{
    CPU_STATUS_FLAG_CARRY       = 0x01,
    CPU_STATUS_FLAG_ZERO        = 0x02,
    CPU_STATUS_FLAG_IRQDISABLE  = 0x04,
    CPU_STATUS_FLAG_DECIMAL     = 0x08,
    CPU_STATUS_FLAG_BREAK       = 0x10,
    CPU_STATUS_FLAG_OVERFLOW    = 0x40,
    CPU_STATUS_FLAG_NEGATIVE    = 0x80
};

struct cpu_state_
{
    uint16_t PC;             
    uint16_t pipeline;
    uint8_t  P; 
    uint8_t  S;

    uint16_t address;         
    uint8_t  rw_mode: 2;
    uint8_t  irq    : 1;
    uint8_t  nmi    : 1;
    uint8_t  in_nmi : 1;
    uint8_t  temp;

    // registers
    uint8_t  A;
    uint8_t  X;
    uint8_t  Y;

    uint8_t  data;
 
    // pins
/*    
    uint8_t  ad1 : 1;
    uint8_t  ad2 : 1;

    uint8_t  out0 : 1;
    uint8_t  out1 : 1;
    uint8_t  out2 : 1;

    uint8_t  oe1 : 1;
    uint8_t  oe2 : 1;
*/

    // pipeline
    uint8_t pipe_state : 2;

    //
    uint8_t wait_op : 2;
                                // 5 bits
    uint16_t cycle:3;        
};

#define _CPU_SET_STACK_PIPE(pipe)           ((pipe & 0xFFF0) | 0x0C)
#define _CPU_IS_STACK_PIPE(pipe)            ((pipe & 0x0F) == 0x0C)

#define _CPU_SET_LOAD_ADDR_OP(pipe, op)     ((pipe & 0xFFF0) | op)
#define _CPU_LOAD_ADDR_OP(pipe)             (pipe & 0x000F)

#define _CPU_SET_LOAD_OP(pipe, op)          ((pipe & 0xFF8F) | (op<<4))
#define _CPU_LOAD_OP(pipe)                  ((pipe>>4)&7)

#define _CPU_SET_LEA_OP(pipe, op)           ((pipe & 0xFE7F) | (op<<7))
#define _CPU_LEA_OP(pipe)                   ((pipe>>7)&3)

#define _CPU_SET_ALU_OP(pipe, op)           ((pipe & 0xE1FF) | (op<<9))
#define _CPU_ALU_OP(pipe)                   ((pipe>>9)&0x0F)

#define _CPU_SET_STACK_OP(pipe, op)         ((pipe & 0xE1FF) | (op<<9))
#define _CPU_STACK_OP(pipe)                 ((pipe>>9)&0x0F)

#define _CPU_SET_WRITE_OP(pipe, op)         ((pipe & 0x1FFF) | (op<<13))
#define _CPU_WRITE_OP(pipe)                 ((pipe>>13)&7)

typedef struct cpu_state_ cpu_state;

#define _CPU_SET_REG_P(cpu, v)        cpu.P = (v) | 0x20
#define _CPU_UPDATE_NZ(cpu, v)       _CPU_SET_REG_P(cpu, (cpu.P & 0x7D) | ((v & 0x80) | (v?0:0x02)))
#define _CPU_SET_REG(cpu, reg, v)    {cpu.reg = v; _CPU_UPDATE_NZ(cpu, cpu.reg);} 
#define _CPU_SET_REG_A(cpu, v)        _CPU_SET_REG(cpu, A, v)
#define _CPU_SET_REG_X(cpu, v)        _CPU_SET_REG(cpu, X, v)
#define _CPU_SET_REG_Y(cpu, v)        _CPU_SET_REG(cpu, Y, v)
#define _CPU_SET_REG_S(cpu, v)        cpu.S = v

#define _CPU_END(cpu)               {cpu.cycle = 0;}
#define _CPU_COND_BRANCH(cpu, cond) {\
    cpu.pipeline = _CPU_SET_LOAD_OP(cpu.pipeline, CPU_LOAD_OP_FETCH) |\
                   _CPU_SET_WRITE_OP(cpu.pipeline, ((cond) ? CPU_WRITE_OP_BRANCH : CPU_WRITE_OP_NONE));\
}


#define _CPU_CMP(cpu, reg) {\
    uint16_t tmp = ((uint16_t)reg) - cpu.data;\
    _CPU_UPDATE_NZ(cpu, (tmp & 0xFF));\
    _CPU_SET_REG_P(cpu, (cpu.P & 0xFE) | ((~tmp >> 8) & 1));\
}

#define _CPU_PIPELINE(addr_op, stack_op_, lea_op_, load_op_, alu_op_, write_op_, wait_op_)\
    {state.pipeline = \
        _CPU_SET_LOAD_ADDR_OP(0, addr_op) | \
        _CPU_SET_STACK_OP(0, stack_op_) |\
        _CPU_SET_LEA_OP(0, lea_op_) |\
        _CPU_SET_LOAD_OP(0, load_op_) |\
        _CPU_SET_ALU_OP(0, alu_op_) | \
        _CPU_SET_WRITE_OP(0, write_op_); state.wait_op = wait_op_;}

#define _CPU_ALU_PIPELINE(addr_op, lea_op_, load_op_, write_op_, alu_op_)\
    _CPU_PIPELINE(addr_op, CPU_STACK_OP_NONE, lea_op_, load_op_, alu_op_, write_op_, 0)

#define _CPU_LS_PIPELINE(addr_op, lea_op_, load_op_, write_op_)\
    _CPU_PIPELINE(addr_op, CPU_STACK_OP_NONE, lea_op_, load_op_, CPU_ALU_OP_NONE, write_op_, 0)

#define _CPU_PIPELINE_LD_IMM(reg)\
    _CPU_LS_PIPELINE(CPU_LOAD_ADDRESS_OP_NONE, CPU_LEA_OP_NONE, CPU_LOAD_OP_FETCH, CPU_WRITE_OP_##reg)

#define _CPU_PIPELINE_LD_ABS(reg)\
    _CPU_LS_PIPELINE(CPU_LOAD_ADDRESS_OP_ABS, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_##reg)

#define _CPU_PIPELINE_LD_ABS_X(reg)\
    _CPU_LS_PIPELINE(CPU_LOAD_ADDRESS_OP_ABS_X, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_##reg)

#define _CPU_PIPELINE_LD_ABS_Y(reg)\
    _CPU_LS_PIPELINE(CPU_LOAD_ADDRESS_OP_ABS_Y, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_##reg)

#define _CPU_PIPELINE_LD_ZP(reg)\
    _CPU_LS_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_##reg)

#define _CPU_PIPELINE_LD_ZP_X(reg)\
    _CPU_LS_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP_X, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_##reg)

#define _CPU_PIPELINE_LD_ZP_Y(reg)\
    _CPU_LS_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP_Y, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_##reg)

#define _CPU_PIPELINE_LD_IND_X(reg)\
    _CPU_LS_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP_X, CPU_LEA_OP_INDIRECT, CPU_LOAD_OP_MEM, CPU_WRITE_OP_##reg)

#define _CPU_PIPELINE_LD_IND_Y(reg)\
    _CPU_LS_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP, CPU_LEA_OP_INDIRECT_Y, CPU_LOAD_OP_MEM, CPU_WRITE_OP_##reg)

#define _CPU_PIPELINE_ST_ABS(reg)\
    _CPU_LS_PIPELINE(CPU_LOAD_ADDRESS_OP_ABS, CPU_LEA_OP_NONE, CPU_LOAD_OP_##reg, CPU_WRITE_OP_MEM)

#define _CPU_PIPELINE_ST_ABS_X(reg)\
    _CPU_LS_PIPELINE(CPU_LOAD_ADDRESS_OP_ABS_X, CPU_LEA_OP_NONE, CPU_LOAD_OP_##reg, CPU_WRITE_OP_MEM)

#define _CPU_PIPELINE_ST_ABS_Y(reg)\
    _CPU_LS_PIPELINE(CPU_LOAD_ADDRESS_OP_ABS_Y, CPU_LEA_OP_NONE, CPU_LOAD_OP_##reg, CPU_WRITE_OP_MEM)

#define _CPU_PIPELINE_ST_ZP(reg)\
    _CPU_LS_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP, CPU_LEA_OP_NONE, CPU_LOAD_OP_##reg, CPU_WRITE_OP_MEM)

#define _CPU_PIPELINE_ST_ZP_X(reg)\
    _CPU_LS_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP_X, CPU_LEA_OP_NONE, CPU_LOAD_OP_##reg, CPU_WRITE_OP_MEM)

#define _CPU_PIPELINE_ST_ZP_Y(reg)\
    _CPU_LS_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP_Y, CPU_LEA_OP_NONE, CPU_LOAD_OP_##reg, CPU_WRITE_OP_MEM)

#define _CPU_PIPELINE_ST_IND_X(reg)\
    _CPU_LS_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP_X, CPU_LEA_OP_INDIRECT, CPU_LOAD_OP_##reg, CPU_WRITE_OP_MEM)

#define _CPU_PIPELINE_ST_IND_Y(reg)\
    _CPU_LS_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP, CPU_LEA_OP_INDIRECT_Y, CPU_LOAD_OP_##reg, CPU_WRITE_OP_MEM)

#define _CPU_PIPELINE_TST_IMM(tst)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_NONE, CPU_LEA_OP_NONE, CPU_LOAD_OP_FETCH, CPU_WRITE_OP_NONE, CPU_ALU_OP_##tst)

#define _CPU_PIPELINE_TST_ABS(tst)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_ABS, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_NONE, CPU_ALU_OP_##tst)

#define _CPU_PIPELINE_TST_ABS_X(tst)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_ABS_X, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_NONE, CPU_ALU_OP_##tst)

#define _CPU_PIPELINE_TST_ABS_Y(tst)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_ABS_Y, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_NONE, CPU_ALU_OP_##tst)

#define _CPU_PIPELINE_TST_ZP(tst)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_NONE, CPU_ALU_OP_##tst)

#define _CPU_PIPELINE_TST_ZP_X(tst)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP_X, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_NONE, CPU_ALU_OP_##tst)

#define _CPU_PIPELINE_TST_ZP_Y(tst)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP_Y, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_NONE, CPU_ALU_OP_##tst)

#define _CPU_PIPELINE_TST_IND_X(tst)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP_X, CPU_LEA_OP_INDIRECT, CPU_LOAD_OP_MEM, CPU_WRITE_OP_NONE, CPU_ALU_OP_##tst)

#define _CPU_PIPELINE_TST_IND_Y(tst)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP, CPU_LEA_OP_INDIRECT_Y, CPU_LOAD_OP_MEM, CPU_WRITE_OP_NONE, CPU_ALU_OP_##tst)

#define _CPU_PIPELINE_ALU_IMM(op)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_NONE, CPU_LEA_OP_NONE, CPU_LOAD_OP_FETCH, CPU_WRITE_OP_A, CPU_ALU_OP_##op)

#define _CPU_PIPELINE_ALU_ABS(op)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_ABS, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_A, CPU_ALU_OP_##op)

#define _CPU_PIPELINE_ALU_ABS_X(op)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_ABS_X, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_A, CPU_ALU_OP_##op)

#define _CPU_PIPELINE_ALU_ABS_Y(op)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_ABS_Y, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_A, CPU_ALU_OP_##op)

#define _CPU_PIPELINE_ALU_ZP(op)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_A, CPU_ALU_OP_##op)

#define _CPU_PIPELINE_ALU_ZP_X(op)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP_X, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_A, CPU_ALU_OP_##op)

#define _CPU_PIPELINE_ALU_ZP_Y(op)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP_Y, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_A, CPU_ALU_OP_##op)

#define _CPU_PIPELINE_ALU_IND_X(op)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP_X, CPU_LEA_OP_INDIRECT, CPU_LOAD_OP_MEM, CPU_WRITE_OP_A, CPU_ALU_OP_##op)

#define _CPU_PIPELINE_ALU_IND_Y(op)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP, CPU_LEA_OP_INDIRECT_Y, CPU_LOAD_OP_MEM, CPU_WRITE_OP_A, CPU_ALU_OP_##op)

#define _CPU_PIPELINE_RMW_ACC(op)\
    _CPU_ALU_PIPELINE(CPU_LOAD_OP_NONE, CPU_LEA_OP_NONE, CPU_LOAD_OP_A, CPU_WRITE_OP_A, CPU_ALU_OP_##op)

#define _CPU_PIPELINE_RMW_ABS(op)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_ABS, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_MEM, CPU_ALU_OP_##op)

#define _CPU_PIPELINE_RMW_ABS_X(op)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_ABS_X, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_MEM, CPU_ALU_OP_##op)

#define _CPU_PIPELINE_RMW_ZP(op)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_MEM, CPU_ALU_OP_##op)

#define _CPU_PIPELINE_RMW_ZP_X(op)\
    _CPU_ALU_PIPELINE(CPU_LOAD_ADDRESS_OP_ZP_X, CPU_LEA_OP_NONE, CPU_LOAD_OP_MEM, CPU_WRITE_OP_MEM, CPU_ALU_OP_##op)

enum CPU_LOAD_ADDRESS_OP
{
    CPU_LOAD_ADDRESS_OP_NONE,
    CPU_LOAD_ADDRESS_OP_LOW     = 0x01,
    CPU_LOAD_ADDRESS_OP_HIGH    = 0x02,
    CPU_LOAD_ADDRESS_OP_X       = 0x04,
    CPU_LOAD_ADDRESS_OP_Y       = 0x08,

    CPU_LOAD_ADDRESS_OP_ZP      = CPU_LOAD_ADDRESS_OP_LOW,
    CPU_LOAD_ADDRESS_OP_ZP_X    = CPU_LOAD_ADDRESS_OP_LOW | CPU_LOAD_ADDRESS_OP_X,
    CPU_LOAD_ADDRESS_OP_ZP_Y    = CPU_LOAD_ADDRESS_OP_LOW | CPU_LOAD_ADDRESS_OP_Y,
    CPU_LOAD_ADDRESS_OP_ABS     = CPU_LOAD_ADDRESS_OP_LOW | CPU_LOAD_ADDRESS_OP_HIGH,
    CPU_LOAD_ADDRESS_OP_ABS_X   = CPU_LOAD_ADDRESS_OP_ABS | CPU_LOAD_ADDRESS_OP_X,
    CPU_LOAD_ADDRESS_OP_ABS_Y   = CPU_LOAD_ADDRESS_OP_ABS | CPU_LOAD_ADDRESS_OP_Y
};

enum CPU_LEA_OP
{
    CPU_LEA_OP_NONE,
    CPU_LEA_OP_VECTOR,
    CPU_LEA_OP_INDIRECT,
    CPU_LEA_OP_INDIRECT_Y
};

enum CPU_STACK_OP
{
    // 3 bits for the parameter and 1 bit for the action
    CPU_STACK_OP_NONE,
    CPU_STACK_OP_PUSH_A,
    CPU_STACK_OP_PUSH_X,
    CPU_STACK_OP_PUSH_Y,
    CPU_STACK_OP_PUSH_P,
    CPU_STACK_OP_PUSH_PC,
    CPU_STACK_OP_PUSH_PC_INC,
    CPU_STACK_OP_PUSH_INT,
    
    // The pull constants are the push constant values but inverted
    CPU_STACK_OP_PULL_INT,
    CPU_STACK_OP_PULL_PC_INC,
    CPU_STACK_OP_PULL_PC,
    CPU_STACK_OP_PULL_P,
    CPU_STACK_OP_PULL_Y,
    CPU_STACK_OP_PULL_X,
    CPU_STACK_OP_PULL_A
};

enum CPU_LOAD_OP
{
    CPU_LOAD_OP_NONE,
    CPU_LOAD_OP_FETCH,
    CPU_LOAD_OP_MEM,
    CPU_LOAD_OP_A,
    CPU_LOAD_OP_X,
    CPU_LOAD_OP_Y,
    CPU_LOAD_OP_AX
};

enum CPU_ALU_OP
{
    CPU_ALU_OP_NONE,
    CPU_ALU_OP_AND,
    CPU_ALU_OP_OR,
    CPU_ALU_OP_XOR,
    CPU_ALU_OP_ADD,
    CPU_ALU_OP_SUB,
    CPU_ALU_OP_CMP,
    CPU_ALU_OP_CMP_X,
    CPU_ALU_OP_CMP_Y,
    CPU_ALU_OP_DEC,
    CPU_ALU_OP_INC,
    CPU_ALU_OP_ASL,
    CPU_ALU_OP_LSR,
    CPU_ALU_OP_ROL,
    CPU_ALU_OP_ROR,
    CPU_ALU_OP_BIT
};

enum CPU_WRITE_OP
{
    CPU_WRITE_OP_NONE,
    CPU_WRITE_OP_MEM,
    CPU_WRITE_OP_A,
    CPU_WRITE_OP_X,
    CPU_WRITE_OP_Y,
    CPU_WRITE_OP_PC,
    CPU_WRITE_OP_AX,
    CPU_WRITE_OP_BRANCH
};

static cpu_state cpu_reset()
{
    cpu_state state;
    memset(&state, 0, sizeof(cpu_state));
    _CPU_SET_REG_P(state, CPU_STATUS_FLAG_IRQDISABLE);
    state.rw_mode = CPU_RW_MODE_READ;
    state.S = 0xFF;
    state.pipeline = _CPU_SET_STACK_PIPE(
                        _CPU_SET_STACK_OP(0, CPU_STACK_OP_PUSH_INT) |
                        _CPU_SET_LEA_OP(0, CPU_LEA_OP_VECTOR) |
                        _CPU_SET_WRITE_OP(0, CPU_WRITE_OP_PC));
    state.cycle = 1;
    return state; 
}

static cpu_state cpu_execute(cpu_state state)
{
    uint8_t cycle = state.cycle++;

    if (state.rw_mode == CPU_RW_MODE_READ && state.address == state.PC)
        state.PC++;

    if (cycle == 0)
    {
        if (state.nmi || (state.irq && !(state.P & CPU_STATUS_FLAG_IRQDISABLE)))
            state.data = 0;

        switch (state.data)
        {
            case IC_BRK:
                _CPU_SET_REG_P(state, state.P | CPU_STATUS_FLAG_BREAK);
                state.pipeline = _CPU_SET_STACK_PIPE(
                                    _CPU_SET_STACK_OP(0, CPU_STACK_OP_PUSH_INT) |
                                    _CPU_SET_LEA_OP(0, CPU_LEA_OP_VECTOR) |
                                    _CPU_SET_WRITE_OP(0, CPU_WRITE_OP_PC));
                break;
            case IC_PHP:
                state.pipeline = _CPU_SET_STACK_PIPE(_CPU_SET_STACK_OP(0, CPU_STACK_OP_PUSH_P));
                state.wait_op = 1;
                break;
            case IC_PHA:
                state.pipeline = _CPU_SET_STACK_PIPE(_CPU_SET_STACK_OP(0, CPU_STACK_OP_PUSH_A));
                state.wait_op = 1;
                break;
            case IC_PLP:
                state.pipeline = _CPU_SET_STACK_PIPE(_CPU_SET_STACK_OP(0, CPU_STACK_OP_PULL_P));
                state.wait_op = 1;
                break;
            case IC_PLA:
                state.pipeline = _CPU_SET_STACK_PIPE(_CPU_SET_STACK_OP(0, CPU_STACK_OP_PULL_A));
                state.wait_op = 1;
                break;
            case IC_RTS:
                state.pipeline = _CPU_SET_STACK_PIPE(_CPU_SET_STACK_OP(0, CPU_STACK_OP_PULL_PC_INC));
                state.wait_op = 2;
                break;
            case IC_RTI:
                state.pipeline = _CPU_SET_STACK_PIPE(_CPU_SET_STACK_OP(0, CPU_STACK_OP_PULL_INT));
                state.wait_op = 1;
                break;
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
            case IC_IL_NOP_IMM0:
            case IC_IL_NOP_IMM1:
            case IC_IL_NOP_IMM2:
            case IC_IL_NOP_IMM3:
            case IC_IL_NOP_IMM4:
            case IC_IL_NOP_IMM5:
                state.wait_op = 1;
                break;
            case IC_JSR:
                state.pipeline = _CPU_SET_STACK_PIPE(
                                    _CPU_SET_LOAD_ADDR_OP(0, CPU_LOAD_ADDRESS_OP_ABS) |
                                    _CPU_SET_STACK_OP(0, CPU_STACK_OP_PUSH_PC) |
                                    _CPU_SET_WRITE_OP(0, CPU_WRITE_OP_PC));
                break;
                // DEFAULTS
            case IC_LDA_IMM:        _CPU_PIPELINE_LD_IMM(A); break;
            case IC_LDA_ABS:        _CPU_PIPELINE_LD_ABS(A); break;
            case IC_LDA_ABS_X:      _CPU_PIPELINE_LD_ABS_X(A); break;
            case IC_LDA_ABS_Y:      _CPU_PIPELINE_LD_ABS_Y(A); break;
            case IC_LDA_ZP:         _CPU_PIPELINE_LD_ZP(A); break;
            case IC_LDA_ZP_X:       _CPU_PIPELINE_LD_ZP_X(A); break;
            case IC_LDA_IND_X:      _CPU_PIPELINE_LD_IND_X(A); break;
            case IC_LDA_IND_Y:      _CPU_PIPELINE_LD_IND_Y(A); break;
            case IC_LDX_IMM:        _CPU_PIPELINE_LD_IMM(X); break;
            case IC_LDX_ABS:        _CPU_PIPELINE_LD_ABS(X); break;
            case IC_LDX_ABS_Y:      _CPU_PIPELINE_LD_ABS_Y(X); break;
            case IC_LDX_ZP:         _CPU_PIPELINE_LD_ZP(X);  break; 
            case IC_LDX_ZP_Y:       _CPU_PIPELINE_LD_ZP_Y(X); break;
            case IC_LDY_IMM:        _CPU_PIPELINE_LD_IMM(Y); break;
            case IC_LDY_ABS:        _CPU_PIPELINE_LD_ABS(Y); break;
            case IC_LDY_ABS_X:      _CPU_PIPELINE_LD_ABS_X(Y); break;
            case IC_LDY_ZP:         _CPU_PIPELINE_LD_ZP(Y);  break; 
            case IC_LDY_ZP_X:       _CPU_PIPELINE_LD_ZP_X(Y); break;
            case IC_IL_LAX_ABS:     _CPU_PIPELINE_LD_ABS(AX); break;
            case IC_IL_LAX_ABS_Y:   _CPU_PIPELINE_LD_ABS_Y(AX); break;
            case IC_IL_LAX_ZP:      _CPU_PIPELINE_LD_ZP(AX); break;
            case IC_IL_LAX_ZP_Y:    _CPU_PIPELINE_LD_ZP_Y(AX); break;
            case IC_IL_LAX_IND_X:   _CPU_PIPELINE_LD_IND_X(AX); break;
            case IC_IL_LAX_IND_Y:   _CPU_PIPELINE_LD_IND_Y(AX); break;
            case IC_STA_ABS:        _CPU_PIPELINE_ST_ABS(A); break;
            case IC_STA_ABS_X:      _CPU_PIPELINE_ST_ABS_X(A); break;
            case IC_STA_ABS_Y:      _CPU_PIPELINE_ST_ABS_Y(A); break;
            case IC_STA_ZP:         _CPU_PIPELINE_ST_ZP(A); break;
            case IC_STA_ZP_X:       _CPU_PIPELINE_ST_ZP_X(A); break;
            case IC_STA_IND_X:      _CPU_PIPELINE_ST_IND_X(A); break;
            case IC_STA_IND_Y:      _CPU_PIPELINE_ST_IND_Y(A); break;
            case IC_STX_ABS:        _CPU_PIPELINE_ST_ABS(X); break; 
            case IC_STX_ZP:         _CPU_PIPELINE_ST_ZP(X); break;
            case IC_STX_ZP_Y:       _CPU_PIPELINE_ST_ZP_Y(X); break;
            case IC_STY_ABS:        _CPU_PIPELINE_ST_ABS(Y); break; 
            case IC_STY_ZP:         _CPU_PIPELINE_ST_ZP(Y); break;
            case IC_STY_ZP_X:       _CPU_PIPELINE_ST_ZP_X(Y); break;
            case IC_IL_SAX_ABS:     _CPU_PIPELINE_ST_ABS(AX); break;
            case IC_IL_SAX_ZP:      _CPU_PIPELINE_ST_ZP(AX); break;
            case IC_IL_SAX_ZP_Y:    _CPU_PIPELINE_ST_ZP_Y(AX); break;
            case IC_IL_SAX_IND_X:   _CPU_PIPELINE_ST_IND_X(AX); break;
            case IC_CMP_IMM:        _CPU_PIPELINE_TST_IMM(CMP); break;
            case IC_CMP_ABS:        _CPU_PIPELINE_TST_ABS(CMP); break;
            case IC_CMP_ABS_X:      _CPU_PIPELINE_TST_ABS_X(CMP); break;
            case IC_CMP_ABS_Y:      _CPU_PIPELINE_TST_ABS_Y(CMP); break;
            case IC_CMP_ZP:         _CPU_PIPELINE_TST_ZP(CMP); break;
            case IC_CMP_ZP_X:       _CPU_PIPELINE_TST_ZP_X(CMP); break;
            case IC_CMP_IND_X:      _CPU_PIPELINE_TST_IND_X(CMP); break;
            case IC_CMP_IND_Y:      _CPU_PIPELINE_TST_IND_Y(CMP); break;
            case IC_CPX_IMM:        _CPU_PIPELINE_TST_IMM(CMP_X); break;
            case IC_CPX_ABS:        _CPU_PIPELINE_TST_ABS(CMP_X); break;
            case IC_CPX_ZP:         _CPU_PIPELINE_TST_ZP(CMP_X); break;
            case IC_CPY_IMM:        _CPU_PIPELINE_TST_IMM(CMP_Y); break;
            case IC_CPY_ABS:        _CPU_PIPELINE_TST_ABS(CMP_Y); break;
            case IC_CPY_ZP:         _CPU_PIPELINE_TST_ZP(CMP_Y); break;
            case IC_BIT_ABS:        _CPU_PIPELINE_TST_ABS(BIT); break;
            case IC_BIT_ZP:         _CPU_PIPELINE_TST_ZP(BIT); break; 
            case IC_ORA_IMM:        _CPU_PIPELINE_ALU_IMM(OR); break;
            case IC_ORA_ABS:        _CPU_PIPELINE_ALU_ABS(OR); break;
            case IC_ORA_ABS_X:      _CPU_PIPELINE_ALU_ABS_X(OR); break;
            case IC_ORA_ABS_Y:      _CPU_PIPELINE_ALU_ABS_Y(OR); break;
            case IC_ORA_ZP:         _CPU_PIPELINE_ALU_ZP(OR); break;
            case IC_ORA_ZP_X:       _CPU_PIPELINE_ALU_ZP_X(OR); break;
            case IC_ORA_IND_X:      _CPU_PIPELINE_ALU_IND_X(OR); break;
            case IC_ORA_IND_Y:      _CPU_PIPELINE_ALU_IND_Y(OR); break;
            case IC_AND_IMM:        _CPU_PIPELINE_ALU_IMM(AND); break;
            case IC_AND_ABS:        _CPU_PIPELINE_ALU_ABS(AND); break;
            case IC_AND_ABS_X:      _CPU_PIPELINE_ALU_ABS_X(AND); break;
            case IC_AND_ABS_Y:      _CPU_PIPELINE_ALU_ABS_Y(AND); break;
            case IC_AND_ZP:         _CPU_PIPELINE_ALU_ZP(AND); break;
            case IC_AND_ZP_X:       _CPU_PIPELINE_ALU_ZP_X(AND); break;
            case IC_AND_IND_X:      _CPU_PIPELINE_ALU_IND_X(AND); break;
            case IC_AND_IND_Y:      _CPU_PIPELINE_ALU_IND_Y(AND); break;
            case IC_EOR_IMM:        _CPU_PIPELINE_ALU_IMM(XOR); break;
            case IC_EOR_ABS:        _CPU_PIPELINE_ALU_ABS(XOR); break;
            case IC_EOR_ABS_X:      _CPU_PIPELINE_ALU_ABS_X(XOR); break;
            case IC_EOR_ABS_Y:      _CPU_PIPELINE_ALU_ABS_Y(XOR); break;
            case IC_EOR_ZP:         _CPU_PIPELINE_ALU_ZP(XOR); break;
            case IC_EOR_ZP_X:       _CPU_PIPELINE_ALU_ZP_X(XOR); break;
            case IC_EOR_IND_X:      _CPU_PIPELINE_ALU_IND_X(XOR); break;
            case IC_EOR_IND_Y:      _CPU_PIPELINE_ALU_IND_Y(XOR); break;
            case IC_ADC_IMM:        _CPU_PIPELINE_ALU_IMM(ADD); break;
            case IC_ADC_ABS:        _CPU_PIPELINE_ALU_ABS(ADD); break;
            case IC_ADC_ABS_X:      _CPU_PIPELINE_ALU_ABS_X(ADD); break;
            case IC_ADC_ABS_Y:      _CPU_PIPELINE_ALU_ABS_Y(ADD); break;
            case IC_ADC_ZP:         _CPU_PIPELINE_ALU_ZP(ADD); break;
            case IC_ADC_ZP_X:       _CPU_PIPELINE_ALU_ZP_X(ADD); break;
            case IC_ADC_IND_X:      _CPU_PIPELINE_ALU_IND_X(ADD); break;
            case IC_ADC_IND_Y:      _CPU_PIPELINE_ALU_IND_Y(ADD); break;
            case IC_IL_SBC_IMM:
            case IC_SBC_IMM:        _CPU_PIPELINE_ALU_IMM(SUB); break;
            case IC_SBC_ABS:        _CPU_PIPELINE_ALU_ABS(SUB); break;
            case IC_SBC_ABS_X:      _CPU_PIPELINE_ALU_ABS_X(SUB); break;
            case IC_SBC_ABS_Y:      _CPU_PIPELINE_ALU_ABS_Y(SUB); break;
            case IC_SBC_ZP:         _CPU_PIPELINE_ALU_ZP(SUB); break;
            case IC_SBC_ZP_X:       _CPU_PIPELINE_ALU_ZP_X(SUB); break;
            case IC_SBC_IND_X:      _CPU_PIPELINE_ALU_IND_X(SUB); break;
            case IC_SBC_IND_Y:      _CPU_PIPELINE_ALU_IND_Y(SUB); break;

            case IC_ASL_ACC:        _CPU_PIPELINE_RMW_ACC(ASL); break;
            case IC_ASL_ABS:        _CPU_PIPELINE_RMW_ABS(ASL); break;
            case IC_ASL_ABS_X:      _CPU_PIPELINE_RMW_ABS_X(ASL); break;
            case IC_ASL_ZP:         _CPU_PIPELINE_RMW_ZP(ASL); break;
            case IC_ASL_ZP_X:       _CPU_PIPELINE_RMW_ZP_X(ASL); break;
            case IC_LSR_ACC:        _CPU_PIPELINE_RMW_ACC(LSR); break;
            case IC_LSR_ABS:        _CPU_PIPELINE_RMW_ABS(LSR); break;
            case IC_LSR_ABS_X:      _CPU_PIPELINE_RMW_ABS_X(LSR); break;
            case IC_LSR_ZP:         _CPU_PIPELINE_RMW_ZP(LSR); break;
            case IC_LSR_ZP_X:       _CPU_PIPELINE_RMW_ZP_X(LSR); break;
            case IC_ROL_ACC:        _CPU_PIPELINE_RMW_ACC(ROL); break;
            case IC_ROL_ABS:        _CPU_PIPELINE_RMW_ABS(ROL); break;
            case IC_ROL_ABS_X:      _CPU_PIPELINE_RMW_ABS_X(ROL); break;
            case IC_ROL_ZP:         _CPU_PIPELINE_RMW_ZP(ROL); break;
            case IC_ROL_ZP_X:       _CPU_PIPELINE_RMW_ZP_X(ROL); break;
            case IC_ROR_ACC:        _CPU_PIPELINE_RMW_ACC(ROR); break;
            case IC_ROR_ABS:        _CPU_PIPELINE_RMW_ABS(ROR); break;
            case IC_ROR_ABS_X:      _CPU_PIPELINE_RMW_ABS_X(ROR); break;
            case IC_ROR_ZP:         _CPU_PIPELINE_RMW_ZP(ROR); break;
            case IC_ROR_ZP_X:       _CPU_PIPELINE_RMW_ZP_X(ROR); break;
            case IC_DEC_ABS:        _CPU_PIPELINE_RMW_ABS(DEC); break;
            case IC_DEC_ABS_X:      _CPU_PIPELINE_RMW_ABS_X(DEC); break;
            case IC_DEC_ZP:         _CPU_PIPELINE_RMW_ZP(DEC); break;
            case IC_DEC_ZP_X:       _CPU_PIPELINE_RMW_ZP_X(DEC); break;
            case IC_INC_ABS:        _CPU_PIPELINE_RMW_ABS(INC); break;
            case IC_INC_ABS_X:      _CPU_PIPELINE_RMW_ABS_X(INC); break;
            case IC_INC_ZP:         _CPU_PIPELINE_RMW_ZP(INC); break;
            case IC_INC_ZP_X:       _CPU_PIPELINE_RMW_ZP_X(INC); break;
            case IC_JMP:
                state.pipeline =_CPU_SET_LOAD_ADDR_OP(0, CPU_LOAD_ADDRESS_OP_ABS) |
                                _CPU_SET_WRITE_OP(state.pipeline, CPU_WRITE_OP_PC);
                break;
            case IC_JMP_IND:
                state.pipeline = _CPU_SET_LOAD_ADDR_OP(0, CPU_LOAD_ADDRESS_OP_ABS) |
                                 _CPU_SET_LEA_OP(0, CPU_LEA_OP_INDIRECT) | 
                                 _CPU_SET_WRITE_OP(0, CPU_WRITE_OP_PC);
                break;
            case IC_BCC:      _CPU_COND_BRANCH(state, (state.P & 0x01) == 0); break;
            case IC_BCS:      _CPU_COND_BRANCH(state, (state.P & 0x01)); break;
            case IC_BNE:      _CPU_COND_BRANCH(state, (state.P & 0x02) == 0); break;
            case IC_BEQ:      _CPU_COND_BRANCH(state, (state.P & 0x02)); break;
            case IC_BVC:      _CPU_COND_BRANCH(state, (state.P & 0x40) == 0); break;
            case IC_BVS:      _CPU_COND_BRANCH(state, (state.P & 0x40)); break;
            case IC_BPL:      _CPU_COND_BRANCH(state, (state.P & 0x80) == 0); break;
            case IC_BMI:      _CPU_COND_BRANCH(state, (state.P & 0x80)); break;
            case IC_IL_NOP_ABS:     _CPU_PIPELINE_LD_ABS(NONE); break;
            case IC_IL_NOP_ABS_X0:  
            case IC_IL_NOP_ABS_X1:
            case IC_IL_NOP_ABS_X2:
            case IC_IL_NOP_ABS_X3:
            case IC_IL_NOP_ABS_X4:
            case IC_IL_NOP_ABS_X5:  _CPU_PIPELINE_LD_ABS_X(NONE); break;
            case IC_IL_NOP_ZP0:
            case IC_IL_NOP_ZP1:
            case IC_IL_NOP_ZP2:     _CPU_PIPELINE_LD_ZP(NONE); break;
            case IC_IL_NOP_ZP_X0:
            case IC_IL_NOP_ZP_X1:
            case IC_IL_NOP_ZP_X2:
            case IC_IL_NOP_ZP_X3:
            case IC_IL_NOP_ZP_X4:
            case IC_IL_NOP_ZP_X5:   _CPU_PIPELINE_LD_ZP_X(NONE); break;
            case IC_IL_NOP_IMP0:
            case IC_IL_NOP_IMP1:
            case IC_IL_NOP_IMP2:
            case IC_IL_NOP_IMP3:
            case IC_IL_NOP_IMP4:
                state.rw_mode = CPU_RW_MODE_READ;
                state.address = state.PC;
                return state;
                // END DEFAULTS
            default: 
            //    state.rw_mode = CPU_RW_MODE_READ;
            //    state.address = state.PC;
            //    printf("Unknown instruction: 0x%02X\n", state.data);
                break;
        }

        //debug
        return state;
    }

    /* CPU Pipeline */ 
    {
        uint8_t load_addr = _CPU_LOAD_ADDR_OP(state.pipeline);
        uint8_t load_addr_xy = load_addr & 0x0C;

        // load address low
        if (load_addr)
        {
            switch(load_addr & 0x03)
            {
                case CPU_LOAD_ADDRESS_OP_ABS:
                    if (state.pipe_state == 0)
                    {
                        state.pipe_state = 1;
                        state.rw_mode = CPU_RW_MODE_READ;
                        state.address = state.PC++;
                        return state;
                    }
                    else
                    {
                        state.temp = state.data;
                        state.pipeline = _CPU_SET_LOAD_ADDR_OP(state.pipeline, load_addr & ~CPU_LOAD_ADDRESS_OP_LOW);
                        state.pipe_state = 0;
                    }
                    break;
                case CPU_LOAD_ADDRESS_OP_ZP:
                    if (state.pipe_state == 0)
                    {
                        state.pipe_state = 1;
                        state.rw_mode = CPU_RW_MODE_READ;
                        state.address = state.PC++;
                        return state;
                    }
                    else
                    {
                        state.address = state.data;
                        if (load_addr_xy == CPU_LOAD_ADDRESS_OP_X)
                            state.address = (state.address + state.X) & 0xFF;
                        if (load_addr_xy == CPU_LOAD_ADDRESS_OP_Y)
                            state.address = (state.address + state.Y) & 0xFF;
                        state.pipeline = _CPU_SET_LOAD_ADDR_OP(state.pipeline, load_addr_xy);
                        state.pipe_state = 0;
                    }
                    break;
                default: break;
            }
        }

        // stack 
        if (load_addr_xy == 0x0C && _CPU_STACK_OP(state.pipeline))
        {
            uint8_t op = _CPU_STACK_OP(state.pipeline);

            if (op & 0x08) // if pull op
            {
                switch(op + state.pipe_state * 0x10)
                {
                    case CPU_STACK_OP_PULL_INT:
                    case CPU_STACK_OP_PULL_PC_INC:
                    case CPU_STACK_OP_PULL_PC:
                    case CPU_STACK_OP_PULL_P:
                    case CPU_STACK_OP_PULL_Y:
                    case CPU_STACK_OP_PULL_X:
                    case CPU_STACK_OP_PULL_A:
                        state.rw_mode = CPU_RW_MODE_READ;
                        state.address = ((uint8_t)++state.S) + 0x0100;
                        state.pipe_state = 1;
                        return state;
                    case CPU_STACK_OP_PULL_INT + 0x10:
                        _CPU_SET_REG_P(state, state.data);
                        state.rw_mode = CPU_RW_MODE_READ;
                        state.address = ((uint8_t)++state.S) + 0x0100;
                        state.pipe_state = 2;
                        return state;
                    case CPU_STACK_OP_PULL_INT + 0x20:
                        state.PC = state.data;
                        state.rw_mode = CPU_RW_MODE_READ;
                        state.address = ((uint8_t)++state.S) + 0x0100;
                        state.pipe_state = 3;
                        return state;
                    case CPU_STACK_OP_PULL_INT + 0x30:
                        state.PC = (state.data << 8) | state.PC;
                        state.in_nmi = 0;
                        break;
                    case CPU_STACK_OP_PULL_PC_INC + 0x10:
                    case CPU_STACK_OP_PULL_PC + 0x10:
                        state.PC = state.data;
                        state.rw_mode = CPU_RW_MODE_READ;
                        state.address = ((uint8_t)++state.S) + 0x0100;
                        state.pipe_state = 2;
                        return state;
                    case CPU_STACK_OP_PULL_PC + 0x20:
                        state.PC = (state.data << 8) | state.PC;
                        break;
                    case CPU_STACK_OP_PULL_PC_INC + 0x20:
                        state.PC = ((state.data << 8) | state.PC) + 1;
                        break;
                    case CPU_STACK_OP_PULL_P + 0x10:
                        _CPU_SET_REG_P(state, state.data & 0xEF);
                        break;
                    case CPU_STACK_OP_PULL_Y + 0x10:
                        _CPU_SET_REG_Y(state, state.data);
                        break;
                    case CPU_STACK_OP_PULL_X + 0x10:
                        _CPU_SET_REG_X(state, state.data);
                        break;
                    case CPU_STACK_OP_PULL_A + 0x10:
                        _CPU_SET_REG_A(state, state.data);
                        break;
                }
                state.pipe_state = 0;
                state.pipeline = _CPU_SET_STACK_OP(state.pipeline, 0);
            }
            else
            {
                state.address = ((uint8_t)state.S--) + 0x0100;
                state.rw_mode = CPU_RW_MODE_WRITE;
                switch(op + state.pipe_state * 0x10)
                {
                    case CPU_STACK_OP_PUSH_A: state.data = state.A; break;
                    case CPU_STACK_OP_PUSH_X: state.data = state.X; break;
                    case CPU_STACK_OP_PUSH_Y: state.data = state.Y; break;
                    case CPU_STACK_OP_PUSH_P: state.data = state.P | CPU_STATUS_FLAG_BREAK; break;
                    case CPU_STACK_OP_PUSH_PC_INC:
                        state.PC += 1;
                    case CPU_STACK_OP_PUSH_PC: 
                        state.data = state.PC >> 8;
                        state.pipe_state = 1;
                        return state;
                    case CPU_STACK_OP_PUSH_PC_INC + 0x10:
                    case CPU_STACK_OP_PUSH_PC + 0x10: 
                        state.data = state.PC;
                        break;
                    case CPU_STACK_OP_PUSH_INT: 
                        state.data = state.PC >> 8;
                        state.pipe_state = 1;
                        return state;
                    case CPU_STACK_OP_PUSH_INT + 0x10: 
                        state.data = state.PC;
                        state.pipe_state = 2;
                        return state;
                    case CPU_STACK_OP_PUSH_INT + 0x20: 
                        state.data = state.P;
                        break;
                };
                state.pipeline = _CPU_SET_STACK_OP(state.pipeline, 0);
                state.pipe_state = 0;
                return state;
            }
        }

        // load address high
        if (load_addr)
        {
            if (load_addr & CPU_LOAD_ADDRESS_OP_HIGH)
            {
                if (state.pipe_state == 0)
                {
                    state.pipe_state = 1;
                    state.rw_mode = CPU_RW_MODE_READ;
                    state.address = state.PC++;
                    return state;
                }
                else
                {
                    state.address = (state.data << 8) | state.temp;
                    if (load_addr_xy == CPU_LOAD_ADDRESS_OP_X)
                        state.address += state.X;
                    if (load_addr_xy == CPU_LOAD_ADDRESS_OP_Y)
                        state.address += state.Y;
                    state.pipe_state = 0;
                    state.pipeline = _CPU_SET_LOAD_ADDR_OP(state.pipeline, load_addr_xy);
                }
            }
        }
      
        // load effective address
        if (_CPU_LEA_OP(state.pipeline))
        {
            uint8_t op = _CPU_LEA_OP(state.pipeline);
            state.rw_mode = CPU_RW_MODE_READ;
            switch(state.pipe_state)
            {
                case 0:
                    if (op == CPU_LEA_OP_VECTOR)
                    {
                        if (state.nmi)
                        {
                            state.nmi = 0;
                            state.in_nmi = 1;
                            state.address = 0xFFFA;
                        }
                        else if (state.irq || (state.P & CPU_STATUS_FLAG_BREAK))
                        {
                            _CPU_SET_REG_P(state, state.P | 0x4);
                            state.address = 0xFFFE;
                        }
                        else
                        {
                            state.address = 0xFFFC;
                        }
                    }
                    state.pipe_state = 1;
                    return state;

                case 1:
                    if (op == CPU_LEA_OP_VECTOR)
                    {
                        state.address = state.address + 1;
                    }
                    else
                    {
                        state.address = (state.address & 0xFF00) | ((state.address + 1) & 0x00FF);
                    }
                    state.temp = state.data;
                    state.pipe_state = 2;
                    return state;
                default:
                    break;
            }

            state.address = (state.data << 8) | state.temp;
            if (op == CPU_LEA_OP_INDIRECT_Y)
                state.address += state.Y;
            state.pipeline = _CPU_SET_LEA_OP(state.pipeline, 0);
            state.pipe_state = 0;
        }

        // load data (from address, PC or register)
        if (_CPU_LOAD_OP(state.pipeline))
        {
            switch(_CPU_LOAD_OP(state.pipeline))
            {
                case CPU_LOAD_OP_FETCH: 
                    state.rw_mode = CPU_RW_MODE_READ;
                    state.address = state.PC++;
                    state.pipeline = _CPU_SET_LOAD_OP(state.pipeline, 0);
                    return state;
                case CPU_LOAD_OP_MEM: 
                    state.rw_mode = CPU_RW_MODE_READ; 
                    state.pipeline = _CPU_SET_LOAD_OP(state.pipeline, 0);
                    return state;
                case CPU_LOAD_OP_A: state.data = state.A; 
                    state.pipeline = _CPU_SET_LOAD_OP(state.pipeline, 0);
                    break;
                case CPU_LOAD_OP_X: state.data = state.X; 
                    state.pipeline = _CPU_SET_LOAD_OP(state.pipeline, 0);
                    break;
                case CPU_LOAD_OP_Y: state.data = state.Y; 
                    state.pipeline = _CPU_SET_LOAD_OP(state.pipeline, 0);
                    break;
                case CPU_LOAD_OP_AX: state.data = state.A & state.X; 
                    state.pipeline = _CPU_SET_LOAD_OP(state.pipeline, 0);
                    break;
            }
        }
    
        // perform ALU operation with data
        if (_CPU_ALU_OP(state.pipeline))
        {
            switch(_CPU_ALU_OP(state.pipeline))
            {
                case CPU_ALU_OP_AND:  state.data = state.A & state.data; break;
                case CPU_ALU_OP_OR:   state.data = state.A | state.data; break;
                case CPU_ALU_OP_XOR:  state.data = state.A ^ state.data; break;
                case CPU_ALU_OP_ADD:
                {
                    uint16_t tmp = ((uint16_t)state.A) + state.data + (state.P & 1);
                    uint8_t overflow = (((state.data ^ tmp) & (state.A ^ tmp)) & 0x80) >> 1;
                    _CPU_SET_REG_P(state, (state.P & 0xBE) | ((tmp >> 8) & 1) | overflow);
                    state.data = tmp & 0xFF;
                }
                break;
                case CPU_ALU_OP_SUB:
                {
                    uint16_t tmp = ((uint16_t)state.A) + ~state.data + (state.P & 1);
                    uint8_t overflow = (((~state.data ^ tmp) & (state.A ^ tmp)) & 0x80) >> 1;
                    _CPU_SET_REG_P(state, (state.P & 0xBE) | ((~tmp >> 8) & 1) | overflow);
                    state.data = tmp & 0xFF;
                }
                break;
                case CPU_ALU_OP_CMP:    _CPU_CMP(state, state.A); break;
                case CPU_ALU_OP_CMP_X:  _CPU_CMP(state, state.X); break;
                case CPU_ALU_OP_CMP_Y:  _CPU_CMP(state, state.Y); break;
                case CPU_ALU_OP_DEC:    --state.data; _CPU_UPDATE_NZ(state, state.data); break;
                case CPU_ALU_OP_INC:    ++state.data; _CPU_UPDATE_NZ(state, state.data); break;
                case CPU_ALU_OP_ASL:
                {
                    _CPU_SET_REG_P(state, (state.P & 0xFE) | (state.data >> 7 ));
                    state.data = (state.data << 1);
                    _CPU_UPDATE_NZ(state, state.data);
                }
                break;
                case CPU_ALU_OP_LSR:
                {
                    _CPU_SET_REG_P(state, (state.P & 0xFE) | (state.data & 1));
                    state.data = (state.data >> 1);
                    _CPU_SET_REG_P(state, (state.P & 0x7D) | (state.data?0:0x02));
                }
                break;
                case CPU_ALU_OP_ROL:
                {
                    uint8_t tmp = state.P & 1;
                    _CPU_SET_REG_P(state, (state.P & 0xFE) | (state.data >> 7));
                    state.data = (state.data << 1) | tmp;
                    _CPU_UPDATE_NZ(state, state.data);
                }
                break;
                case CPU_ALU_OP_ROR:
                {
                    uint8_t tmp = state.P & 1;
                    _CPU_SET_REG_P(state, (state.P & 0xFE) | (state.data & 1));
                    state.data = (state.data >> 1) | (tmp << 7);
                    _CPU_UPDATE_NZ(state, state.data);
                }
                break;
                case CPU_ALU_OP_BIT:
                _CPU_SET_REG_P(state, (state.P & 0x3D) | (state.data & 0xC0) | (((state.A & state.data) == 0)?2:0));
                break;
                case CPU_ALU_OP_NONE: break;
                default: break;
            }
            state.pipeline = _CPU_SET_ALU_OP(state.pipeline, 0);
        }

        // write data to some destination
        if (_CPU_WRITE_OP(state.pipeline))
        {
            switch(_CPU_WRITE_OP(state.pipeline))
            {
                case CPU_WRITE_OP_MEM:  
                    state.rw_mode = CPU_RW_MODE_WRITE; 
                    state.pipeline = _CPU_SET_WRITE_OP(state.pipeline, 0); 
                    return state;
                case CPU_WRITE_OP_A:    _CPU_SET_REG_A(state, state.data); break;
                case CPU_WRITE_OP_X:    _CPU_SET_REG_X(state, state.data); break;
                case CPU_WRITE_OP_Y:    _CPU_SET_REG_Y(state, state.data); break;
                case CPU_WRITE_OP_AX:   _CPU_SET_REG_A(state, state.data); state.X = state.A; break;
                case CPU_WRITE_OP_PC:   state.PC = state.address; break;
                case CPU_WRITE_OP_BRANCH:  
                {
                    uint16_t newPC = state.PC + (int8_t)state.data;
                    state.wait_op = ((newPC & 0xFF00) != (state.PC & 0xFF00)) ? 2 : 1;
                    state.PC = newPC;
                }
                break;
            }
            state.pipeline = _CPU_SET_WRITE_OP(state.pipeline, 0);
        }

        if (state.wait_op)
        {
            state.wait_op--;
            state.rw_mode = CPU_RW_MODE_NONE;
            return state;
        }
    }
    //

    _CPU_END(state);

    state.rw_mode = CPU_RW_MODE_READ;
    state.address = state.PC;
    return state;
}

#endif
