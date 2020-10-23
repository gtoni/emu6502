DISPLAY_ARRAY = $4000

    .org $8000
reset:
    brk
    jmp reset

irq:
nmi:
    lda #0
    pha
; test

    lda #%11
    ldx #10
    sta 300, x
    
    lda #3
    cmp 300, x 

    lda #$ff
    sta 300
    inc 300
    beq success ;not equal - zero flag is not set 
; end test
    pla
    lda #8
    pha
success:
    pla
loop:
    pha
    jsr display_message
    pla
    jmp loop

display_message:
    tax
    ldy #0
print_char:
    lda message, x
    beq end_display_message
    sta DISPLAY_ARRAY, y
    iny
    inx
    jmp print_char
end_display_message:
    lda #" "
    sta DISPLAY_ARRAY, y
    rts

message:     .asciiz "SUCCESS"
             .asciiz "FAIL   "

    .org $fffa
    .word nmi
    .word reset
    .word irq
