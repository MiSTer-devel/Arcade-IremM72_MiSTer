BITS 16

fill_regs:
    mov ax, 0x5555
    mov bx, 0xdddd
    mov cx, 0x1111
    mov dx, 0x9008
    mov si, 0xc5c5
    mov di, 0x6666
    ret

%macro exercise_arith 1
    call fill_regs
    %1 ax, cx
    %1 cx, cx
    %1 si, di
    %1 di, di
    %1 dx, di
    %1 ax, ax
    %1 dl, al
    %1 cl, dh
    %1 ah, ah
%endmacro

%macro exercise_shift 1
    call fill_regs
    %1 ax, 3
    %1 cx, 5
    %1 si, 8
    %1 di, 11
    %1 dx, 4
    %1 ax, 2
    %1 ax, 1
    %1 si, 1
    %1 di, 1
    %1 dx, 1
    %1 ax, 1
    %1 al, 1
    %1 ch, 1
    %1 bl, 1
    %1 dh, 1 
    mov cl, 3
    %1 ax, cl
    %1 al, cl

%endmacro

exercise_stack:
    call fill_regs
    pusha
    push ax
    push dx
    popa
    pop cx
    pop bx

    enter 16, 0
    leave

    enter 16, 1
    leave

    enter 16, 2
    leave

    enter 16, 4
    leave

    ret


exercise_mem:
    enter 32, 0
    mov word [bp - 2], 0x8811
    mov byte [bp - 3], 0x22
    mov byte [bp - 4], 0x33
    mov byte [bp - 5], 0x44
    mov word [bp - 7], 0x5566

    mov ax, [bp - 2]
    mov ax, [bp - 4]
    mov al, [bp - 2]
    mov al, [bp - 1]
    mov al, [bp - 3]
    mov al, [bp - 4]
    mov ax, [bp - 3]
    mov ax, [bp - 5]
    mov al, [bp - 6]
    mov al, [bp - 7]
    mov ax, [bp - 7]
    leave
    ret

exercise_mulu:
    call fill_regs
    mul cl
    mul dl
    mul si
    mul di
    ret

exercise_mul:
    call fill_regs
    imul cx
    imul cx, dx, 6
    imul ax, di, -9
    imul di, 42
    imul bx, 1023
    ret

exercise_divu:
    mov ax, 0x1000
.divloop1:
    xor dx, dx
    mov cx, 0x9
    div cx
    cmp ax, 0
    jne .divloop1

    ; trigger divide-by-zero exception
    mov ax, 0xf000
    xor dx, dx
    div dx

    ; trigger overflow
    mov dx, 0x7fff
    mov cx, 2
    div cx

    ret 

exercise_div:
    mov ax, 0xf800
.divloop1:
    cwd
    mov cx, 0x3
    idiv cx
    cmp ax, 0
    jne .divloop1

    ret 


exercise_string:
    cld
    mov di, 0x1000
    mov ax, 0xf001
    mov cx, 0x10
    rep stosw

    mov si, 0x1000
    xor ax, ax
    mov cx, 0x10
    rep lodsw
    ret

exercise_dec:
    mov ax, 0x20
.loop1:
    dec ax
    jne .loop1

.loop2:
    dec al
    jne .loop2

    ret

exercise_cvtbd:
    mov cx, 0x100
.loop:
    xor ax, ax
    mov al, cl
    aam
    loop .loop
    ret

exercise_cvtdb:
    mov ax, 0x0000
    aad
    mov ax, 0x0101
    aad
    mov ax, 0x0401
    aad
    mov ax, 0x0903
    aad
    mov ax, 0x1003
    aad
    mov ax, 0x0320
    aad
    mov ax, 0xffff
    aad
    ret

bcd1:
    db 0x40, 0x11, 0x99, 0x75, 0x43, 0x60, 0x07, 0x83
    db 0x75, 0x99, 0x99, 0x25, 0x39, 0x84, 0x73, 0x02

exercise_add4s:
    pusha

    mov dx, cs
    mov ds, dx
    mov dx, ss
    mov es, dx
    mov si, bcd1
    mov di, 0x8000
    mov cx, 16
    rep movsb

    mov ds, dx
    mov cx, 16
    mov si, 0x8000
    mov di, 0x8008
    db 0x0f, 0x20 ; add4s

    mov cx, 8
    mov si, 0x8000
    mov di, 0x8000
    db 0x0f, 0x22 ; sub4s

    mov cx, 0x10
    mov si, 0x8000
.load_loop:
    mov al, [si]
    inc si
    loop .load_loop

    popa
    ret

exercise_ror4:
    mov al, 0x01
    mov cl, 0x23
    db 0x0f, 0x28, 0xc1; ROL4
    db 0x0f, 0x28, 0xc1; ROL4
    db 0x0f, 0x2a, 0xc1; ROR4
    db 0x0f, 0x2a, 0xc1; ROR4
    db 0x0f, 0x2a, 0xc1; ROR4
    ret

global exercise_ops
exercise_ops:
    call exercise_ror4
    call exercise_add4s
    call exercise_div
    call exercise_divu
    call exercise_mulu
    call exercise_mul

    call exercise_cvtdb
    call exercise_cvtbd

    exercise_arith add
    exercise_arith sub
    exercise_arith sbb
    exercise_arith adc
    exercise_arith or
    exercise_arith and
    exercise_arith xor
    exercise_arith cmp
    exercise_arith test
    exercise_shift rol
    exercise_shift ror
    exercise_shift rcl
    exercise_shift rcr
    exercise_shift shl
    exercise_shift sal
    exercise_shift shr

    exercise_arith xchg

    call exercise_stack

    call exercise_string

    call exercise_mem

    call exercise_dec


    mov dx, 0xdead
    mov al, 0xff
    out dx, al
    ret

global vram_timing
vram_timing:
    push es
    push ds
    push di
    push si

    mov ax, 0xd000
    mov es, ax
    mov di, 0xf000
    mov cx, 0x20
    rep stosw

    mov ax, 0xd000
    mov ds, ax
    mov si, 0xf000
    mov cx, 0x20
    rep lodsw

    pop si
    pop di
    pop ds
    pop es
    ret