BITS 16

align 256
global small_mov_loop
small_mov_loop:
    mov bx, 0xfff0
    mov dx, 0xdead
    jmp .inner_loop
align 16
.inner_loop:
    mov di, 0xfff2
    mov si, 0xfff2
    rcl ax, 31
    rcl ax, 31
    rcl ax, 31
    rcl ax, 31

    stosb
    stosb
    stosb
    stosb
    lodsb
    lodsb
    lodsb
    lodsb

    mov [bx], ax
    mov [bx], ax
    mov [bx], ax
    mov [bx], ax

    mov [bx], ax
    mov [bx], ax
    mov [bx], ax
    mov [bx], ax

    mov [bx], ax
    mov [bx], ax
    mov [bx], ax
    mov [bx], ax



    out dx, al

    jmp .inner_loop

align 256
global odd_mov_loop
odd_mov_loop:
    mov bx, 0xfff0
    mov dx, 0xdead
    mov cl, 31
    jmp .inner_loop
align 16
.inner_loop:
    ; fill prefetch
    rcl ax, cl
    rcl ax, cl
    rcl ax, cl
    rcl ax, cl

    mov [bx+2], ax
    mov [bx+4], ax
    mov [bx+6], ax
    mov [bx+8], ax

    mov [bx+2], ax
    mov [bx+4], ax
    mov [bx+6], ax
    mov [bx+8], ax

    mov [bx+2], ax
    mov [bx+4], ax
    mov [bx+6], ax
    mov [bx+8], ax

    mov [bx+2], ax
    mov [bx+4], ax
    mov [bx+6], ax
    mov [bx+8], ax

    mov [bx+2], ax
    mov [bx+4], ax
    mov [bx+6], ax
    mov [bx+8], ax

    out dx, al

    jmp .inner_loop

align 256
global nop_loop
nop_loop:
    jmp .inner_loop
align 16
.inner_loop:

%rep 32
    nop
%endrep

    out dx, al

    jmp .inner_loop

align 256
global push_all_loop
push_all_loop:
    mov cl, 31
    jmp .inner_loop
align 16
.inner_loop:
    ; fill prefetch
    rcl ax, cl
    rcl ax, cl

    pusha
    popa
    
    nop
    rcl ax, cl

    pusha
    popa

    nop
    rcl ax, cl
    push ax
    nop
    rcl ax, cl
    pop ax

    out dx, al

    jmp .inner_loop


align 256
global prefetch_loop
prefetch_loop:
    mov bx, 0xfff0
    mov dx, 0xdead
    mov cl, 31
    jmp .pre_nop
align 16

align 2
.pre_nop:
    rcl ax, cl
    nop
    nop
    jmp .pre_mov2

align 2
.pre_mov2:
    rcl ax, cl
    mov bx, bx
    jmp .pre_mov3

align 2
.pre_mov3:
    rcl ax, cl
    mov byte [bx], 0xee
    jmp .pre_mov4

align 2
.pre_mov4:
    rcl ax, cl
    mov word [bx], 0xeeee
    jmp .pre_mov5

align 2
.pre_mov5:
    rcl ax, cl
    mov word [bx+2], 0xeeee
    jmp .pre_mov6

align 2
.pre_mov6:
    rcl ax, cl
    mov word [bx+2000], 0xeeee
    jmp .pre_mov3_seg

align 2
.pre_mov3_seg:
    rcl ax, cl
    mov byte es:[bx], 0xee
    jmp .finish

align 2
.finish:
    out dx, al

    jmp .pre_nop


align 256
global prefetch_loop_mem
prefetch_loop_mem:
    mov bx, 0xfff0
    mov di, bx
    mov dx, 0xdead
    mov cl, 31
    jmp .pre_nop
align 16

align 2
.pre_nop:
    rcl word [di], cl
    nop
    nop
    jmp .pre_mov2

align 2
.pre_mov2:
    rcl word [di], cl
    mov bx, bx
    jmp .pre_mov3

align 2
.pre_mov3:
    rcl word [di], cl
    mov byte [bx], 0xee
    jmp .pre_mov4

align 2
.pre_mov4:
    rcl word [di], cl
    mov word [bx], 0xeeee
    jmp .pre_mov5

align 2
.pre_mov5:
    rcl word [di], cl
    mov word [bx+2], 0xeeee
    jmp .pre_mov6

align 2
.pre_mov6:
    rcl word [di], cl
    mov word [bx+2000], 0xeeee
    jmp .pre_mov3_seg

align 2
.pre_mov3_seg:
    rcl word [di], cl
    mov byte es:[bx], 0xee
    jmp .finish

align 2
.finish:
    out dx, al

    jmp .pre_nop


align 256
global nop_fetching
nop_fetching:
    mov bx, 0xfff0
    mov dx, 0xdead
    jmp .start
.start:
align 8
    ror    ax,0x1f
    mov    cx,[bx]
    nop
    mov    ax,[bx]
    sub    ax,cx

    out dx, al

    jmp .start



%macro time_op 2
.time_op_%1:
align 2
    ;nop ; 1
    ror ax, 11 ; 3
    mov dx, [bx] ; 2
    %2
    mov ax, [bx]
    sub ax, dx
    mov dx, 0xdeee
    out dx, ax
%endmacro

%macro time_op2 3
.time_op2_%1:
align 2
    ;nop ; 1
    ror ax, 11 ; 3
    mov dx, [bx] ; 2
    %2
    %3
    mov ax, [bx]
    sub ax, dx
    mov dx, 0xdeee
    out dx, ax
%endmacro

%macro time_basic_lock 2
.%1:
align 2
    ror ax, 11
    lock mov dx, [bx]
    %2
    lock mov ax, [bx]
    sub ax, dx
    mov dx, 0xdeee
    out dx, ax
%endmacro

%macro with_jmp 2
    ror ax, 11
    mov dx, [bx]
    jmp .%1
align 2
.%1:
    %2
    mov ax, [bx]
    sub ax, dx
    mov dx, 0xdeee
    out dx, ax

%endmacro

align 256
global time_basic_ops
time_basic_ops:
    enter 1024, 0
    mov ax, ss
    mov es, ax
    mov si, sp
    mov di, sp

    mov ax, 0xb000
    mov ds, ax
    xor bx, bx

.start:
    ;time_basic_lock lock_just_nop, nop
    ;time_basic_lock lock_dec_ax, { dec ax }
    ;time_basic_lock lock_dec_al, { dec al }
    ;time_basic_lock lock_mov_reg_mem, { mov ax, [di] }
    ;time_basic_lock lock_mov_mem_reg, { mov [di], ax }
    ;time_basic_lock lock_dec_mem2, { dec word [di] }
    ;time_basic_lock lock_dec_mem3, { dec word [di+4] }

    ;time_op just_nop, nop
    ;time_op dec_ax, { dec ax }
    ;time_op dec_al, { dec al }

    time_op big, { rep lock mov word cs:[bx+2000], 0xeeee }

    time_op push_1, { push ax }
    time_op2 push_2, { push ax }, { push ax }
    time_op pop_1, { pop ax }
    time_op2 pop_2, { pop ax }, { pop ax }

    time_op trans_1, { xlat }
    time_op2 trans_2, { xlat }, { xlat }


    time_op mov_reg_mem, { mov ax, [di] }
    time_op mov_mem_reg, { mov [di], ax }
    time_op dec_mem2, { dec word [di] }
    time_op dec_mem3, { dec word [di+4] }
    ;time_basic dec_mem4, { dec word [di+256] }
    ;time_basic dec_mem5, { dec word ss:[di+256] }

    ;time_basic just_nop_lock, { lock nop }
    ;time_basic dec_ax_lock, { lock dec ax }
    ;time_basic dec_al_lock, { lock dec al }
    ;time_basic mov_reg_mem_lock, { lock mov ax, [di] }
    ;time_basic mov_mem_reg_lock, { lock mov [di], ax }
    ;time_basic dec_mem2_lock, { lock dec word [di] }
    ;time_basic dec_mem3_lock, { lock dec word [di+4] }

    ;time_basic dec_mem4_lock, { lock dec word [di+256] }
    ;time_basic dec_mem5_lock, { lock dec word ss:[di+256] }


    ;with_jmp just_nop_lock_jmp, { lock nop }
    ;with_jmp dec_ax_lock_jmp, { lock dec ax }
    ;with_jmp dec_al_lock_jmp, { lock dec al }
    ;with_jmp dec_mem2_lock_jmp, { lock dec word [di] }
    ;with_jmp dec_mem3_lock_jmp, { lock dec word [di+4] }
    ;with_jmp dec_mem4_lock_jmp, { lock dec word [di+256] }
    ;with_jmp dec_mem5_lock_jmp, { lock dec word ss:[di+256] }


    mov dx, 0xdead
    out dx, ax
    jmp .start
