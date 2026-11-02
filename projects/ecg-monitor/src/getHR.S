
@ uint8_t getHR(short* dataArray)
@ Computes BPM from 3s ECG buffer using peak detection

@ Register map:
@   r0  = dataArray pointer (argument + return value)
@   r1  = loop index
@   r2  = current sample / scratch
@   r3  = maxVal / scratch
@   r4  = threshold
@   r5  = peakCount
@   r6  = refractoryCounter
@   r7  = zeroCount
@   r8  = loop end value (1079)
@   r9  = saved base pointer
@   r10 = BUFFER_SIZE (1080)
@   r11 = REFRACTORY (36)

    .syntax unified
    .thumb
    .global getHR
    .type   getHR, %function

    .equ BUFFER_SIZE,   1080
    .equ REFRACTORY,    36

getHR:
    PUSH    {r4-r11, lr}

    MOV     r9,  r0              @ save base pointer
    MOV     r10, #BUFFER_SIZE
    MOV     r11, #REFRACTORY

    
    MOV     r1, #0               @ index = 0
    MOV     r7, #0               @ zeroCount = 0

zero_loop:
    CMP     r1, r10
    BGE     zero_done
    LDRSH   r2, [r9, r1, LSL #1]
    CMP     r2, #0
    IT      EQ
    ADDEQ   r7, r7, #1
    ADD     r1, r1, #1
    B       zero_loop

zero_done:
    MOV     r3, #108
    CMP     r7, r3
    BLE     find_max
    MOV     r0, #0
    B       getHR_exit


find_max:
    MOV     r1, #0               @ index = 0
    MOV     r3, #0               @ maxVal = 0

max_loop:
    CMP     r1, r10
    BGE     max_done
    LDRSH   r2, [r9, r1, LSL #1]
    CMP     r2, r3
    IT      GT
    MOVGT   r3, r2               @ update maxVal
    ADD     r1, r1, #1
    B       max_loop

max_done:
    @ threshold = maxVal * 3 / 5 (~60%)
    MOV     r4, r3
    MOV     r2, #3
    MUL     r4, r4, r2
    MOV     r2, #5
    SDIV    r4, r4, r2

    
    MOV     r1, #1               @ i = 1
    MOV     r5, #0               @ peakCount = 0
    MOV     r6, #0               @ refractoryCounter = 0
    MOV     r8, #1079            @ loop end = BUFFER_SIZE - 1

peak_loop:
    CMP     r1, r8
    BGE     peak_done

    @ refractory check
    CMP     r6, #0
    BLE     check_peak
    SUB     r6, r6, #1
    ADD     r1, r1, #1
    B       peak_loop

check_peak:
    @ load dataArray[i]
    LDRSH   r2, [r9, r1, LSL #1]

    @ if dataArray[i] <= threshold skip
    CMP     r2, r4
    BLE     next_sample

    @ check dataArray[i] >= dataArray[i-1]
    SUB     r3, r1, #1
    LDRSH   r3, [r9, r3, LSL #1]
    CMP     r2, r3
    BLT     next_sample

    @ check dataArray[i] >= dataArray[i+1]
    ADD     r3, r1, #1
    LDRSH   r3, [r9, r3, LSL #1]
    CMP     r2, r3
    BLT     next_sample

    @ R-peak found
    ADD     r5, r5, #1           @ peakCount++
    MOV     r6, r11              @ reset refractory = 36

next_sample:
    ADD     r1, r1, #1
    B       peak_loop

peak_done:

    MOV     r2, #20
    MUL     r0, r5, r2


    CMP     r0, #40
    BLT     invalid_hr
    CMP     r0, #200
    BGT     invalid_hr
    B       getHR_exit

invalid_hr:
    MOV     r0, #0

getHR_exit:
    UXTB    r0, r0               @ truncate to uint8_t
    POP     {r4-r11, pc}

    .size getHR, .-getHR