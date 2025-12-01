
`ifndef DEFINES
`define DEFINES

    `define SEXT(VAL, W_SRC, W_DEST, SHIFT) \
        {{((W_DEST) - (W_SRC) - (SHIFT)){VAL[(W_SRC)-1]}}, VAL, {(SHIFT){1'b0}}}
    `define SUM   0
    `define CARRY 1
    `define ABS(VAL) (((VAL) > 0) ? (VAL) : -(VAL))

`endif
