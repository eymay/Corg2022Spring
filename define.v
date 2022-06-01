`define  OUT_D_ARF_PC \
    ARF_OutDSel = 2'b00;
`define  OUT_C_ARF_PC \
    ARF_OutCSel = 2'b00;
`define  OUT_D_ARF_AR \
    ARF_OutDSel = 2'b10;
`define  OUT_C_ARF_AR \
    ARF_OutCSel = 2'b10;
`define  OUT_D_ARF_SP \
    ARF_OutDSel = 2'b11;
`define  OUT_C_ARF_SP \
    ARF_OutCSel = 2'b11;


`define IN_ARF_PC \
    ARF_RegSel = 3'b011; \
    ARF_FunSel = 2'b10; 

`define OUT_A_REG1 \
    RF_OutASel = 2'b00;
`define OUT_A_REG2 \
    RF_OutASel = 2'b01;
`define OUT_A_REG3 \
    RF_OutASel = 2'b10;
`define OUT_A_REG4 \
    RF_OutASel = 2'b11;
`define OUT_B_REG1 \
    RF_OutBSel = 2'b00;
`define OUT_B_REG2 \
    RF_OutBSel = 2'b01;
`define OUT_B_REG3 \
    RF_OutBSel = 2'b10;
`define OUT_B_REG4 \
    RF_OutBSel = 2'b11;



    
`define IN_ARF_AR \
    ARF_RegSel = 3'b101; \
    ARF_FunSel = 2'b10; //load 
`define IN_ARF_SP \
    ARF_RegSel = 3'b110; \
    ARF_FunSel = 2'b10; //load 

`define IN_REG1 \
    RF_RegSel = 4'b0111; \
    RF_FunSel = 2'b10; //load 
    
`define IN_REG2 \
    RF_RegSel = 4'b1011;\
    RF_FunSel = 2'b10; //load 
    
`define IN_REG3 \
    RF_RegSel = 4'b1101; \
    RF_FunSel = 2'b10; //load  
`define IN_REG4 \
    RF_RegSel = 4'b1110;\
    RF_FunSel = 2'b10; //load 

`define IN_IR(LH) \
    IR_Enable = 1; \
    IR_Funsel = 2'b10; \
    IR_LH = LH;

`define WRITE_RX(Regsel) \
    case("`RegSel") \
        2'b00: RF_RegSel = 4'b0111; \
        2'b01: RF_RegSel = 4'b1011; \
        2'b10: RF_RegSel = 4'b1101; \
        2'b11: RF_RegSel = 4'b1110; \
        endcase \

`define INC_ARF_PC \
    ARF_RegSel = 3'b011;\
    ARF_FunSel = 2'b01; //increment 
`define INC_ARF_AR \
    ARF_RegSel = 3'b101;\
    ARF_FunSel = 2'b01; //increment 
`define INC_ARF_SP \
    ARF_RegSel = 3'b110;\
    ARF_FunSel = 2'b01; //increment 

`define INC_REG1 \
    RF_RegSel = 4'b0111;\
    RF_FunSel = 2'b01; //increment 
`define INC_REG2 \
    RF_RegSel = 4'b1011;\
    RF_FunSel = 2'b01; //increment 
`define INC_REG3 \
    RF_RegSel = 4'b1101;\
    RF_FunSel = 2'b01; //increment 
`define INC_REG4 \
    RF_RegSel = 4'b1110;\
    RF_FunSel = 2'b01; //increment 

`define INC_IR \
    IR_Enable = 1; \
    IR_Funsel = 2'b01; \ //increment

`define DEC_ARF_PC \
    ARF_RegSel = 3'b011;\
    ARF_FunSel = 2'b00; //decrement 
`define DEC_ARF_AR \
    ARF_RegSel = 3'b101;\
    ARF_FunSel = 2'b00; //decrement 
`define DEC_ARF_SP \
    ARF_RegSel = 3'b110;\
    ARF_FunSel = 2'b00; //decrement 

`define DEC_REG1 \
    RF_RegSel = 4'b0111;\
    RF_FunSel = 2'b00; //decrement 
`define DEC_REG2 \
    RF_RegSel = 4'b1011;\
    RF_FunSel = 2'b00; //decrement 
`define DEC_REG3 \
    RF_RegSel = 4'b1101;\
    RF_FunSel = 2'b00; //decrement 
`define DEC_REG4 \
    RF_RegSel = 4'b1110;\
    RF_FunSel = 2'b00; //decrement 

`define DEC_IR \
    IR_Enable = 1; \
    IR_Funsel = 2'b00; \ //decrement

`define OUT_MEM\
    Mem_CS = 0; \
    Mem_WR = 0;

`define IN_MEM\
    Mem_CS = 0; \
    Mem_WR = 1;

`define MUX_A_IROUT\
    MuxASel = 2'b00;
`define MUX_A_MEMOUT\
    MuxASel = 2'b01;
`define MUX_A_ARF_OUTC\
    MuxASel = 2'b10;
`define MUX_A_ARF_OUTALU\
    MuxASel = 2'b11;

`define MUX_B_IROUT\
    MuxBSel = 2'b01;
`define MUX_B_MEMOUT\
    MuxBSel = 2'b10;
`define MUX_B_OUTALU\
    MuxBSel = 2'b11;
     
`define MUX_C_ARF\
    MuxCSel = 0;
`define MUX_C_REG_OUTA\
    MuxCSel = 1;

`define CLR_ARF\
    ARF_RegSel = 3'b000;\
    ARF_FunSel = 2'b11; //clear

`define CLR_RF\
    RF_RegSel = 4'b0000;\
    RF_FunSel = 2'b11; //clear