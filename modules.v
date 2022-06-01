
//Part 1 


module n_bitRegister #(parameter N = 8) (
    input CLK, E, [1:0] FunSel, [N-1:0] I,
    output [N-1:0] Q
);

    reg [N-1:0] Q_temp;
    assign Q = Q_temp;
    always @( posedge CLK ) begin
    if(E) begin
        case (FunSel)
        0: begin
            Q_temp = Q - 1;
        end
        1: begin
            Q_temp = Q + 1;
        end
        2: begin
            Q_temp = I;
        end
        3: begin
            Q_temp =0;
        end
        default: begin
            Q_temp = Q_temp;
        end
        endcase
    end else begin
    
        Q_temp = Q;
    end
    end
    
endmodule

//Part 2

module RegFile (
    input CLK, [1:0] OutASel, [1:0] OutBSel, [1:0] FunSel, [3:0] RegSel, [7:0] I, 
    output [7:0] OutA, [7:0] OutB
);
    wire [7:0] R1_Q;
    wire [7:0] R2_Q;
    wire [7:0] R3_Q;
    wire [7:0] R4_Q;
    n_bitRegister #(.N(8)) R1(.CLK(CLK),.E(~RegSel[3]), .FunSel(FunSel), .I(I), .Q(R1_Q));
    n_bitRegister #(.N(8)) R2(.CLK(CLK),.E(~RegSel[2]), .FunSel(FunSel), .I(I), .Q(R2_Q));
    n_bitRegister #(.N(8)) R3(.CLK(CLK),.E(~RegSel[1]), .FunSel(FunSel), .I(I), .Q(R3_Q));
    n_bitRegister #(.N(8)) R4(.CLK(CLK),.E(~RegSel[0]), .FunSel(FunSel), .I(I), .Q(R4_Q));

    //wire [3:0] R_En;

    reg [7:0] OutA_temp, OutB_temp;
    assign OutA = OutA_temp;
    assign OutB = OutB_temp;
    always@(*) begin
        case (OutASel)
        0: begin
            OutA_temp = R1_Q;
        end
        1: begin
            OutA_temp = R2_Q;
        end
        2: begin
            OutA_temp = R3_Q;
        end
        3: begin
            OutA_temp = R4_Q;
        end
        default: begin
            OutA_temp = OutA_temp;
        end
        endcase
    end
    always@(*) begin
        case (OutBSel)
        0: begin
            OutB_temp = R1_Q;
        end
        1: begin
            OutB_temp = R2_Q;
        end
        2: begin
            OutB_temp = R3_Q;
        end
        3: begin
            OutB_temp = R4_Q;
        end
        default: begin
            OutB_temp = OutB_temp;
        end
        endcase
    end

endmodule

module ARF (
    input CLK, [1:0] OutCSel, [1:0] OutDSel, [1:0] FunSel, [3:0] RegSel, [7:0] I,
    output [7:0] OutC, [7:0] OutD
);

    wire [7:0] PC_Q;
    wire [7:0] AR_Q;
    wire [7:0] SP_Q;
    
    n_bitRegister #(.N(8)) PC(.CLK(CLK),.E(~RegSel[2]), .FunSel(FunSel), .I(I), .Q(PC_Q));
    n_bitRegister #(.N(8)) AR(.CLK(CLK),.E(~RegSel[1]), .FunSel(FunSel), .I(I), .Q(AR_Q));
    n_bitRegister #(.N(8)) SP(.CLK(CLK),.E(~RegSel[0]), .FunSel(FunSel), .I(I), .Q(SP_Q));



    reg [7:0] OutC_temp, OutD_temp;
    assign OutC = OutC_temp;
    assign OutD = OutD_temp;

    always@(*) begin
        case (OutCSel)
        0: begin
            OutC_temp = PC_Q;
        end
        1: begin
            OutC_temp = PC_Q;
        end
        2: begin
            OutC_temp = AR_Q;
        end
        3: begin
            OutC_temp = SP_Q;
        end
        default: begin
            OutC_temp = OutC_temp;
        end
        endcase
    end

    always@(*) begin
        case (OutDSel)
        0: begin
            OutD_temp = PC_Q;
        end
        1: begin
            OutD_temp = PC_Q;
        end
        2: begin
            OutD_temp = AR_Q;
        end
        3: begin
            OutD_temp = SP_Q;
        end
        default: begin
            OutD_temp = OutD_temp;
        end
        endcase
     end    

    
endmodule

module IR (
    input CLK, LH, En, [1:0] FunSel, [7:0] I,
    output [15:0] IRout
);
    reg [15:0] I_temp; //reg
    //wire [15:0] IR_Q;
    
    n_bitRegister #(.N(16)) IR(.CLK(CLK),.E(En), .FunSel(FunSel), .I(I_temp), .Q(IRout));
    
    //assign IRout = LH ? I_temp[15:8] : I_temp[7:0];
    
    
    always @(*) begin
        case (LH)
            0: begin
                I_temp[15:8] = I;
            end
            1: begin
                I_temp[7:0] = I;
            end
        endcase
    end
    
endmodule

//Part 3
module ALU (
    input CLK, [3:0] FunSel, input [7:0] A, [7:0] B, 
    output [7:0] OutALU, reg [3:0] OutFlag = 4'b0
);


wire Cin;
reg [7:0] ALU_result;
assign Cin = OutFlag[2];
assign OutALU = ALU_result;
reg  enable_o;
    always @(*) begin 
    case (FunSel)
        4'b0000: begin
            ALU_result <= A;
            enable_o <= 0;
        end
        4'b0001: begin
            ALU_result <= B;
            enable_o <= 0;
        end
        4'b0010: begin
            ALU_result <= ~A;
            enable_o <= 0;
        end
        4'b0011: begin
            ALU_result <= ~B;
            enable_o <= 0;
        end
        4'b0100: begin
            ALU_result <= A + B;
            enable_o <= 1;
        end
        4'b0101: begin
            ALU_result <= A + B + Cin;
            enable_o <= 1;
        end
        4'b0110: begin
            ALU_result <= A - B;
            enable_o <= 1;
        end
        4'b0111: begin
            ALU_result <= A & B;
            enable_o <= 0;
        end
        4'b1000: begin
            ALU_result <= A | B;
            enable_o <= 0;
        end
        4'b1001: begin
            ALU_result <= A ^ B;
            enable_o <= 0;
        end
        4'b1010: begin
            OutFlag[2] <= A[7];
            ALU_result <= A << 1;
            enable_o <= 0;
        end
        4'b1011: begin
            OutFlag[2] <= A[0];
            ALU_result <= A >> 1;
            enable_o <= 0;
        end
        4'b1100: begin
            ALU_result <= A <<< 1;
            enable_o <= 1;
        end
        4'b1101: begin
            ALU_result <= A >>> 1;
            enable_o <= 0;
        end
        4'b1110: begin
            OutFlag[2] <= A[7];
            ALU_result[0] <= OutFlag[2];
            ALU_result[1] <= A[0];
            ALU_result[2] <= A[1];
            ALU_result[3] <= A[2];
            ALU_result[4] <= A[3];
            ALU_result[5] <= A[4];
            ALU_result[6] <= A[5];
            ALU_result[7] <= A[6];
            enable_o = 1;
            //ALU_result = {A[6:0], A[7]};
        end
        4'b1111: begin
            OutFlag[2] <= A[0];
            ALU_result[0] <= A[1];
            ALU_result[1] <= A[2];
            ALU_result[2] <= A[3];
            ALU_result[3] <= A[4];
            ALU_result[4] <= A[5];
            ALU_result[5] <= A[6];
            ALU_result[6] <= A[7];
            ALU_result[7] <= OutFlag[2];
            //ALU_result = { A[0], A[7:1]};

            enable_o = 1;
        end
    endcase
    end
    always @(negedge CLK) begin
        if(ALU_result == 0) begin
            OutFlag[3] = 1;
        end else begin
            OutFlag[3] = 0;
        end

        if(ALU_result[7] == 1) begin
            OutFlag[1] = 1;
        end else begin
            OutFlag[1] = 0;
        end
        
        if((A[7] == ~ALU_result[7]) && (enable_o))
            OutFlag[0] = 1;

        /*
        if(ALU_result > 8'b11111111) begin
            OutFlag[3] = 1;
        end else begin
            OutFlag[3] = 0;
        end
        */
   end

endmodule


//Part 4

module Memory(
    input wire[7:0] address,
    input wire[7:0] data,
    input wire wr, //Read = 0, Write = 1
    input wire cs, //Chip is enable when cs = 0
    input wire clock,
    output reg[7:0] o // Output
);
    //Declaration o?f the RAM Area
    reg[7:0] RAM_DATA[0:255];
    //Read Ram data from the file
    initial $readmemh("RAM.mem", RAM_DATA);
    //Read the selected data from RAM
    always @(*) begin
        o = ~wr && ~cs ? RAM_DATA[address] : 8'hZ;
    end
    
    //Write the data to RAM
    always @(posedge clock) begin
        if (wr && ~cs) begin
            RAM_DATA[address] <= data; 
        end
    end
endmodule
/*
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



`define IN_ARF_PC \
    ARF_RegSel = 3'b011; \ //enable 
    ARF_FunSel = 2'b10; //load 
`define IN_ARF_AR \
    ARF_RegSel = 3'b101; \ //enable 
    ARF_FunSel = 2'b10; //load 
`define IN_ARF_SP \
    ARF_RegSel = 3'b110; \ //enable 
    ARF_FunSel = 2'b10; //load 

`define IN_REG1 \
    RF_RegSel = 4'b0111; \ //enable 
    RF_FunSel = 2'b10; //load 
    
`define IN_REG2 \
    RF_RegSel = 4'b1011;\ //enable 
    RF_FunSel = 2'b10; //load 
    
`define IN_REG3 \
    RF_RegSel = 4'b1101; \ 
    RF_FunSel = 2'b10; //load 
`define IN_REG4 \
    RF_RegSel = 4'b1110;\ //enable 
    RF_FunSel = 2'b10; //load 

`define IN_IR(LH)\ 
    IR_Enable = 1; \
    IR_FunSel = 2'b10; \ //load 
    IR_LH = LH;

`define WRITE_RX(Regsel)\
    case("`RegSel") \
        2'b00: RF_RegSel = 4'b0111; \
        2'b01: RF_RegSel = 4'b1011; \
        2'b10: RF_RegSel = 4'b1101; \
        2'b11: RF_RegSel = 4'b1110; \       
        endcase

`define INC_ARF_PC \
    ARF_RegSel = 3'b011;\ //enable 
    ARF_FunSel = 2'b01; //increment 
`define INC_ARF_AR \
    ARF_RegSel = 3'b101;\ //enable 
    ARF_FunSel = 2'b01; //increment 
`define INC_ARF_SP \
    ARF_RegSel = 3'b110;\ //enable 
    ARF_FunSel = 2'b01; //increment 

`define INC_REG1 \
    RF_RegSel = 4'b0111;\ //enable 
    RF_FunSel = 2'b01; //increment 
`define INC_REG2 \
    RF_RegSel = 4'b1011;\ //enable 
    RF_FunSel = 2'b01; //increment 
`define INC_REG3 \
    RF_RegSel = 4'b1101;\ //enable 
    RF_FunSel = 2'b01; //increment 
`define INC_REG4 \
    RF_RegSel = 4'b1110;\ //enable 
    RF_FunSel = 2'b01; //increment 

`define INC_IR\ 
    IR_Enable = 1; \
    IR_FunSel = 2'b01; \ //increment

`define DEC_ARF_PC \
    ARF_RegSel = 3'b011;\ //enable 
    ARF_FunSel = 2'b00; //increment 
`define DEC_ARF_AR \
    ARF_RegSel = 3'b101;\ //enable 
    ARF_FunSel = 2'b00; //increment 
`define DEC_ARF_SP \
    ARF_RegSel = 3'b110;\ //enable 
    ARF_FunSel = 2'b00; //increment 

`define DEC_REG1 \
    RF_RegSel = 4'b0111;\ //enable 
    RF_FunSel = 2'b00; //increment 
`define DEC_REG2 \
    RF_RegSel = 4'b1011;\ //enable 
    RF_FunSel = 2'b00; //increment 
`define DEC_REG3 \
    RF_RegSel = 4'b1101;\ //enable 
    RF_FunSel = 2'b00; //increment 
`define DEC_REG4 \
    RF_RegSel = 4'b1110;\ //enable 
    RF_FunSel = 2'b00; //increment 

`define DEC_IR\ 
    IR_Enable = 1; \
    IR_FunSel = 2'b00; \ //decrement

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
    ARF_RegSel = 3'b000;\ //enable 
    ARF_FunSel = 2'b11; //clear

`define CLR_RF\
    RF_RegSel = 4'b0000;\ //enable 
    RF_FunSel = 2'b11; //clear

*/
module control_unit (
//input [3:0] opcode,
input [7:0] ir_15_8,
input [7:0] ir_7_0,
input [3:0] ALU_OutFlag,
input clk,
input reset,
output 
    reg [1:0] RF_OutASel, 
    reg [1:0] RF_OutBSel, 
    reg [1:0] RF_FunSel,
    reg [3:0] RF_RegSel,
    reg [3:0] ALU_FunSel,
    reg [1:0] ARF_OutCSel, 
    reg [1:0] ARF_OutDSel, 
    reg [1:0] ARF_FunSel,
    reg [2:0] ARF_RegSel,
    reg IR_LH,
    reg IR_Enable,
    reg [1:0] IR_Funsel,
    reg Mem_WR,
    reg Mem_CS,
    reg [1:0] MuxASel,
   reg [1:0] MuxBSel,
    reg MuxCSel
);
    reg [3:0] opcode;
    //Instruction type 1, has address reference
    reg AddressMode;
    reg [2:0] RegSel;
    //Instruction type 2, no address reference
    reg [3:0] Destreg;
    reg [7:0] Value;
    reg [3:0] SRCREG1, SRCREG2;
    
    reg [2:0] SeqCounter = 0;
    reg [15:0] ProgCounter = 0;
    reg finish = 0;
    always @(posedge clk) begin
        if (finish == 1'b1) begin
            SeqCounter <= 0;
        end
        else  begin
        SeqCounter <= SeqCounter + 1;
        ProgCounter <= ProgCounter + 1;
            
        end
    end
    
    always@(*) begin
        if(ProgCounter == 0 || reset ==1) begin
            `CLR_ARF
            `CLR_RF
        end
    end

    //Fetch
    always@(*)begin
    if(SeqCounter == 0) begin
        `OUT_D_ARF_PC
        `INC_ARF_PC
        `OUT_MEM
        `IN_IR(1)
    end

    if(SeqCounter == 1) begin
        `OUT_D_ARF_PC
        `INC_ARF_PC
        `OUT_MEM
        `IN_IR(0)
    end

    //Decode
    if(SeqCounter == 2) begin
        opcode = ir_15_8[7:4];
        if((opcode == 2'h00) || (opcode == 2'h01) || (opcode == 2'h02) || (opcode == 2'h0F) ) begin 
            RegSel <= ir_15_8[1:0];
            AddressMode <= ir_15_8[2];
        end else begin
            Destreg = ir_15_8[3:0];
            SRCREG2 = ir_7_0[3:0];
            SRCREG1 = ir_7_0[7:4];
        end

        
    end

    //Execute 1
    if(SeqCounter == 3) begin
        case(opcode) 
            2'h00:begin //BRA Branch
                `MUX_B_IROUT
                `IN_ARF_PC
                /*
                MuxBSel = 2'b01; //select ir output
                ARF_FunSel = 2'b10; //load arf
                ARF_RegSel = 3'b011; //enable PC only
                */
                end
            2'h01:begin //LD Load   
                `OUT_MEM
                if(AddressMode) begin
                    `MUX_A_IROUT
                end else begin
                    `MUX_A_MEMOUT
                end
                //MuxASel = AddressMode ? 2'b00 : 2'b01;
                `WRITE_RX(Regsel)
                
                end
            2'h02:begin //ST Store
                RF_OutBSel = RegSel;
                ALU_FunSel = 4'b0001; //pass B
                `OUT_MEM 
                //Mem_CS = 0;
                //Mem_WR = 1;
                end
            2'h03:begin //MOV Move
                ALU_FunSel = 4'b0000; //load A
                end
            2'h04:begin //AND
                ALU_FunSel = 4'b0111; //A and B
                end
            2'h05:begin //OR
                 ALU_FunSel = 4'b1000; //A or B
                end
            2'h06:begin //NOT
                ALU_FunSel = 4'b0010; // not A
                end
            2'h07:begin //ADD
                ALU_FunSel = 4'b0100; //  A + B
                end
            2'h08:begin //SUB
                ALU_FunSel = 4'b0110; //  A - B
                end
            2'h09:begin //LSR logical shift right
                ALU_FunSel = 4'b1010; //  A<<
                end
            2'h0A:begin //LSL
                ALU_FunSel = 4'b1011; //  >>A
                end
            2'h0B:begin // PUL
                `OUT_D_ARF_SP
                `OUT_MEM
                `MUX_A_MEMOUT
                `WRITE_RX(Regsel)

                //RF_OutBSel = RegSel;
                //ALU_FunSel = 4'b0001; //pass B
                //ARF_OutDSel = 2'b11; // select SP
                /*
                Mem_CS = 0;
                Mem_WR = 1;
                //,
                ARF_RegSel = 3'b110;
                ARF_FunSel = 2'b00;
                */
                end
            2'h0C:begin //PSH
                `DEC_ARF_SP
                /*
                ARF_RegSel = 3'b110;
                ARF_FunSel = 2'b01;
                
                ARF_OutDSel = 2'b11; // select SP
                Mem_CS = 0;
                Mem_WR = 0;
                MuxASel = 2'b01; //memory output
                
                RF_FunSel = 2'b10;
                */
                end
            2'h0D:begin //INC
                ALU_FunSel = 4'b0000; //load A
                end
            2'h0E:begin //DEC
                ALU_FunSel = 4'b0000; //load A
                end
            2'h0F:begin // BNE
                    if (ALU_OutFlag[3] == 0) begin
                        MuxBSel = AddressMode ? 2'b01 : 2'b10;
                        ARF_FunSel = 2'b10;
                        ARF_RegSel = 3'b011;
                    end
                end
            
            

            endcase

            if((opcode >= 2'h03 && opcode <= 2'h0A) ||  opcode == 2'h0D ||  opcode == 2'h0E ) begin 
            if(Destreg >= 4'b0100) begin 
                MuxASel = 2'b11;
                RF_FunSel = 2'b10;
            end else begin
                MuxBSel = 2'b11;
                ARF_FunSel = 2'b10;
            end
            
            if( opcode == 2'h0D )begin
                if(SRCREG1>= 4'b0100) begin
                    RF_FunSel = 2'b01; //increment
                end else begin
                    ARF_FunSel = 2'b01;
                end
            end
            
                
            if(opcode == 2'h0E) begin
                if(SRCREG1>= 4'b0100) begin
                    RF_FunSel = 2'b00; //decrement
                end else begin
                    ARF_FunSel = 2'b00;
                 end
            end
            case(Destreg)
            4'b0000:begin
                ARF_RegSel = 3'b011;
                end
            4'b0001:begin
                ARF_RegSel = 3'b011;
                end
            4'b0010:begin
                ARF_RegSel = 3'b101;
                end
            4'b0011:begin
                ARF_RegSel = 3'b110;
                end
            4'b0100:begin
                RF_RegSel = 4'b0111;
                end
            4'b0101:begin
                RF_RegSel = 4'b1011;
                end
            4'b0110:begin
                RF_RegSel = 4'b1101;
                end
            4'b0111:begin
                RF_RegSel = 4'b1110;
                end
        
            endcase
            
            case(SRCREG1)
             4'b0000:begin
                ARF_OutCSel = 2'b00;
                end
            4'b0001:begin
                ARF_OutCSel = 2'b00;
                end
            4'b0010:begin
                ARF_OutCSel = 2'b10;
                end
            4'b0011:begin
                ARF_OutCSel = 2'b11;
                end
            4'b0100:begin
                RF_OutBSel = 2'b00;
                end
            4'b0101:begin
                RF_OutBSel = 2'b01;
                end
            4'b0110:begin
                RF_OutBSel = 2'b10;
                end
            4'b0111:begin
                RF_OutBSel = 2'b11;
                end
                    
            endcase

            case (SRCREG2)
            4'b0100:begin
                RF_OutASel = 2'b00;
                end
            4'b0101:begin
                RF_OutASel = 2'b01;
                end
            4'b0110:begin
                RF_OutASel = 2'b10;
                end
            4'b0111:begin
                RF_OutASel = 2'b11;
                end
            endcase

            MuxCSel = ((SRCREG2 < 4'b0100) || (SRCREG1 < 4'b0100)) ? 0:1;
            end 
           
        end
        
    

    //Execute 2
    if(SeqCounter == 4) begin
        case(opcode) 
            2'h0B:begin // PUL
                `INC_ARF_SP
                end
            2'h0C:begin //PSH
                RF_OutBSel = RegSel;
                ALU_FunSel = 4'b0001; //pass B
                `OUT_D_ARF_SP
                `IN_MEM
                end
                
            default: begin
                finish = 1;
            end

        
        endcase
    end

    if(SeqCounter == 5) 
        finish = 1;
    
    end
    /*
    always@(*) begin
        if((opcode == 2'h00) || (opcode == 2'h01) || (opcode == 2'h02) || (opcode == 2'h0F) ) begin 
            RegSel <= ir_11_8[1:0];
            AddressMode <= ir_11_8[2];
            */
            /*
            if(~AddressMode) begin //direct addressing
                MuxBSel = 2'b01; //select arf
                ARF_FunSel = 2'b10; //load arf
                ARF_RegSel = 3'b101; //enable address register only
                ARF_OutDSel = 2'b10; //output AR to memory
                Mem_CS = 0; //memory enabled
                Mem_WR = 0; //read memory
                //memory returned value                
            end
            */
            /*
        end else begin
            Destreg = ir_11_8[3:0];
            SRCREG2 = ir_7_0[3:0];
            SRCREG1 = ir_7_0[7:4];
        end
    end
    */
    /*
    
    always@(*) begin
        if(opcode == 2'h01 || opcode == 2'h0C) begin //enable regfile register for writing only
        case(RegSel)
        2'b00: RF_RegSel = 4'b0111;
        2'b01: RF_RegSel = 4'b1011;
        2'b10: RF_RegSel = 4'b1101;
        2'b11: RF_RegSel = 4'b1110;        
        endcase
        end 
        
        
        
        /*else begin
         RF_RegSel = 4'b1111; // disable registers 
        end */
       
    
    /*
    always@(*) begin
         if((opcode >= 2'h03 && opcode <= 2'h0A) ||  opcode == 2'h0D ||  opcode == 2'h0E ) begin 
            if(Destreg >= 4'b0100) begin 
                MuxASel = 2'b11;
                RF_FunSel = 2'b10;
            end else begin
                MuxBSel = 2'b11;
                ARF_FunSel = 2'b10;
            end
            
            if( opcode == 2'h0D )begin
                if(SRCREG1>= 4'b0100) begin
                    RF_FunSel = 2'b01; //increment
                end else begin
                    ARF_FunSel = 2'b01;
                end
            end
            
                
            if(opcode == 2'h0E) begin
                if(SRCREG1>= 4'b0100) begin
                    RF_FunSel = 2'b00; //decrement
                end else begin
                    ARF_FunSel = 2'b00;
                 end
            end
            case(Destreg)
            4'b0000:begin
                ARF_RegSel = 3'b011;
                end
            4'b0001:begin
                ARF_RegSel = 3'b011;
                end
            4'b0010:begin
                ARF_RegSel = 3'b101;
                end
            4'b0011:begin
                ARF_RegSel = 3'b110;
                end
            4'b0100:begin
                RF_RegSel = 4'b0111;
                end
            4'b0101:begin
                RF_RegSel = 4'b1011;
                end
            4'b0110:begin
                RF_RegSel = 4'b1101;
                end
            4'b0111:begin
                RF_RegSel = 4'b1110;
                end
        
            endcase
            
            case(SRCREG1)
             4'b0000:begin
                ARF_OutCSel = 2'b00;
                end
            4'b0001:begin
                ARF_OutCSel = 2'b00;
                end
            4'b0010:begin
                ARF_OutCSel = 2'b10;
                end
            4'b0011:begin
                ARF_OutCSel = 2'b11;
                end
            4'b0100:begin
                RF_OutBSel = 2'b00;
                end
            4'b0101:begin
                RF_OutBSel = 2'b01;
                end
            4'b0110:begin
                RF_OutBSel = 2'b10;
                end
            4'b0111:begin
                RF_OutBSel = 2'b11;
                end
                    
            endcase

            case (SRCREG2)
            4'b0000:begin
                ARF_OutCSel = 2'b00;
                end
            4'b0001:begin
                ARF_OutCSel = 2'b00;
                end
            4'b0010:begin
                ARF_OutCSel = 2'b10;
                end
            4'b0011:begin
                ARF_OutCSel = 2'b11;
                end
            4'b0100:begin
                RF_OutASel = 2'b00;
                end
            4'b0101:begin
                RF_OutASel = 2'b01;
                end
            4'b0110:begin
                RF_OutASel = 2'b10;
                end
            4'b0111:begin
                RF_OutASel = 2'b11;
                end
            endcase

            MuxCSel = ((SRCREG2 < 4'b0100) || (SRCREG1 < 4'b0100)) ? 0:1;
            end 
           
        end
    */
     
    
    
    
    
    
endmodule

module ALUSystem
( input
    [1:0] RF_OutASel, 
    [1:0] RF_OutBSel, 
    [1:0] RF_FunSel,
    [3:0] RF_RegSel,
    [3:0] ALU_FunSel,
    [1:0] ARF_OutCSel, 
    [1:0] ARF_OutDSel, 
    [1:0] ARF_FunSel,
    [2:0] ARF_RegSel,
    IR_LH,
    IR_Enable,
    [1:0] IR_Funsel,
    Mem_WR,
    Mem_CS,
    [1:0] MuxASel,
    [1:0] MuxBSel,
    MuxCSel,
    Clock
    );
wire [7:0] ALUOut;
wire [7:0] Address;
wire [7:0] MemoryOut;
wire [7:0] ARF_COut;
reg [7:0] MuxBOut;
wire [7:0] IR_Out_LSB;
Memory Mem(.address(Address), .data(ALUOut), .wr(Mem_WR), .cs(Mem_CS), .clock(Clock), .o(MemoryOut));
//address, data ve output 8 bit gerisi tek bit

ARF arf1(.OutCSel(ARF_OutCSel), .OutDSel(ARF_OutDSel), .FunSel(ARF_FunSel), .RegSel(ARF_RegSel), .I(MuxBOut) , .OutC(ARF_COut), .OutD(Address), .CLK(Clock));

always @(*) begin
    case (MuxBSel)
        2'b01: begin
            MuxBOut <= IR_Out_LSB;
        end
        2'b10: begin
            MuxBOut <= MemoryOut;
        end
        2'b11: begin
            MuxBOut <= ALUOut;
        end
    endcase
end

wire [15:0] IROut;

assign IR_Out_LSB = IROut[7:0];

IR ir1(.LH(IR_LH), .En(IR_Enable), .FunSel(IR_Funsel), .IRout(IROut), .CLK(Clock));

reg [7:0] MuxAOut;

always @(*) begin
    case (MuxASel)
        2'b00: begin
            MuxAOut = IR_Out_LSB;
        end
        2'b01: begin
            MuxAOut = MemoryOut;
        end
        2'b10: begin
            MuxAOut = ARF_COut;
        end
        2'b11: begin
            MuxAOut = ALUOut;
        end
    endcase
end

wire [7:0] AOut, BOut;
RegFile rf1(.OutASel(RF_OutASel), .OutBSel(RF_OutBSel), .FunSel(RF_FunSel), .RegSel(RF_RegSel),  .I(MuxAOut), .OutA(AOut), .OutB(BOut),.CLK(Clock));

wire [7:0] MuxCOut;

assign MuxCOut = MuxCSel ? AOut: ARF_COut;
/*
always @(MuxCSel) begin
    if (MuxCSel) begin
        MuxCOut = AOut;
    end else begin
        MuxCOut = ARF_COut;
    end
    
end
*/
wire [3:0] ALUOutFlag;
ALU alu1(.FunSel(ALU_FunSel), .A(MuxCOut), .B(BOut), .OutALU(ALUOut), .OutFlag(ALUOutFlag), .CLK(Clock));

endmodule

module top_system( input Clock, input reset);


  wire  [1:0] RF_OutASel; 
  wire  [1:0] RF_OutBSel; 
  wire  [1:0] RF_FunSel;
  wire  [3:0] RF_RegSel;
  wire  [3:0] ALU_FunSel;
  wire  [1:0] ARF_OutCSel; 
  wire  [1:0] ARF_OutDSel; 
  wire  [1:0] ARF_FunSel;
  wire  [2:0] ARF_RegSel;
  wire  IR_LH;
  wire  IR_Enable;
  wire  [1:0] IR_Funsel;
  wire  Mem_WR;
  wire  Mem_CS;
  wire  [1:0] MuxASel;
  wire  [1:0] MuxBSel;
  wire  MuxCSel;

control_unit cpu(
.ir_15_8(IROut[15:8]),
.ir_7_0(IROut[7:0]),
.ALU_OutFlag(ALUOutFlag),
.clk(Clock),
.RF_OutASel(RF_OutASel), 
.RF_OutBSel(RF_OutBSel), 
.RF_FunSel(RF_FunSel),
.RF_RegSel(RF_RegSel),
.ALU_FunSel(ALU_FunSel),
.ARF_OutCSel(ARF_OutCSel), 
.ARF_OutDSel(ARF_OutDSel), 
.ARF_FunSel(ARF_FunSel),
.ARF_RegSel(ARF_RegSel),
.IR_LH(IR_LH),
.IR_Enable(IR_Enable),
.IR_Funsel(IR_Funsel),
.Mem_WR(Mem_WR),
.Mem_CS(Mem_CS),
.MuxASel(MuxASel),
.MuxBSel(MuxBSel),
.MuxCSel(MuxCSel),
.reset(reset)
);

ALUSystem ALU
( 
    .RF_OutASel(RF_OutASel),, 
    .RF_OutBSel(RF_OutBSel), 
    .RF_FunSel(RF_FunSel),  
    .RF_RegSel(RF_RegSel),  
    .ALU_FunSel(ALU_FunSel), 
    .ARF_OutCSel(ARF_OutCSel), 
    .ARF_OutDSel(ARF_OutDSel), 
    .ARF_FunSel(ARF_FunSel),
    .ARF_RegSel(ARF_RegSel),
    .IR_LH(IR_LH),
    .IR_Enable(IR_Enable),
    .IR_Funsel(IR_Funsel),
    .Mem_WR(Mem_WR),
    .Mem_CS(Mem_CS),
    .MuxASel(MuxASel),
    .MuxBSel(MuxBSel),
    .MuxCSel(MuxCSel),
    .Clock(Clock)
    );
endmodule



