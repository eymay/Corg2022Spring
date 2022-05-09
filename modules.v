
//Part 1 


module n_bitRegister #(parameter N = 8) (
    input CLK, E, [1:0] FunSel, [N-1:0] I,
    output [N-1:0] Q
);

    reg [N-1:0] Q_temp;
    assign Q = Q_temp;
    always @( posedge CLK or E) begin
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
    input [1:0] OutASel, [1:0] OutBSel, [1:0] FunSel, [3:0] RegSel, [7:0] I, CLK,
    output [7:0] OutA, [7:0] OutB
);
    wire [7:0] R1_Q;
    wire [7:0] R2_Q;
    wire [7:0] R3_Q;
    wire [7:0] R4_Q;
    n_bitRegister #(.N(8)) R1(.CLK(CLK),.E(~RegSel[0]), .FunSel(FunSel), .I(I), .Q(R1_Q));
    n_bitRegister #(.N(8)) R2(.CLK(CLK),.E(~RegSel[1]), .FunSel(FunSel), .I(I), .Q(R2_Q));
    n_bitRegister #(.N(8)) R3(.CLK(CLK),.E(~RegSel[2]), .FunSel(FunSel), .I(I), .Q(R3_Q));
    n_bitRegister #(.N(8)) R4(.CLK(CLK),.E(~RegSel[3]), .FunSel(FunSel), .I(I), .Q(R4_Q));

    //wire [3:0] R_En;

    reg [7:0] OutA_temp, OutB_temp;
    assign OutA = OutA_temp;
    assign OutB = OutB_temp;
    always@(OutASel) begin
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
    always@(OutBSel) begin
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
    input [1:0] OutCSel, [1:0] OutDSel, [1:0] FunSel, [3:0] RegSel, [7:0] I,CLK,
    output [7:0] OutC, [7:0] OutD
);

    wire [7:0] PC_Q;
    wire [7:0] AR_Q;
    wire [7:0] SP_Q;
    
    n_bitRegister #(.N(8)) PC(.CLK(CLK),.E(~RegSel[0]), .FunSel(FunSel), .I(I), .Q(PC_Q));
    n_bitRegister #(.N(8)) AR(.CLK(CLK),.E(~RegSel[1]), .FunSel(FunSel), .I(I), .Q(AR_Q));
    n_bitRegister #(.N(8)) SP(.CLK(CLK),.E(~RegSel[2]), .FunSel(FunSel), .I(I), .Q(SP_Q));



    reg [7:0] OutC_temp, OutD_temp;
    assign OutC = OutC_temp;
    assign OutD = OutD_temp;

    always@(OutCSel) begin
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

    always@(OutDSel) begin
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
    input NL_H, En, [1:0] FunSel, [7:0] I,CLK,
    output [15:0] IRout
);
    reg [15:0] I_temp; //reg
    wire [15:0] IR_Q;
    
    n_bitRegister #(.N(16)) IR(.CLK(CLK),.E(En), .FunSel(FunSel), .I(I_temp), .Q(IR_Q));
    
    assign IRout = IR_Q;

    always @(NL_H) begin
        case (NL_H)
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
    input [3:0] FunSel, input [7:0] A, [7:0] B, Cin, 
    output reg[7:0] OutALU, 
    output reg [3:0] OutFlag 
);

reg enable_c, enable_o;
    always @(FunSel) 
    case (FunSel)
        4'b0000: begin
            OutALU = A;
            enable_c = 0;
            enable_o = 0;
        end
        4'b0001: begin
            OutALU = B;
            enable_c = 0;
            enable_o = 0;
        end
        4'b0010: begin
            OutALU = ~A;
            enable_c = 0;
            enable_o = 0;
        end
        4'b0011: begin
            OutALU = ~B;
            enable_c = 0;
            enable_o = 0;
        end
        4'b0100: begin
            OutALU = A + B;
            enable_c = 1;
            enable_o = 1;
        end
        4'b0101: begin
            OutALU = A + B + Cin;
            enable_c = 1;
            enable_o = 1;
        end
        4'b0110: begin
            OutALU = A - B;
            enable_c = 1;
            enable_o = 1;
        end
        4'b0111: begin
            OutALU = A & B;
            enable_c = 0;
            enable_o = 0;
        end
        4'b1000: begin
            OutALU = A | B;
            enable_c = 0;
            enable_o = 0;
        end
        4'b1001: begin
            OutALU = A ^ B;
            enable_c = 0;
            enable_o = 0;
        end
        4'b1010: begin
            OutFlag[1] = A[7];
            OutALU = A << 1;
            enable_c = 1;
            enable_o = 0;
        end
        4'b1011: begin
            OutFlag[1] = A[0];
            OutALU = A >> 1;
            enable_c = 1;
            enable_o = 0;
        end
        4'b1100: begin
            OutALU = A <<< 1;
            enable_c = 0;
            enable_o = 1;
        end
        4'b1101: begin
            OutALU = A >>> 1;
            enable_c = 0;
            enable_o = 0;
        end
        4'b1110: begin
            OutFlag[1] <= A[7];
            OutALU[0] <= OutFlag[1];
            OutALU[1] <= A[0];
            OutALU[2] <= A[1];
            OutALU[3] <= A[2];
            OutALU[4] <= A[3];
            OutALU[5] <= A[4];
            OutALU[6] <= A[5];
            OutALU[7] <= A[6];
            enable_c = 1;
            enable_o = 1;
            //OutALU = {A[6:0], A[7]};
        end
        4'b1111: begin
            OutFlag[1] <= A[0];
            OutALU[0] <= A[1];
            OutALU[1] <= A[2];
            OutALU[2] <= A[3];
            OutALU[3] <= A[4];
            OutALU[4] <= A[5];
            OutALU[5] <= A[6];
            OutALU[6] <= A[7];
            OutALU[7] <= OutFlag[1];
            //OutALU = { A[0], A[7:1]};
            enable_c = 1;
            enable_o = 1;
        end
    endcase
    always @(OutALU) begin
        if(OutALU == 0) begin
            OutFlag[0] = 1;
        end else begin
            OutFlag[0] = 0;
        end

        if(OutALU[7] == 1) begin
            OutFlag[2] = 1;
        end else begin
            OutFlag[2] = 0;
        end
        
        if(A[7] == ~OutALU[7] & enable_o)
            OutFlag[3] = 1;

        /*
        if(OutALU > 8'b11111111) begin
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
wire [7:0] OutALU;
wire [7:0] Address;
wire [7:0] MemOut;
wire [7:0] OutC_ARF;
wire [7:0] I_ARF, IR_Out_LSB;
Memory Mem(.address(Address), .data(OutALU), .wr(Mem_WR), .cs(Mem_CS), .clock(Clock), .o(MemOut));
//address, data ve output 8 bit gerisi tek bit

ARF arf1(.OutCSel(ARF_OutCSel), .OutDSel(ARF_OutDSel), .FunSel(ARF_FunSel), .RegSel(ARF_RegSel), .I(I_ARF) , .OutC(OutC_ARF), .OutD(Address));

always @(MuxBSel) begin
    case (MuxBSel)
        2'b01: begin
            I_ARF = IR_Out_LSB;
        end
        2'b10: begin
            I_ARF = MemOut;
        end
        2'b11: begin
            I_ARF = OutALU;
        end
    endcase
end

wire [15:0] IR_Out;

assign IR_Out_LSB = IR_Out[7:0];

IR ir1(.LH(IR_LH), .Enable(IR_Enable), .FunSel(IR_Funsel), .Out(IR_Out));

wire [7:0] MuxAOut;

always @(MuxASel) begin
    case (MuxASel)
        2'b00: begin
            MuxAOut = IR_Out_LSB;
        end
        2'b01: begin
            MuxAOut = Mem_Out;
        end
        2'b10: begin
            MuxAOut = OutC_ARF;
        end
        2'b11: begin
            MuxAOut = OutALU;
        end
        
end

wire [7:0] RegFileOutA, RegFileOutB;
RF rf1(.OutASel(RF_OutASel), .OutBSel(RF_OutBSel), .FunSel(RF_FunSel), .RegSel(RF_RegSel),  .I(MuxAOut), .A(RegFileOutA), .B(RegFileOutB));

wire [7:0] MuxCOut
always @(MuxCSel) begin
    if (MuxCsel) begin
        MuxCOut = RegFileOutA;
    end else begin
        MuxCOut = OutC_ARF;
    end
    
end

ALU alu1(.FunSel(ALU_FunSel), .A(MuxCOut), .B(RegFileOutB), .Out(OutALU));


*/






