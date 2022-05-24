
//Part 1 


module n_bitRegister #(parameter N = 8) (
    input CLK, E, [1:0] FunSel, [N-1:0] I,
    output [N-1:0] Q
);

    reg [N-1:0] Q_temp;
    assign Q = Q_temp;
    always @( negedge CLK ) begin
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
    wire [15:0] IR_Q;
    
    n_bitRegister #(.N(16)) IR(.CLK(CLK),.E(En), .FunSel(FunSel), .I(I_temp), .Q(IR_Q));
    
    assign IRout = LH ? I_temp[15:8] : I_temp[7:0];
    
    /*
    always @(LH) begin
        case (LH)
            0: begin
                I_temp[15:8] = I;
            end
            1: begin
                I_temp[7:0] = I;
            end
        endcase
    end
    */
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







