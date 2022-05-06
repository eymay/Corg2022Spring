
//Part 1 


module n-bitRegister #(
    parameter N
) (
    input E, [1:0] FunSel, [N-1:0] I,
    output [N-1:0] Q
);

    wire [N-1:0] Q_temp;
    assign Q = Q_temp;
    always @( E) begin
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
    end
    
endmodule

module RegFile (
    input [1:0] OutASel, [1:0] OutBSel, [1:0] FunSel, [3:0] RegSel, [7:0] I,
    output [7:0] OutA, [7:0] OutB
);
    n-bitRegister #(.N(8)) R1(.E(~RegSel[0]), .FunSel(FunSel), .I(I), .Q(R1_Q));
    n-bitRegister #(.N(8)) R2(.E(~RegSel[1]), .FunSel(FunSel), .I(I), .Q(R2_Q));
    n-bitRegister #(.N(8)) R3(.E(~RegSel[2]), .FunSel(FunSel), .I(I), .Q(R3_Q));
    n-bitRegister #(.N(8)) R4(.E(~RegSel[3]), .FunSel(FunSel), .I(I), .Q(R4_Q));
    wire [7:0] R1_Q;
    wire [7:0] R2_Q;
    wire [7:0] R3_Q;
    wire [7:0] R4_Q;
    //wire [3:0] R_En;

    wire [7:0] OutA_temp, OutB_temp;
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
    input [1:0] OutCSel, [1:0] OutDSel, [1:0] FunSel, [3:0] RegSel, [7:0] I,
    output [7:0] OutC, [7:0] OutD
);

    n-bitRegister #(.N(8)) PC(.E(~RegSel[0]), .FunSel(FunSel), .I(I), .Q(PC_Q));
    n-bitRegister #(.N(8)) AR(.E(~RegSel[1]), .FunSel(FunSel), .I(I), .Q(AR_Q));
    n-bitRegister #(.N(8)) SP(.E(~RegSel[2]), .FunSel(FunSel), .I(I), .Q(SP_Q));

    wire [7:0] PC_Q;
    wire [7:0] AR_Q;
    wire [7:0] SP_Q;

    wire [7:0] OutC_temp, OutC_temp;
    assign OutC = OutC_temp;
    assign OutD = OutD_temp;

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
    input NL/H, En, [1:0] FunSel, [7:0] I,
    output [15:0] IRout
);
    wire [15:0] I_temp;

    n-bitRegister #(.N(16)) IR(.E(En), .FunSel(FunSel), .I(I_temp), .Q(IR_Q));
    wire [15:0] IR_Q;
    assign IRout = IR_Q;

    always @(NL/H) begin
        case (NL/H)
            0: begin
                I_temp[15:8] = I;
            end
            1: begin
                I_temp[7:0] = I;
            end
            default: 
        endcase
    end
    
endmodule

module ALU (
    input [3:0] FunSel, [7:0] A, [7:0] B, Cin, 
    output [7:0] OutALU, 
    output reg [3:0] OutFlag 
);

    always @(FunSel) 
    case (FunSel)
        4'b0000: begin
            OutALU = A;
        end
        4'b0001: begin
            OutALU = B;
        end
        4'b0010: begin
            OutALU = ~A;
        end
        4'b0011: begin
            OutALU = ~B;
        end
        4'b0100: begin
            OutALU = A + B;
        end
        4'b0101: begin
            OutALU = A + B + Cin;
        end
        4'b0110: begin
            OutALU = A - B;
        end
        4'b0111: begin
            OutALU = A & B;
        end
        4'b1000: begin
            OutALU = A | B;
        end
        4'b1001: begin
            OutALU = A ^ B;
        end
        4'b1010: begin
            OutFlag[1] = A[7];
            OutALU = A << 1;
        end
        4'b1011: begin
            OutFlag[1] = A[0];
            OutALU = A >> 1;
        end
        4'b1100: begin
            OutALU = A <<< 1;
        end
        4'b1101: begin
            OutALU = A >>> 1;
        end
        4'b1110: begin
            OutFlag[1] <= A[7];
            A[0] <= OutFlag[1];
            A[1] <= A[0];
            A[2] <= A[1];
            A[3] <= A[2];
            A[4] <= A[3];
            A[5] <= A[4];
            A[6] <= A[5];
            A[7] <= A[6];
            
            //OutALU = {A[6:0], A[7]};
        end
        4'b1111: begin
            OutFlag[1] <= A[0];
            A[0] <= A[1];
            A[1] <= A[2];
            A[2] <= A[3];
            A[3] <= A[4];
            A[4] <= A[5];
            A[5] <= A[6];
            A[6] <= A[7];
            A[7] <= OutFlag[1];
            //OutALU = { A[0], A[7:1]};
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

        if(OutALU > 8'b11111111) begin
            OutFlag[3] = 1;
        end else begin
            OutFlag[3] = 0;
        end

endmodule