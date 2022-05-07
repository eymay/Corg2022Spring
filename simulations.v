`timescale 1ns/1ps

module Register_Test();
    reg E;
    reg [1:0] FunSel;
    reg [7:0] I;
    
    wire [7:0] Q;
     //integer e, f, i;    
     reg [1:0] e = 2'b00;
     reg [1:0] f = 2'b00;
     
     reg [7:0] i = 8'b00000000;
     
    n_bitRegister #(.N(8)) R1(.E(E), .FunSel(FunSel), .I(I), .Q(Q));
     
     initial begin
     I = 8'b10101010;

     
     for(e = 0; e < 2'b10; e = e+1) begin
            $display("Enable:%0d ", e );
        #10;
        E = e;
        for(f = 0; f<3'b100; f = f+1) begin
        #10;
            FunSel = f;
            $display("FunSel:%0d ", f );
        end
       
     end
      $finish;
     end
     
     
     
endmodule     
     
     