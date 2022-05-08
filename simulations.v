`timescale 1ns/1ps

module Register_Test();
    reg En;
    reg CLK;
    reg [1:0] FunSel;
    reg [7:0] I;
    
    wire [7:0] Q;
     //integer e, f, i;    
     reg [1:0] e = 1'b0;
     reg [1:0] f = 2'b00;
     
     reg [7:0] i = 8'b00000000;
     
    n_bitRegister #(.N(8)) R1(.E(En),.CLK(CLK), .FunSel(FunSel), .I(I), .Q(Q));
    always #1 CLK = ~CLK;
    always #100 En = ~En;
    
     initial begin
     I = 8'b10101010;
     En = 1;
     CLK = 0;
     
         /*
            $display("Enable:%0d ", E );
        
        E =1;
        for(f = 0; f<3'b100; f = f+1) begin
        #10;
            FunSel = f;
            $display("FunSel:%0d ", f );
        end
        #5;
       
        E =0;
        #5;
         $display("Enable:%0d ", E );
         for(f = 0; f<3'b100; f = f+1) begin
                 #10;
                     FunSel = f;
                     $display("FunSel:%0d ", f );
                 end
            */
            
     forever
                begin 
                
                #5
                for(f = 0; f<3'b100; f = f+1) begin
                #10
                FunSel = f;
                $display("FunSel1:%0d ", f );
                end
                //#5
               // En = 0;
                for(f = 0; f<3'b100; f = f+1) begin
                    #10
                    FunSel = f;
                    $display("FunSel2:%0d ", f );
                
                 end
                $finish;  
    
     end
     
    end
     
     
     
endmodule     
     
     