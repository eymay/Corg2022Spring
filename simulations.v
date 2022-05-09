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
                /*for(f = 0; f<3'b100; f = f+1) begin
                    #10
                    FunSel = f;
                    $display("FunSel2:%0d ", f );
                
                 end*/
                $finish;  
    
     end
     
    end
     
     
     
endmodule     

module RegFile_Test();
    reg CLK;
    reg [1:0] FunSelect;
    reg [7:0] Input;
    reg [1:0] OutASelect;
    reg [1:0] OutBSelect;
    reg [3:0] RegisterSelect;
    
    wire [7:0] OutputA;
    wire [7:0] OutputB;
    
     //integer e, f, i;    
     reg [3:0] r = 4'b0000;
     reg [1:0] f = 2'b00;
     
     reg [7:0] i = 8'b00000000;
     reg [1:0] o = 2'b00;
    integer a,j,k;
    RegFile file(.OutASel(OutASelect),.OutBSel(OutBSelect),.FunSel(FunSelect),.RegSel(RegisterSelect),.I(Input),.CLK(CLK),.OutA(OutputA),.OutB(OutputB));
    always #1 CLK = ~CLK;
    
     initial begin
     #5
     Input = 8'b10101010;
     CLK = 0;
     for(a=0;a<16;a=a+1) begin
        RegisterSelect = a;
        $display("RegisterSelect: %0d",a);
        for(j=0;j<4;j=j+1) begin
            #10
            FunSelect = j;
            $display("FunSelect: %0d ", j);
            for(k = 0; k < 4; k = k+1)begin
                #5
                OutASelect = k;
                OutBSelect = k;
                $display("OutA and OutB Select: %0d",k);
            end
        end
     end

          
    end
     
     
     
endmodule     

     