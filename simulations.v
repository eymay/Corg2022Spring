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


module ARF_Test();
    reg CLK;
    reg [1:0] FunSelect;
    reg [7:0] Input;
    reg [1:0] OutCSelect;
    reg [1:0] OutDSelect;
    reg [3:0] RegisterSelect;
    
    wire [7:0] OutputC;
    wire [7:0] OutputD;
    
     //integer e, f, i;    
     reg [3:0] r = 4'b0000;
     reg [1:0] f = 2'b00;
     
     reg [7:0] i = 8'b00000000;
     reg [1:0] o = 2'b00;
    integer a,j,k;

    ARF file(.OutCSel(OutCSelect),.OutDSel(OutDSelect),.FunSel(FunSelect),.RegSel(RegisterSelect),.I(Input),.CLK(CLK),.OutC(OutputC),.OutD(OutputD));
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
                OutCSelect = k;
                OutDSelect = k;
                $display("OutA and OutB Select: %0d",k);
            end
        end
     end

          
    end
endmodule    


module IR_Test();
    reg CLK;
    reg E;
    reg LH;

    reg [1:0] FunSelect;
    reg [7:0] Input;
    
    wire [15:0] Out;
    
    integer i,j,k;
    
    IR Ir(.NL_H(LH),.En(E),.FunSel(FunSelect),.I(Input),.CLK(CLK));
    always #1 CLK = ~CLK;
    always #50 E = ~E;
    always #100 LH = ~LH;
    
    initial begin
         Input = 8'b10101010; //for low input
         E = 1;
         CLK = 0;
         LH=0;
         forever begin
            #5
            for(i=0; i < 4; i = i + 1) begin
                #10
                FunSelect = i;
                $display("FunSelect: %0d",i);
            end
         end
         
          
         
         
    end



endmodule 

module test_alu();
    reg [7:0] A, B;
    reg [3:0] FunSel;
    reg [1:0] Reg_FunSel;
    reg CLK;
    wire [7:0] OutALU;
    wire [3:0] OutFlag;

    ALU uut(CLK, FunSel, A, B, OutALU, OutFlag);
    
    always #1 CLK = ~CLK;
    initial begin
        Reg_FunSel = 2'b11;
        CLK = 0;
        #5
        Reg_FunSel = 2'b01;
        
        A = 8'b1010_1111; B = 8'b0111_1101; FunSel = 4'b0000; #100;

        FunSel = 4'b0001; #100;

        FunSel = 4'b0010; #100;

        FunSel = 4'b0011; #100;

        FunSel = 4'b0111; #100;

        FunSel = 4'b1000; #100;

        FunSel = 4'b1001; #100;
        $finish;
    end

endmodule

module Project1Test();
    //Input Registers of ALUSystem
    reg[1:0] RF_OutASel; 
    reg[1:0] RF_OutBSel; 
    reg[1:0] RF_FunSel;
    reg[3:0] RF_RegSel;
    reg[3:0] ALU_FunSel;
    reg[1:0] ARF_OutCSel; 
    reg[1:0] ARF_OutDSel; 
    reg[1:0] ARF_FunSel;
    reg[2:0] ARF_RegSel;
    reg      IR_LH;
    reg      IR_Enable;
    reg[1:0]      IR_Funsel;
    reg      Mem_WR;
    reg      Mem_CS;
    reg[1:0] MuxASel;
    reg[1:0] MuxBSel;
    reg MuxCSel;
    reg      Clock;
    
    //Test Bench Connection of ALU System
    ALUSystem _ALUSystem(
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
    .Clock(Clock)
    );
    
    //Test Vector Variables
    reg [31:0] VectorNum, Errors, TotalLine; 
    reg [34:0] TestVectors[10000:0];
    reg Reset, Operation;
    
    //Clock Signal Generation
    always 
    begin
        Clock = 1; #5; Clock = 0; #5; // 10ns period
    end
    
    //Read Test Bench Values
    initial begin
        $readmemb("TestBench.mem", TestVectors); // Read vectors
        VectorNum = 0; Errors = 0; TotalLine=0; Reset=0;// Initialize
    end
    
    // Apply test vectors on rising edge of clock
    always @(posedge Clock)
    begin
        #1; 
        {Operation, RF_OutASel, RF_OutBSel, RF_FunSel, 
        RF_RegSel, ALU_FunSel, ARF_OutCSel, ARF_OutDSel, 
        ARF_FunSel, ARF_RegSel, IR_LH, IR_Enable, IR_Funsel, 
        Mem_WR, Mem_CS, MuxASel, MuxBSel, MuxCSel} = TestVectors[VectorNum];
    end
    
    // Check results on falling edge of clk
    always @(negedge Clock)
        if (~Reset) // skip during reset
        begin
            $display("Input Values:");
            $display("Operation: %d", Operation);
            $display("Register File: OutASel: %d, OutBSel: %d, FunSel: %d, Regsel: %d", RF_OutASel, RF_OutBSel, RF_FunSel, RF_RegSel);            
            $display("ALU FunSel: %d", ALU_FunSel);
            $display("Addres Register File: OutCSel: %d, OutDSel: %d, FunSel: %d, Regsel: %d", ARF_OutCSel, ARF_OutDSel, ARF_FunSel, ARF_RegSel);            
            $display("Instruction Register: LH: %d, Enable: %d, FunSel: %d", IR_LH, IR_Enable, IR_Funsel);            
            $display("Memory: WR: %d, CS: %d", Mem_WR, Mem_CS);
            $display("MuxASel: %d, MuxBSel: %d, MuxCSel: %d", MuxASel, MuxBSel, MuxCSel);
            
            $display("");
            $display("Ouput Values:");
            $display("Register File: AOut: %d, BOut: %d", _ALUSystem.AOut, _ALUSystem.BOut);            
            $display("ALUOut: %d, ALUOutFlag: %d, ALUOutFlags: Z:%d, C:%d, N:%d, O:%d,", _ALUSystem.ALUOut, _ALUSystem.ALUOutFlag, _ALUSystem.ALUOutFlag[3],_ALUSystem.ALUOutFlag[2],_ALUSystem.ALUOutFlag[1],_ALUSystem.ALUOutFlag[0]);
            $display("Address Register File: COut: %d, DOut (Address): %d", _ALUSystem.ARF_COut, _ALUSystem.Address);            
            $display("Memory Out: %d", _ALUSystem.MemoryOut);            
            $display("Instruction Register: IROut: %d", _ALUSystem.IROut);            
            $display("MuxAOut: %d, MuxBOut: %d, MuxCOut: %d", _ALUSystem.MuxAOut, _ALUSystem.MuxBOut, _ALUSystem.MuxCOut);
            
            // increment array index and read next testvector
            VectorNum = VectorNum + 1;
            if (TestVectors[VectorNum] === 35'bx)
            begin
                $display("%d tests completed.",
                VectorNum);
                $finish; // End simulation
            end
        end
endmodule

     