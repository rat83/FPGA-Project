`timescale 1ns/1ns

module testbench();
	// clock divider to slow system down for testing
	reg CLOCK_50;
	reg [4:0] count;
	
	initial begin
		CLOCK_50 = 1'b0;
		count = 4'b0;
	end
	
	always begin
		#10
		CLOCK_50  = !CLOCK_50;
	end
	
	// analog update divided clock
	always @ (posedge CLOCK_50) 
	begin
			  count <= count + 1; 
	end	
	
	assign AnalogClock = (count==0);		

	reg reset;
	
	wire signed [26:0] outX, outY, outZ;
	
	initial begin
		reset = 1'b0;
		#20
		reset = 1'b1;
	end

	MainIntegrator dut (
	.clk(count),
	.reset(reset),
	.outX(outX),
	.outY(outY),
	.outZ(outZ)
	);
endmodule
