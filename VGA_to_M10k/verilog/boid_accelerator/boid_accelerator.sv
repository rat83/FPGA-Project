module boid_accelerator(
	input logic clk;
	input logic reset;
	
	output logic [31:0]  
		x,  y, 	// current position
		px, py, 	// previous position (for erasure)
		vx, vy; 	// velocity
	
);



	// speed update
	
	

endmodule

module d_reg
	#(
		parameter d_width = 32;
		parameter d_res_v = 0;
	)(
	input logic 	clk;
	input logic 	reset;
	input logic 	[d_width-1:0] d;
	output logic 	[d_width-1:0] q;
	);
	
	always @(posedge clk) begin
		if (reset) begin
			q <= d_res_v;
		end else begin
			q <= d;
		end
	end
	
endmodule

module fix15_mul(
	input logic 	[31:0] a, b;
	output logic	[31:0] q;
	);
	
	assign logic [63:0] temp = a * b;
	assign q = temp[47:15];
	
endmodule

module amax_bmin(
	input logic 	[31:0] a, b;
	output logic	[31:0] q;
	);
	
	logic [31:0] a_temp, b_temp;
	
	logic [31:0] alpha, beta;
	
	//absolute value
	always_comb begin
		if (a[31] == 1) begin
			a_temp = -a;
		end else begin
			a_temp = a;
		end
		
		if (b[31] == 1) begin
			b_temp = -b;
		end else begin
			b_temp = b;
		end		
		
		if (a > b) begin
			alpha = a;
			beta = b;
		end else begin
			alpha = b;
			beta = a;
		end
		
		q = alpha + (beta >> 1);
		
	end
	
	
endmodule
	
	
