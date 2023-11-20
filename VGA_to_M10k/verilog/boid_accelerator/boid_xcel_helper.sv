module d_reg
	#(
		parameter d_width = 32,
		parameter d_res_v = 0
	)(
	input logic 	clk,
	input logic 	reset,
	input logic 	[d_width-1:0] d,
	output logic 	[d_width-1:0] q
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
	input logic signed 	[31:0] a, b,
	output logic signed	[31:0] q
	);
	
	logic signed [63:0] temp;
	assign temp = a * b;
	assign q = temp >>> 15;
	
endmodule

module amax_bmin(
	input logic 	[31:0] a, b,
	output logic	[31:0] q
	);
	
	logic signed [31:0] 	a_temp, 	b_temp,
								a_out,	b_out;
	
	logic signed [31:0] alpha, beta;
	
	fix15_mul a_mul(
		.a(a),
		.b(32'hffff0000),
		.q(a_out)
		);
		
	fix15_mul b_mul(
		.a(32'hffff0000),
		.b(b),
		.q(b_out)
		);
	
	
	
	//absolute value
	always_comb begin
		if (a[31] == 1) begin
			a_temp = a_out + (32'b1);
		end else begin
			a_temp = a;
		end
		
		if (b[31] == 1) begin
			b_temp = b_out + (32'b1);
		end else begin
			b_temp = b;
		end		
		
		if (a > b) begin
			alpha = a_temp;
			beta = b_temp;
		end else begin
			alpha = b_temp;
			beta = a_temp;
		end
		
		q = alpha + (beta >>> 1);
		
	end
	
	
endmodule

// fall_edge_detector will pulse 1 cycle high when a falling edge is recorded (with 1 cycle delay)

module fall_edge_detector(
  input  logic clk, signal,
  output logic q
);

  logic signalPrev;

  always_ff @(posedge clk) begin
    signalPrev <= signal;
    q       <= (!signal && signalPrev);
  end

endmodule

// zero_pad_fix15 will sign extend the variable on fix_in to 32 bits, with 
// compile-time defined parameters to determine how it should be interpreted.
// on a [total_bit_width - 1:0] register, the variable to be sign extended
// will be on bits [ (16 + fix_whole_bit_width + lsb_offset) : lsb_offset]
// and the sign will be on bit 16 + fix_whole_bit_width + lsb_offset.
// Requirement: total_bit_width, as defined, must be equal to or greater than 
// 16 + fix_whole_bit_width + lsb_offset.

module zero_pad_fix15
#(	
	parameter fix_whole_bit_width = 16,
	parameter lsb_offset				= 0,
	parameter total_bit_width		= 32
)
(	
	input 	[fix_whole_bit_width - 1:0] fix_in,
	output	[31:0]						fix_out
);
	localparam input_fix_bit_width = fix_whole_bit_width;
	assign fix_out = 
		{{(32 - input_fix_bit_width + lsb_offset){fix_in[input_fix_bit_width - 1]}}, fix_in[input_fix_bit_width - 2:lsb_offset] };

endmodule

module lut_32_divider
(	
	input logic [5:0] lut_sel,
	output logic signed[31:0] div_val
);
	// Lookup table for values of lut_sel mapping to 1/lut_sel for values
	// from 1 to 31, and returning 0 for lut_sel = 0. This will lead to
	// the centering and matching factors being zeroed out if no neighboring boids are detected.
	
	always @(*) begin
		case(lut_sel)
			0: 	div_val = 32'b0;
			1: 	div_val = 32'd1 << 16;
			2: 	div_val = 32'h00004000;
			3: 	div_val = 32'h00002aaa;
			4: 	div_val = 32'h00002000;
			5: 	div_val = 32'h00001999;
			6: 	div_val = 32'h00001555;
			7: 	div_val = 32'h00001249;
			8: 	div_val = 32'h00001000;
			9: 	div_val = 32'h00000e38;
			10: 	div_val = 32'h00000ccc;
			11: 	div_val = 32'h00000ba2;
			12: 	div_val = 32'h00000aaa;
			13: 	div_val = 32'h000009d8;
			14: 	div_val = 32'h00000924;
			15: 	div_val = 32'h00000888;
			16: 	div_val = 32'h00000800;
			17: 	div_val = 32'h00000787;
			18: 	div_val = 32'h0000071c;
			19: 	div_val = 32'h000006bc;
			20: 	div_val = 32'h00000666;
			21: 	div_val = 32'h00000618;
			22: 	div_val = 32'h000005d1;
			23: 	div_val = 32'h00000590;
			24: 	div_val = 32'h00000555;
			25: 	div_val = 32'h0000051e;
			26: 	div_val = 32'h000004ec;
			27: 	div_val = 32'h000004bd;
			28: 	div_val = 32'h00000492;
			29: 	div_val = 32'h00000469;
			30: 	div_val = 32'h00000444;
			31: 	div_val = 32'h00000421;
			default: div_val = 32'b0;
		endcase
	end

endmodule