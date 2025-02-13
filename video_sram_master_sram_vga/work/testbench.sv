`timescale 1ns/1ns

module testbench();

logic clk;

always begin
	#10
	clk = !clk;
end

logic [3:0] num;
logic [6:0] segs;

HexDigit dut ( 
	.*
);

initial begin
	clk = 1'b0;
	num = 4'b0;
end

always_ff@(posedge clk) begin
	num <= num + 4'd1;
end

endmodule