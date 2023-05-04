module MainIntegrator(clk, reset, outX, outY, outZ);
	input clk, reset;
	output signed [26:0] outX; 		//the state variable X
	output signed [26:0] outY; 		//the state variable Y
	output signed [26:0] outZ; 		//the state variable Z
	
	logic signed [26:0] functX, functY, functZ;
	logic signed [26:0] betaTz, sigTxy, xmuly, rhozTx;
	
	// disgusting localparam list. factored out for later use of EnResetReg
	localparam [26:0] initX = {7{1'b1},20{1'b0}};
	localparam [26:0] initY = {8'b1,19{1'b0}};
	localparam [26:0] initZ = {7'd25,20{1'b0}};
	localparam [26:0} dt 	= {7{1'b0},8'b1,12{1'b0}};
	localparam [26:0] rho 	= {7'd28,20{1'b0}};
	localparam [26:0] sigma = {7'd10,20{1'b0}};
	localparam [26:0] beta 	= {7'd2, 20{1'b0}};
	
	// these multipliers are essentially all setting up functX/Y/Z
	// they implement the circuit defined in the handout
	signed_mult mul1 (
	.out(rhozTx),
	.a(rho - outZ),
	.b(outX)
	);
	
	signed_mult mul2 (
	.out(xmuly),
	.a(outX), 
	.b(outY)
	);
	
	signed_mult mul3 (
	.out(betaTz),
	.a(outZ),
	.b(beta) 
	);
	
	signed_mult mul4 (
	.out(sigTxy),
	.a(outY - outX),
	.b(sigma)
	);
	
	// these multipliers are the final stage, feeding into the integrators
	signed_mult mulX (
	.out(functX)
	.a(sigTxy),
	.b(dt)
	);
	
	signed_mult mulY (
	.out(functY)
	.a(rhozTx - xmuly),
	.b(dt)
	);
	
	signed_mult mulZ (
	.out(functZ),
	.a(xmuly - betaTz),
	.b(dt)
	);
	
	// these integrators contain the state register and update the register
	integrator xk (
	.clk(clk),
	.reset(reset),
	.InitialOut(initX), // initial X
	.funct(functX),
	.out(outX)
	);
	
	integrator yk (
	.clk(clk),
	.reset(reset),
	.InitialOut(initY), // initial Y
	.funct(functY),
	.out(outY)
	);
	
	integrator zk (
	.clk(clk),
	.reset(reset),
	.InitialOut(initZ), // initial Z
	.funct(functZ),
	.out(outZ)
	);
	
endmodule


/////////////////////////////////////////////////
//// integrator /////////////////////////////////
/////////////////////////////////////////////////

module integrator(out,funct,InitialOut,clk,reset);
	output signed [26:0] out; 		//the state variable V
	input signed [26:0] funct;      //the dV/dt function
	input clk, reset;
	input signed [26:0] InitialOut;  //the initial state variable V
	
	wire signed	[26:0] out, v1new ;
	reg signed	[26:0] v1 ;
	
	always @ (posedge clk) 
	begin
		if (reset==0) //reset	
			v1 <= InitialOut ; // 
		else 
			v1 <= v1new ;	
	end
	assign v1new = v1 + funct ;
	assign out = v1 ;
endmodule

//////////////////////////////////////////////////
//// signed mult of 7.20 format 2'comp////////////
//////////////////////////////////////////////////

module signed_mult (out, a, b);
	output 	signed  	[26:0]	out;
	input 	signed	[26:0] 	a;
	input 	signed	[26:0] 	b;
	// intermediate full bit length
	wire 	signed	[53:0]	mult_out;
	assign mult_out = a * b;
	// select bits for 7.20 fixed point
	assign out = {mult_out[53], mult_out[45:20]};
endmodule


//////////////////////////////////////////////////
//// EnReg ///////////////////////////////////////
//////////////////////////////////////////////////

module EnReg (clk, d, q);
	input 	[26:0] 	d;
	output	[26:0]	q;
	input clk;
	
	always @(posedge clk) begin
		d <= q;
	end 
endmodule 