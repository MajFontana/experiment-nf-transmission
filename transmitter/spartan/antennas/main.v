`timescale 10ns/1ns



module Main
(
	inout [7:0] jb,
	
	input clk,
	input [7:0] sw,
	input [5:0] btn,
	input uartrx,
	
	output [7:0] led,
	output uarttx
);
	
	//////// Declarations ////////
	
	// IO switches
	wire [7:0] SW;
	
	// IO buttons
	wire BTN_RST;
	wire BTN_UP;
	wire BTN_LEFT;
	wire BTN_DOWN;
	wire BTN_RIGHT;
	wire BTN_CTR;
	
	// IO LEDs
	wire [7:0] LED;
	
	// IO UART
	wire UART_RX;
	wire UART_TX;
	
	// IO PMOD
	wire [7:0] PMOD_IN;
	wire [7:0] PMOD_OUT;
	wire [7:0] PMOD_DIR;
	
	// Reset
	reg reset0 = 1; // Buffer for synchronous deassertion
	reg RST = 1; // Master reset signal
	wire reset_trigger; // Active high reset input
	
	// Clock
	wire CLK; // Master clock
	wire LOCKED; // Status - DCM locked
	
 
 
	//////// Hardware interface ////////
	
	// IO
	assign SW = sw;
	assign {BTN_CTR, BTN_RIGHT, BTN_DOWN, BTN_LEFT, BTN_UP, BTN_RST} = btn[5:0];
	assign led = LED;
	assign UART_RX = uartrx;
	assign uarttx = UART_TX;
	
	genvar i;
	generate
		for (i = 0; i < 8; i = i + 1) begin
			assign jb[i] = PMOD_DIR[i] ? PMOD_OUT[i] : 1'bz;
			assign PMOD_IN[i] = PMOD_DIR[i] ? 1'd0 : jb[i];
		end
	endgenerate
	
	// Reset
	assign reset_trigger = ~BTN_RST;
	
	always @(posedge CLK or posedge reset_trigger) begin
		if (reset_trigger | ~LOCKED) begin
			reset0 <= 1;
			RST <= 1;
		end
		else begin
			RST <= reset0;
			reset0 <= 0;
		end
	end
	
	// Clock
	assign CLK = clk;
	assign LOCKED = 1'd1;
	
	/*clk_wiz_v3_6 clock
   (	// Clock in ports
		.CLK_IN1(clk), // IN
		// Clock out ports
		.CLK_OUT1(CLK), // OUT
		// Status and control signals
		.RESET(RST), // IN
		.LOCKED(LOCKED) // OUT
	);*/
	
	
	
	//////// Logic ////////
	assign LED[7:0] = 0;
	assign PMOD_OUT[7:1] = 0;
	assign PMOD_DIR = 8'b00000001;
	assign UART_TX = 0;
	
	wire [127:0] signals;
	wire data;
	
	Carrier #(100000000, 4) car(RST, CLK, 1'd1, 2'b00, data);
	
	genvar bin;
	generate
		for (bin = 0; bin < 128; bin = bin + 1) begin
            Carrier #(100000000, (bin + 1) * 10000) car(RST, CLK, 1'd1, 2'b00, signals[bin]);
        end
	endgenerate
	
	assign PMOD_OUT[0] = ~BTN_CTR & ((~SW[7] & signals[SW[6:0]]) | (SW[7] & data & signals[SW[6:0]]));
	
endmodule
