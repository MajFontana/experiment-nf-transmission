module Carrier
#(
	parameter real CLOCK_FREQUENCY=100000000,
	parameter real TARGET_FREQUENCY=10000000
)
(
	input RST,
	input CLK,
    input ENABLE,
	input [1:0] VALUE,

	output TX
);

	parameter integer interval = CLOCK_FREQUENCY / TARGET_FREQUENCY / 2;
	


	reg [$clog2(interval) - 1:0] counter;
	wire strobe[3:0];
	
	assign strobe[0] = !counter;
	assign strobe[1] = counter == interval / 2;
	assign strobe[2] = counter == interval / 4;
	assign strobe[3] = counter == 3 * interval / 4;

	always @(posedge CLK or posedge RST) begin
		if (RST) begin
			counter <= interval - 1'd1;
		end
		else begin
			if (!counter) begin
				counter <= interval - 1'd1;
			end
			else begin
				counter <= counter - 1'd1;
			end
		end
	end
	


	reg toggle[3:0];
	
	assign TX = toggle[VALUE] & ENABLE;

	genvar i;
	generate
        for (i = 0; i < 4; i = i + 1) begin
            always @(posedge CLK or posedge RST) begin
                if (RST) begin
                    toggle[i] <= 1'd0;
                end
                else begin
                    if (strobe[i]) begin
                        toggle[i] <= ~toggle[i];
                    end
                end
            end
        end
	endgenerate

endmodule
