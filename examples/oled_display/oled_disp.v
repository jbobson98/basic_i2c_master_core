module oled_disp (
    input wire  i_clk,
    input wire  i_rst,
    input wire  i_cmd_1,
    input wire  i_cmd_2,
    output wire o_oled_rst,
    output wire o_oled_rst_led,
    output wire [4:0] o_status_led,
    inout wire  i2c_sda,
    inout wire  i2c_scl
);

/* Top Level Parameters */
localparam CLK_FREQ         = 100_000_000;
localparam I2C_FAST_MODE_EN = 0;

/* Oled Display Commands */
localparam OLED_DEV_ADDR    = 8'h3C;    // [Pin4->GND: addr = 0x3C] [Pin4->VCC: addr = 0x3D]
localparam OLED_ALL_ON      = 8'hA5;
localparam OLED_DISP_ON     = 8'hAF;
localparam OLED_DISP_ON_DIM = 8'hAC;
localparam OLED_DISP_OFF    = 8'hAE;
localparam OLED_CMD_BYTE    = 8'h00;
//localparam OLED_DC_BYTE     = 8'h40; data byte

/* Debounce and synchronize buttons */
wire rst_sync, cmd_1_sync, cmd_2_sync;
debouncer #(.CLK_FREQ(CLK_FREQ)) reset_debouncer 
    (.clk(i_clk), .rst(1'b0), .btn_in(i_rst), .btn_out(rst_sync));
debouncer #(.CLK_FREQ(CLK_FREQ)) cmd_1_debouncer 
    (.clk(i_clk), .rst(1'b0), .btn_in(i_cmd_1), .btn_out(cmd_1_sync));
debouncer #(.CLK_FREQ(CLK_FREQ)) cmd_2_debouncer 
    (.clk(i_clk), .rst(1'b0), .btn_in(i_cmd_2), .btn_out(cmd_2_sync));

/* Oled Reset Timer */
localparam integer RESET_TIMER_MAX_COUNT = (0.01 * CLK_FREQ); // 10ms reset time
localparam integer RESET_TIMER_WIDTH     = $clog2(RESET_TIMER_MAX_COUNT + 1);
wire rst_cnt_reached;
reg rst_cnt_start = 1'b0;
flex_counter #(.MAX_COUNT(RESET_TIMER_MAX_COUNT), .WIDTH(RESET_TIMER_WIDTH)) reset_timer 
    (.clk(i_clk), .rst(i_rst), .cen(rst_cnt_start), .maxcnt(rst_cnt_reached), .count());

/* Instantiate the I2C Master */
wire [9:0] oled_addr;
assign oled_addr = OLED_DEV_ADDR;
reg [7:0] i2c_byte_cnt     = 8'b00000000;
reg [7:0] i2c_tx_data      = 8'b00000000;
reg [3:0] i2c_control_reg  = 4'b0000;
reg [3:0] i2c_mode_reg     = 4'b0000;
wire i2c_rx_data_valid, i2c_tx_data_needed;
wire [7:0] i2c_rx_data;
wire [4:0] i2c_status_reg;
i2c_master #(
    .SYS_CLOCK_FREQ_HZ(CLK_FREQ),
    .FAST_MODE_ENABLED(I2C_FAST_MODE_EN)
) i2c_master_core (
    .i_clk(i_clk),
    .i_rst(rst_sync),
    .i_slave_addr(oled_addr),
    .i_byte_cnt(i2c_byte_cnt),
    .i_control_reg(i2c_control_reg),
    .i_mode_reg(i2c_mode_reg),
    .i_tx_data(i2c_tx_data),
    .o_rx_data_valid(i2c_rx_data_valid),
    .o_tx_data_needed(i2c_tx_data_needed),
    .o_rx_data(i2c_rx_data),
    .o_status_reg(i2c_status_reg),
    .io_sda(i2c_sda),
    .io_scl(i2c_scl)
);

/* States */
reg [7:0] state;
localparam IDLE       = 8'd0;
localparam RESET      = 8'd1;
localparam SEND_CMD_1 = 8'd2;
localparam SEND_CMD_2 = 8'd3;
localparam SEND_CMD_3 = 8'd4;

/* Output Assignment */
assign o_oled_rst = ~(state == RESET);
assign o_oled_rst_led = ~(state == RESET);
assign o_status_led = i2c_status_reg;

always @(posedge i_clk or posedge rst_sync) begin

    if(rst_sync) begin
        state           <= IDLE;
        rst_cnt_start   <= 1'b0;
        i2c_tx_data     <= 0;
        i2c_mode_reg    <= 0;
        i2c_control_reg <= 0;
        i2c_byte_cnt    <= 0;
    end else begin
        case(state)


            IDLE: begin
                if(cmd_2_sync) begin
                    state <= RESET;
                    rst_cnt_start <= 1'b1;
                end else if(cmd_1_sync) begin
                    i2c_tx_data     <= OLED_CMD_BYTE;
                    i2c_byte_cnt    <= 4;
                    i2c_mode_reg    <= 4'b0000;
                    i2c_control_reg <= 4'b1000;
                    state           <= SEND_CMD_1;
                end
            end

            SEND_CMD_1: begin
                i2c_control_reg <= 4'b0000;
                if(i2c_tx_data_needed) begin   // JMB TODO: might need another state in i2c_master after tx_data_need goes high
                    i2c_tx_data <= OLED_DISP_ON;
                    state <= SEND_CMD_2;
                end
            end

            SEND_CMD_2: begin
                if(i2c_tx_data_needed) begin
                    i2c_tx_data <= OLED_CMD_BYTE;
                    state <= SEND_CMD_3;
                end
            end

            SEND_CMD_3: begin
                i2c_control_reg <= 4'b0000;
                if(i2c_tx_data_needed) begin
                    i2c_tx_data <= OLED_ALL_ON;
                end

                if(~i2c_status_reg[4]) begin // tx done
                    state <= IDLE;
                end
            end


            RESET: begin
                if(rst_cnt_reached) begin
                    state <= IDLE;
                    rst_cnt_start <= 1'b0;
                end
            end

            default: state <= IDLE;
        endcase
    end
end

endmodule