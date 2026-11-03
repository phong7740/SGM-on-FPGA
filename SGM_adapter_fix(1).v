`timescale 1ns/1ps

// Backpressure-safe Avalon-ST adapter for sgm_de1_soc_core.
//
// Input:
//   in_data[15:8] = left grayscale pixel
//   in_data[7:0]  = right grayscale pixel
//
// Output:
//   out_data[7:0] = disparity
//
// Backpressure:
//   output_ready = 0 -> freeze the SGM pipeline and stop input acceptance.
//   output_ready = 1 -> allow the SGM pipeline to advance.
//
// Assumptions:
//   1. The SGM core propagates enable to all stateful pipeline stages.
//   2. Avalon-ST readyLatency = 0.
//   3. Each frame contains IMAGE_WIDTH * IMAGE_HEIGHT pixel pairs.
//   4. DRAIN_CYCLES is sufficient to flush the SGM pipeline.
module sgm_avalon_st_adapter #(
    parameter integer IMAGE_WIDTH  = 640,
    parameter integer IMAGE_HEIGHT = 480,
    parameter integer DRAIN_CYCLES = 8224
)(
    input  wire        clk,
    input  wire        rst_n,

    // Input Avalon-ST packet signals
    input  wire        in_startofpacket,
    input  wire        in_endofpacket,

    // Input pixel-pair stream
    input  wire [15:0] in_data,
    input  wire        in_valid,
    output wire        in_ready,

    // Ready from downstream Avalon-ST FIFO
    input  wire        output_ready,

    // Output disparity stream
    output wire [7:0]  out_data,
    output wire        out_valid,
    output wire        out_startofpacket,
    output wire        out_endofpacket
);

    localparam integer FRAME_PIXELS =
                         IMAGE_WIDTH * IMAGE_HEIGHT;

    localparam integer DRAIN_COUNT_WIDTH =
                         (DRAIN_CYCLES <= 1)
                         ? 1
                         : $clog2(DRAIN_CYCLES + 1);

    reg [31:0] input_pixel_count;
    reg        drain_active;
    reg [DRAIN_COUNT_WIDTH-1:0] drain_count;

    reg [31:0] output_pixel_count;


    // ---------------------------------------------------------
    // 1. Downstream backpressure
    // ---------------------------------------------------------

    // output_ready is mapped to the Avalon-ST ready signal
    // of the downstream FIFO.

    wire output_path_available;
    assign output_path_available = output_ready;

    // Stop accepting new real pixels during the drain phase
    // or whenever the downstream FIFO is not ready.

    assign in_ready =
            (!drain_active) && output_path_available;

    wire accept_real_pixel;
    assign accept_real_pixel = in_valid && in_ready;


    // ---------------------------------------------------------
    // 2. Pipeline drain
    // ---------------------------------------------------------

    // Inject zero-valued pixel pairs after accepting a complete
    // frame to flush delayed results through the SGM pipeline.

    wire inject_drain_pixel;
    assign inject_drain_pixel =
            drain_active && output_path_available;

    wire core_pixel_valid;
    assign core_pixel_valid =
            accept_real_pixel || inject_drain_pixel;


    // Start of frame for the SGM core.
    // Frame boundaries are determined by the configured
    // number of input pixel pairs.

    wire core_sof;
    assign core_sof =
            accept_real_pixel && (input_pixel_count == 0);


    // Freeze the SGM core during downstream backpressure.

    wire core_enable;
    assign core_enable = output_ready;


    // ---------------------------------------------------------
    // 3. SGM core
    // ---------------------------------------------------------

    wire [7:0] core_disparity;
    wire       core_disparity_sof;
    wire       core_disparity_valid;

    sgm_de1_soc_core #(
        .IMAGE_WIDTH  (IMAGE_WIDTH),
        .IMAGE_HEIGHT (IMAGE_HEIGHT),
        .MAX_DISP     (64),
        .P1           (10),
        .P2           (60),
        .PIXEL_WIDTH  (8),
        .CENSUS_BITS  (62),
        .COST_WIDTH   (6),
        .AGG_WIDTH    (8)
    ) u_sgm_core (
        .clk             (clk),
        .rst_n           (rst_n),
        .enable          (core_enable),
        .pixel_valid     (core_pixel_valid),
        .sof_in          (core_sof),
        .left_pixel      (in_data[15:8]),
        .right_pixel     (in_data[7:0]),
        .disparity       (core_disparity),
        .disparity_sof   (core_disparity_sof),
        .disparity_valid (core_disparity_valid)
    );


    // ---------------------------------------------------------
    // 4. Output Avalon-ST stream
    // ---------------------------------------------------------

    assign out_data  = core_disparity;
    assign out_valid = core_disparity_valid;

    assign out_startofpacket =
            core_disparity_valid && core_disparity_sof;

    // Mark the last disparity pixel of each frame.
    // The counter advances only when VALID and READY are high.

    assign out_endofpacket =
            core_disparity_valid &&
            (output_pixel_count == FRAME_PIXELS - 1);

    wire output_handshake;
    assign output_handshake = out_valid && output_ready;


    // ---------------------------------------------------------
    // 5. Input, drain and output counters
    // ---------------------------------------------------------

    always @(posedge clk) begin
        if (!rst_n) begin
            input_pixel_count  <= 32'd0;
            drain_active       <= 1'b0;
            drain_count        <= {DRAIN_COUNT_WIDTH{1'b0}};
            output_pixel_count <= 32'd0;
        end
        else begin

            // ---------------------------------------------
            // Count accepted real input pixel pairs
            // ---------------------------------------------

            if (accept_real_pixel) begin
                if (input_pixel_count == FRAME_PIXELS - 1) begin
                    input_pixel_count <= 32'd0;

                    // Begin flushing after the final real pixel
                    // of the current frame has been accepted.
                    drain_active <= 1'b1;
                    drain_count  <= {DRAIN_COUNT_WIDTH{1'b0}};
                end
                else begin
                    input_pixel_count <=
                            input_pixel_count + 1'b1;
                end
            end


            // ---------------------------------------------
            // Inject drain samples
            // ---------------------------------------------

            if (inject_drain_pixel) begin
                if (drain_count == DRAIN_CYCLES - 1) begin
                    drain_active <= 1'b0;
                    drain_count  <= {DRAIN_COUNT_WIDTH{1'b0}};
                end
                else begin
                    drain_count <= drain_count + 1'b1;
                end
            end


            // ---------------------------------------------
            // Count output disparity handshakes
            // ---------------------------------------------

            if (output_handshake) begin
                if (core_disparity_sof) begin
                    // First output pixel of a frame accepted.
                    output_pixel_count <= 32'd1;
                end
                else if (output_pixel_count >=
                         FRAME_PIXELS - 1) begin
                    // Frame completed.
                    output_pixel_count <= 32'd0;
                end
                else begin
                    output_pixel_count <=
                            output_pixel_count + 1'b1;
                end
            end

        end
    end

endmodule