module sgm_de1_soc_core #(
    parameter IMAGE_WIDTH  = 640,
    parameter IMAGE_HEIGHT = 480,
    parameter MAX_DISP     = 64,
    parameter P1           = 10,
    parameter P2           = 60,
    parameter PIXEL_WIDTH  = 8,
    parameter CENSUS_BITS  = 62,
    parameter COST_WIDTH   = 6,
    parameter AGG_WIDTH    = 8
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    enable,
    input  wire                    pixel_valid,
    input  wire                    sof_in,
    input  wire                    [PIXEL_WIDTH-1:0] left_pixel,
    input  wire                    [PIXEL_WIDTH-1:0] right_pixel,

    output wire                    [PIXEL_WIDTH-1:0] disparity,
    output wire                    disparity_sof,
    output wire                    disparity_valid
);

localparam CENSUS_WIDTH = CENSUS_BITS + 1;
localparam COST_BUS_W   = MAX_DISP * COST_WIDTH + 1;
localparam AGG_BUS_W    = MAX_DISP * AGG_WIDTH + 1;

wire [PIXEL_WIDTH:0] left_pixel_tagged;
wire [PIXEL_WIDTH:0] right_pixel_tagged;
wire [CENSUS_WIDTH-1:0] left_census;
wire [CENSUS_WIDTH-1:0] right_census;
wire [COST_BUS_W-1:0] matching_cost;
wire [AGG_BUS_W-1:0] aggregated_cost;
wire aggregated_valid;
wire [PIXEL_WIDTH:0] disparity_tagged;

assign left_pixel_tagged  = {sof_in, left_pixel};
assign right_pixel_tagged = {sof_in, right_pixel};

sgm_census_transform #(
    .IMAGE_WIDTH    (IMAGE_WIDTH),
    .IMAGE_HEIGHT   (IMAGE_HEIGHT),
    .BUSDATAWID    (PIXEL_WIDTH + 1),
    .LINEBUFFERLEN  (IMAGE_WIDTH),
    .OUTPUTDATAWID  (CENSUS_WIDTH)
) u_census_left (
    .clk       (clk),
    .en        (enable & pixel_valid),
    .data_in   (left_pixel_tagged),
    .sof_in    (1'b0),
    .eol_in    (1'b0),
    .sof_out   (),
    .eol_out   (),
    .data_out  (left_census),
    .rst_n     (rst_n)
);

sgm_census_transform #(
    .IMAGE_WIDTH    (IMAGE_WIDTH),
    .IMAGE_HEIGHT   (IMAGE_HEIGHT),
    .BUSDATAWID    (PIXEL_WIDTH + 1),
    .LINEBUFFERLEN  (IMAGE_WIDTH),
    .OUTPUTDATAWID  (CENSUS_WIDTH)
) u_census_right (
    .clk       (clk),
    .en        (enable & pixel_valid),
    .data_in   (right_pixel_tagged),
    .sof_in    (1'b0),
    .eol_in    (1'b0),
    .sof_out   (),
    .eol_out   (),
    .data_out  (right_census),
    .rst_n     (rst_n)
);

sgm_matching_cost #(
    .MAXDISPARITY       (MAX_DISP),
    .MATCHINGCOSTBITWID (COST_WIDTH),
    .INPUTDATAWID       (CENSUS_WIDTH),
    .OUTPUTDATAWID      (COST_BUS_W)
) u_matching_cost (
    .clk           (clk),
    .en            (enable & pixel_valid),
    .sof_in        (1'b0),
    .eol_in        (1'b0),
    .sof_out       (),
    .eol_out       (),
    .leftCTstream  (left_census),
    .rightCTstream (right_census),
    .MatchingCost  (matching_cost),
    .rst_n         (rst_n)
);

sgm_4path_aggregation #(
    .IMAGE_WIDTH    (IMAGE_WIDTH),
    .IMAGE_HEIGHT   (IMAGE_HEIGHT),
    .MAXDISPARITY   (MAX_DISP),
    .PENALTY1       (P1),
    .PENALTY2       (P2),
    .CPDI_WIDTH     (COST_WIDTH),
    .LPDI_WIDTH     (AGG_WIDTH),
    .INPUTDATAWID   (COST_BUS_W),
    .OUTPUTDATAWID  (AGG_BUS_W)
) u_sgm_4path (
    .clk       (clk),
    .en        (enable),
    .pixelEN   (pixel_valid),
    .sof_in    (1'b0),
    .eol_in    (1'b0),
    .sof_out   (),
    .eol_out   (),
    .CPDi_1D   (matching_cost),
    .LPDi      (aggregated_cost),
    .LPDi_valid(aggregated_valid),
    .rst_n     (rst_n)
);

sgm_wta #(
    .MAXDISPARITY  (MAX_DISP),
    .LPDI_WIDTH    (AGG_WIDTH),
    .INPUTDATAWID  (AGG_BUS_W),
    .OUTPUTDATAWID (PIXEL_WIDTH + 1),
    .IMAGE_WIDTH   (IMAGE_WIDTH),
    .IMAGE_HEIGHT  (IMAGE_HEIGHT)
) u_wta (
    .clk             (clk),
    .en              (enable),
    .pixelEN         (aggregated_valid),
    .LPDi            (aggregated_cost),
    .disparity       (disparity_tagged),
    .disparity_valid (disparity_valid),
    .rst_n           (rst_n)
);

assign disparity     = disparity_tagged[PIXEL_WIDTH-1:0];
assign disparity_sof = disparity_tagged[PIXEL_WIDTH];

endmodule
// 9x7 Census transform. Input/output carry SOF in the MSB.
// 9x7 Census transform with explicit zero-padding at image borders.
// The original shift-register window is allowed to contain wrapped/stale
// pixels at line/frame boundaries, but those values are never used when the
// corresponding neighbour coordinate is outside the image.
module sgm_census_transform
#(
    parameter IMAGE_WIDTH = 640,
    parameter IMAGE_HEIGHT = 480,
    parameter BUSDATAWID = 9,
    parameter LINEBUFFERLEN = 640,
    parameter OUTPUTDATAWID = 63
)(
    input  wire                     clk,
    input  wire                     en,
    input  wire [BUSDATAWID-1:0]    data_in,
    input  wire                     sof_in,
    input  wire                     eol_in,
    output wire                     sof_out,
    output wire                     eol_out,
    output wire [OUTPUTDATAWID-1:0] data_out,
    input  wire                     rst_n
);

wire [BUSDATAWID-1:0] matrixInput [0:6];
wire [BUSDATAWID-1:0] LineBufferInput [0:6];
wire [BUSDATAWID-1:0] LineBufferOutput [0:6];
reg [9:0] BlkRamAddressA_sclkrate, BlkRamAddressB_sclkrate;
wire [10:0] BlkRamAddressB_pre;
reg [BUSDATAWID-1:0] RegShiftArray9X7_sclkrate [0:6][0:8];
integer r, c, census_idx;

assign BlkRamAddressB_pre = BlkRamAddressA_sclkrate + 2;

always @(posedge clk) begin
    if (!rst_n) begin
        BlkRamAddressA_sclkrate <= 0;
        BlkRamAddressB_sclkrate <= 0;
    end else if (en) begin
        if (BlkRamAddressA_sclkrate == LINEBUFFERLEN-1)
            BlkRamAddressA_sclkrate <= 0;
        else
            BlkRamAddressA_sclkrate <= BlkRamAddressA_sclkrate + 1'b1;

        if (BlkRamAddressB_pre > LINEBUFFERLEN-1)
            BlkRamAddressB_sclkrate <= BlkRamAddressB_pre - LINEBUFFERLEN;
        else
            BlkRamAddressB_sclkrate <= BlkRamAddressB_pre[9:0];
    end
end

assign LineBufferInput[0] = data_in;
assign matrixInput[0] = data_in;

genvar k;
generate
    for (k = 1; k < 6; k = k + 1) begin : gen_linebuffer_input
        assign LineBufferInput[k] = LineBufferOutput[k-1];
    end
    for (k = 1; k < 7; k = k + 1) begin : gen_matrix_input
        assign matrixInput[k] = LineBufferOutput[k-1];
    end
    for (k = 0; k < 6; k = k + 1) begin : gen_linebuffer_inst
        sgm_census_linebuffer #(
            .DATA_WIDTH(BUSDATAWID),
            .DEPTH(LINEBUFFERLEN),
            .ADDR_WIDTH(10)
        ) uLineBuffer (
            .clka(clk), .ena(en), .wea(en),
            .addra(BlkRamAddressA_sclkrate), .dina(LineBufferInput[k]),
            .clkb(clk), .enb(en), .addrb(BlkRamAddressB_sclkrate),
            .doutb(LineBufferOutput[k])
        );
    end
endgenerate

// Sliding 7x9 window; data and SOF tag travel together as in the original RTL.
always @(posedge clk) begin
    if (!rst_n) begin
        for (r = 0; r < 7; r = r + 1) begin
            RegShiftArray9X7_sclkrate[r][0] <= 0;
            for (c = 1; c < 9; c = c + 1)
                RegShiftArray9X7_sclkrate[r][c] <= 0;
        end
    end else if (en) begin
        for (r = 0; r < 7; r = r + 1) begin
            RegShiftArray9X7_sclkrate[r][0] <= matrixInput[r];
            for (c = 1; c < 9; c = c + 1)
                RegShiftArray9X7_sclkrate[r][c] <= RegShiftArray9X7_sclkrate[r][c-1];
        end
    end
end

wire [7:0] CentralVal;
wire CentralBit_SOF;
reg [0:63] CompareValue_sclkrate;
reg CentralBit_SOF_dly_sclkrate, SofOut_reg_sclkrate;
reg [0:61] CensusTF_sclkrate;

assign CentralVal = RegShiftArray9X7_sclkrate[3][4][BUSDATAWID-2:0];
assign CentralBit_SOF = RegShiftArray9X7_sclkrate[3][4][BUSDATAWID-1];

// Coordinates of the central sample currently present in RegShiftArray[3][4].
// Anchor this counter to the *delayed central SOF*, not raw data_in SOF, so it
// remains aligned even while the line buffers/window are flushing.
reg [9:0] center_col_sclkrate;
reg [15:0] center_row_sclkrate;
reg center_tracking_sclkrate;
integer center_x_comb;
integer center_y_comb;

always @* begin
    if (CentralBit_SOF) begin
        center_x_comb = 0;
        center_y_comb = 0;
    end else begin
        center_x_comb = center_col_sclkrate;
        center_y_comb = center_row_sclkrate;
    end
end

always @(posedge clk) begin
    if (!rst_n) begin
        center_col_sclkrate <= 0;
        center_row_sclkrate <= 0;
        center_tracking_sclkrate <= 1'b0;
    end else if (en) begin
        if (CentralBit_SOF) begin
            center_tracking_sclkrate <= 1'b1;
            if (IMAGE_WIDTH > 1) begin
                center_col_sclkrate <= 1;
                center_row_sclkrate <= 0;
            end else begin
                center_col_sclkrate <= 0;
                center_row_sclkrate <= 1;
            end
        end else if (center_tracking_sclkrate) begin
            if (center_col_sclkrate == IMAGE_WIDTH-1) begin
                center_col_sclkrate <= 0;
                center_row_sclkrate <= center_row_sclkrate + 1'b1;
            end else begin
                center_col_sclkrate <= center_col_sclkrate + 1'b1;
            end
        end
    end
end

// Each comparison is explicitly padded with an intensity-zero neighbour when
// the neighbour coordinate falls outside the image. For a zero neighbour,
// Census(center > neighbour) is simply (CentralVal > 0).
generate
    genvar rr, cc;
    for (rr = 0; rr < 7; rr = rr + 1) begin : gen_census_row_compare
        for (cc = 0; cc < 9; cc = cc + 1) begin : gen_census_col_compare
            if ((rr == 3) && (cc == 4)) begin : gen_skip_center
                always @(posedge clk) begin
                    if (!rst_n) CompareValue_sclkrate[rr*9+cc] <= 1'b0;
                    else if (en) CompareValue_sclkrate[rr*9+cc] <= 1'b0;
                end
            end else begin : gen_compare
                always @(posedge clk) begin
                    if (!rst_n) begin
                        CompareValue_sclkrate[rr*9+cc] <= 1'b0;
                    end else if (en) begin
                        if (!center_tracking_sclkrate && !CentralBit_SOF) begin
                            CompareValue_sclkrate[rr*9+cc] <= 1'b0;
                        end else if ((center_y_comb < 0) ||
                                     (center_y_comb >= IMAGE_HEIGHT) ||
                                     (center_x_comb < 0) ||
                                     (center_x_comb >= IMAGE_WIDTH)) begin
                            CompareValue_sclkrate[rr*9+cc] <= 1'b0;
                        end else begin
                            if (((center_x_comb + (4-cc)) < 0) ||
                                ((center_x_comb + (4-cc)) >= IMAGE_WIDTH) ||
                                ((center_y_comb + (3-rr)) < 0) ||
                                ((center_y_comb + (3-rr)) >= IMAGE_HEIGHT)) begin
                                CompareValue_sclkrate[rr*9+cc] <= (CentralVal > 8'd0);
                            end else begin
                                CompareValue_sclkrate[rr*9+cc] <=
                                    (CentralVal > RegShiftArray9X7_sclkrate[rr][cc][BUSDATAWID-2:0]);
                            end
                        end
                    end
                end
            end
        end
    end
endgenerate

always @(posedge clk) begin
    if (!rst_n) begin
        CentralBit_SOF_dly_sclkrate <= 1'b0;
        SofOut_reg_sclkrate <= 1'b0;
        for (census_idx = 0; census_idx < 62; census_idx = census_idx + 1)
            CensusTF_sclkrate[census_idx] <= 1'b0;
    end else if (en) begin
        for (census_idx = 0; census_idx < 62; census_idx = census_idx + 1) begin
            if (census_idx < 31)
                CensusTF_sclkrate[census_idx] <= CompareValue_sclkrate[census_idx];
            else
                CensusTF_sclkrate[census_idx] <= CompareValue_sclkrate[census_idx+1];
        end
        CentralBit_SOF_dly_sclkrate <= CentralBit_SOF;
        SofOut_reg_sclkrate <= CentralBit_SOF_dly_sclkrate;
    end
end

assign data_out = {SofOut_reg_sclkrate, CensusTF_sclkrate};
assign sof_out = SofOut_reg_sclkrate;
assign eol_out = 1'b0;

endmodule
module sgm_census_linebuffer #(
    parameter DATA_WIDTH = 9,
    parameter DEPTH = 1024,
    parameter ADDR_WIDTH = 10
)(
    input  wire                  clka,
    input  wire                  ena,
    input  wire                  wea,
    input  wire [ADDR_WIDTH-1:0] addra,
    input  wire [DATA_WIDTH-1:0] dina,
    input  wire                  clkb,
    input  wire                  enb,
    input  wire [ADDR_WIDTH-1:0] addrb,
    output reg  [DATA_WIDTH-1:0] doutb
);

(* ramstyle = "M10K" *) reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

always @(posedge clka) begin
    if (ena && wea)
        mem[addra] <= dina;
end

always @(posedge clkb) begin
    if (enb)
        doutb <= mem[addrb];
end

endmodule
// Census Hamming matching cost for MAXDISPARITY candidates.
module sgm_matching_cost
#(
    parameter  MAXDISPARITY = 64 ,
    parameter  MATCHINGCOSTBITWID = 6 ,
    parameter  INPUTDATAWID = 63,
    parameter  OUTPUTDATAWID = 385

  )
(
    input wire                      clk                               ,
    input wire                      en                                ,
    input wire                      sof_in                            ,
    input wire                      eol_in                            ,
    output wire                     sof_out                           ,
    output wire                     eol_out                           ,
    input wire[INPUTDATAWID-1:0]    leftCTstream                      ,
    input wire[INPUTDATAWID-1:0]    rightCTstream                     ,
    output wire[OUTPUTDATAWID-1:0]  MatchingCost                      ,
    input  wire                     rst_n

);

reg[INPUTDATAWID-1:0] rightShiftReg_sclkrate [0:MAXDISPARITY-1] ;
reg[INPUTDATAWID-1:0] leftShiftReg_sclkrate [0:MAXDISPARITY-1] ;
wire [INPUTDATAWID-2:0] ComparedBase ;
integer i ;
integer DispIndx, AddPiplineIndx ;

wire SOF_atCompareBase ;

always@(posedge clk )
if(!rst_n)
begin
    rightShiftReg_sclkrate[0] <= 0;
    for(i=1;i<MAXDISPARITY;i=i+1)
    begin
        rightShiftReg_sclkrate[i] <= 0;
    end
end
else if(en) begin
    rightShiftReg_sclkrate[0] <= rightCTstream;
    for(i=1;i<MAXDISPARITY;i=i+1)
    begin
        rightShiftReg_sclkrate[i] <= rightShiftReg_sclkrate[i-1];
    end
end

always@(posedge clk )
if(!rst_n)
begin
    leftShiftReg_sclkrate[0] <= 0;
    for(i=1;i<MAXDISPARITY;i=i+1)
    begin
        leftShiftReg_sclkrate[i] <= 0;
    end
end
else if(en) begin
    leftShiftReg_sclkrate[0] <= leftCTstream;
    for(i=1;i<MAXDISPARITY;i=i+1)
    begin
        leftShiftReg_sclkrate[i] <= leftShiftReg_sclkrate[i-1];
    end
end

assign SOF_atCompareBase = leftShiftReg_sclkrate[0][62] ;
assign ComparedBase = leftShiftReg_sclkrate[0][61:0] ;

reg [INPUTDATAWID-2:0] LeftvsRightXorValue_sclkrate [0:MAXDISPARITY-1] ;
reg sof_dely_sclkrate [0:7] ;
always@(posedge clk)
if(en)
begin
    for (i = 0; i < MAXDISPARITY; i=i+1) begin
        LeftvsRightXorValue_sclkrate[i] <= ComparedBase ^ rightShiftReg_sclkrate[i][61:0] ;
    end
end

reg[1:0] PiplineAddStg0_sclkrate [0:MAXDISPARITY-1] [0:30] ;
reg[2:0] PiplineAddStg1_sclkrate [0:MAXDISPARITY-1] [0:15] ;
reg[3:0] PiplineAddStg2_sclkrate [0:MAXDISPARITY-1] [0:7]  ;
reg[4:0] PiplineAddStg3_sclkrate [0:MAXDISPARITY-1] [0:3]  ;
reg[5:0] PiplineAddStg4_sclkrate [0:MAXDISPARITY-1] [0:1]  ;
reg[5:0] PiplineAddOut_sclkrate [0:MAXDISPARITY-1]        ;

always@(posedge clk)
if(en)
begin : parallel_add
    sof_dely_sclkrate[0] <= SOF_atCompareBase ;

    for(DispIndx=0;DispIndx<MAXDISPARITY;DispIndx=DispIndx+1)
    begin
        for(AddPiplineIndx=0;AddPiplineIndx<31;AddPiplineIndx=AddPiplineIndx+1)
        begin
            PiplineAddStg0_sclkrate[DispIndx][AddPiplineIndx] <= LeftvsRightXorValue_sclkrate[DispIndx][AddPiplineIndx*2] + LeftvsRightXorValue_sclkrate[DispIndx][AddPiplineIndx*2 + 1];
        end
    end
    sof_dely_sclkrate[1] <= sof_dely_sclkrate[0] ;

    for(DispIndx=0;DispIndx<MAXDISPARITY;DispIndx=DispIndx+1)
    begin
        for(AddPiplineIndx=0;AddPiplineIndx<15;AddPiplineIndx=AddPiplineIndx+1)
        begin
            PiplineAddStg1_sclkrate[DispIndx][AddPiplineIndx] <= PiplineAddStg0_sclkrate[DispIndx][AddPiplineIndx*2] + PiplineAddStg0_sclkrate[DispIndx][AddPiplineIndx*2 + 1];
        end
        PiplineAddStg1_sclkrate[DispIndx][15] <=  PiplineAddStg0_sclkrate[DispIndx][30];
    end
    sof_dely_sclkrate[2] <= sof_dely_sclkrate[1] ;

    for(DispIndx=0;DispIndx<MAXDISPARITY;DispIndx=DispIndx+1)
    begin
        for(AddPiplineIndx=0;AddPiplineIndx<8;AddPiplineIndx=AddPiplineIndx+1)
        begin
            PiplineAddStg2_sclkrate[DispIndx][AddPiplineIndx] <= PiplineAddStg1_sclkrate[DispIndx][AddPiplineIndx*2] + PiplineAddStg1_sclkrate[DispIndx][AddPiplineIndx*2 + 1];
        end
    end
    sof_dely_sclkrate[3] <= sof_dely_sclkrate[2] ;

    for(DispIndx=0;DispIndx<MAXDISPARITY;DispIndx=DispIndx+1)
    begin
        for(AddPiplineIndx=0;AddPiplineIndx<4;AddPiplineIndx=AddPiplineIndx+1)
        begin
            PiplineAddStg3_sclkrate[DispIndx][AddPiplineIndx] <= PiplineAddStg2_sclkrate[DispIndx][AddPiplineIndx*2] + PiplineAddStg2_sclkrate[DispIndx][AddPiplineIndx*2 + 1];
        end
    end
    sof_dely_sclkrate[4] <= sof_dely_sclkrate[3] ;

    for(DispIndx=0;DispIndx<MAXDISPARITY;DispIndx=DispIndx+1)
    begin
        for(AddPiplineIndx=0;AddPiplineIndx<2;AddPiplineIndx=AddPiplineIndx+1)
        begin
            PiplineAddStg4_sclkrate[DispIndx][AddPiplineIndx] <= PiplineAddStg3_sclkrate[DispIndx][AddPiplineIndx*2] + PiplineAddStg3_sclkrate[DispIndx][AddPiplineIndx*2 + 1];
        end
    end
    sof_dely_sclkrate[5] <= sof_dely_sclkrate[4] ;

    for(DispIndx=0;DispIndx<MAXDISPARITY;DispIndx=DispIndx+1)
    begin
        for(AddPiplineIndx=0;AddPiplineIndx<1;AddPiplineIndx=AddPiplineIndx+1)
        begin
            PiplineAddOut_sclkrate[DispIndx] <= PiplineAddStg4_sclkrate[DispIndx][AddPiplineIndx*2] + PiplineAddStg4_sclkrate[DispIndx][AddPiplineIndx*2 + 1];
        end
    end
    sof_dely_sclkrate[6] <= sof_dely_sclkrate[5] ;
end

genvar k;
generate
for (k = 0; k < MAXDISPARITY; k=k+1) begin : gen_matching_cost_unpack
    assign MatchingCost[(k+1)*MATCHINGCOSTBITWID-1:k*MATCHINGCOSTBITWID] = PiplineAddOut_sclkrate[k] ;
end
endgenerate

assign MatchingCost[OUTPUTDATAWID-1]  =  sof_dely_sclkrate[6] ;
assign sof_out = sof_dely_sclkrate[6] ;



endmodule
module sgm_path_agg_ram #(
    parameter DATA_WIDTH = 512,
    parameter DEPTH = 1024,
    parameter ADDR_WIDTH = 10
)(
    input  wire                  clka,
    input  wire                  ena,
    input  wire                  wea,
    input  wire [ADDR_WIDTH-1:0] addra,
    input  wire [DATA_WIDTH-1:0] dina,
    input  wire                  clkb,
    input  wire                  enb,
    input  wire [ADDR_WIDTH-1:0] addrb,
    output reg  [DATA_WIDTH-1:0] doutb
);

(* ramstyle = "M10K" *) reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

always @(posedge clka) begin
    if (ena && wea)
        mem[addra] <= dina;
end

always @(posedge clkb) begin
    if (enb)
        doutb <= mem[addrb];
end

endmodule
// 4-path SGM aggregation: left, upper-left, up, upper-right.
// 4-path SGM aggregation, corrected reference RTL.
// The recurrence and min-reduction are combinational so that the minimum
// belongs to the same predecessor vector used by the recurrence.
// Previous-row path vectors are stored in alternating row banks to avoid
// overwriting row y-1 while row y is being processed.
// Resource-optimized 4-path SGM aggregation.
// Previous-row vectors are stored in synchronous dual-port RAMs so Quartus
// can infer M10K instead of implementing 6 x IMAGE_WIDTH x 512 flip-flops.
// This version adds one accepted-sample stage for the synchronous RAM read.
module sgm_4path_aggregation
#(
    parameter IMAGE_WIDTH    = 640,
    parameter IMAGE_HEIGHT   = 480,
    parameter MAXDISPARITY   = 64,
    parameter PENALTY1       = 10,
    parameter PENALTY2       = 60,
    parameter CPDI_WIDTH     = 6,
    parameter LPDI_WIDTH     = 8,
    parameter INPUTDATAWID   = 385,
    parameter OUTPUTDATAWID  = 513
)(
    input  wire                         clk,
    input  wire                         en,
    input  wire                         pixelEN,
    input  wire                         sof_in,
    input  wire                         eol_in,
    output wire                         sof_out,
    output wire                         eol_out,
    input  wire [INPUTDATAWID-1:0]      CPDi_1D,
    output wire [OUTPUTDATAWID-1:0]     LPDi,
    output wire                         LPDi_valid,
    input  wire                         rst_n
);
    localparam COST_VECTOR_W = MAXDISPARITY * LPDI_WIDTH;
    localparam COST_INPUT_W  = MAXDISPARITY * CPDI_WIDTH;
    localparam CAND_W        = LPDI_WIDTH + 2;
    localparam RAM_ADDR_W    = 10; // IMAGE_WIDTH=640 on the target design
    localparam [CAND_W-1:0] PEN1_WIDE = PENALTY1;
    localparam [CAND_W-1:0] PEN2_WIDE = PENALTY2;
    localparam [CAND_W-1:0] LPDI_MAX_WIDE = { { (CAND_W-LPDI_WIDTH){1'b0} }, {LPDI_WIDTH{1'b1}} };

    integer i_comb;
    integer d_comb;
    integer d_seq;

    // Input stream coordinates (in cost-vector sample space).
    reg [9:0] colCnt_sclkrate;
    reg [9:0] rowCnt_sclkrate;
    wire SOF_atPx_y_wire;
    wire accept_input;
    wire [9:0] effective_col;
    wire [9:0] effective_row;
    wire effective_row_odd;
    wire effective_row_zero;

    assign SOF_atPx_y_wire = CPDi_1D[INPUTDATAWID-1];
    assign accept_input = en && pixelEN;
    assign effective_col = SOF_atPx_y_wire ? 10'd0 : colCnt_sclkrate;
    assign effective_row = SOF_atPx_y_wire ? 10'd0 : rowCnt_sclkrate;
    assign effective_row_odd = effective_row[0];
    assign effective_row_zero = (effective_row == 0);

    // The synchronous RAM response belongs to this registered cost vector.
    reg pending_valid;
    reg pending_sof;
    reg [9:0] pending_col;
    reg [9:0] pending_row;
    reg pending_row_odd;
    reg pending_row_zero;
    reg [COST_INPUT_W-1:0] pending_cost_vector;
    wire process_pending;
    assign process_pending = en && pending_valid;

    wire reset_r0 = pending_sof || (pending_col == 0);
    wire reset_r1 = pending_sof || pending_row_zero || (pending_col == 0);
    wire reset_r2 = pending_sof || pending_row_zero;
    wire reset_r3 = pending_sof || pending_row_zero || (pending_col == IMAGE_WIDTH-1);

    // Cost vector and four path vectors, retained as named arrays for debug.
    wire [CPDI_WIDTH-1:0] CPDi [0:MAXDISPARITY-1];
    reg  [LPDI_WIDTH-1:0] prev0 [0:MAXDISPARITY-1];
    reg  [LPDI_WIDTH-1:0] prev1 [0:MAXDISPARITY-1];
    reg  [LPDI_WIDTH-1:0] prev2 [0:MAXDISPARITY-1];
    reg  [LPDI_WIDTH-1:0] prev3 [0:MAXDISPARITY-1];

    reg [LPDI_WIDTH-1:0] Lr0PXminus1_YDi_2DArray_Reg_sclkrate [0:MAXDISPARITY-1];
    reg [LPDI_WIDTH-1:0] Lr0PX_YDi_2DArray_BeforeRegister_wire [0:MAXDISPARITY-1];
    reg [LPDI_WIDTH-1:0] Lr1PX_YDi_2DArray_BeforeRegister_wire [0:MAXDISPARITY-1];
    reg [LPDI_WIDTH-1:0] Lr2PX_YDi_2DArray_BeforeRegister_wire [0:MAXDISPARITY-1];
    reg [LPDI_WIDTH-1:0] Lr3PX_YDi_2DArray_BeforeRegister_wire [0:MAXDISPARITY-1];

    wire [LPDI_WIDTH-1:0] Lr1PXminus1_Yminus1Di_2dArray_wire [0:MAXDISPARITY-1];
    wire [LPDI_WIDTH-1:0] Lr2PX_Yminus1Di_2dArray_wire [0:MAXDISPARITY-1];
    wire [LPDI_WIDTH-1:0] Lr3PXplus1_Yminus1Di_2dArray_wire [0:MAXDISPARITY-1];

    // Each RAM stores one path's vector for one row parity.
    wire [COST_VECTOR_W-1:0] row1_even_q, row1_odd_q;
    wire [COST_VECTOR_W-1:0] row2_even_q, row2_odd_q;
    wire [COST_VECTOR_W-1:0] row3_even_q, row3_odd_q;
    wire [COST_VECTOR_W-1:0] row1_previous_word = pending_row_odd ? row1_even_q : row1_odd_q;
    wire [COST_VECTOR_W-1:0] row2_previous_word = pending_row_odd ? row2_even_q : row2_odd_q;
    wire [COST_VECTOR_W-1:0] row3_previous_word = pending_row_odd ? row3_even_q : row3_odd_q;
    wire [COST_VECTOR_W-1:0] row1_write_word;
    wire [COST_VECTOR_W-1:0] row2_write_word;
    wire [COST_VECTOR_W-1:0] row3_write_word;

    wire [9:0] pending_addr = pending_col;
    wire [9:0] read_addr_r1 = (effective_col == 0) ? 10'd0 : effective_col - 10'd1;
    wire [9:0] read_addr_r2 = effective_col;
    wire [9:0] read_addr_r3 = (effective_col >= IMAGE_WIDTH-1) ? 10'd0 : effective_col + 10'd1;

    // For current odd rows, previous row is in the even bank; vice versa.
    wire read_r1_even = accept_input && effective_row_odd && !effective_row_zero && (effective_col > 0);
    wire read_r1_odd  = accept_input && !effective_row_odd && !effective_row_zero && (effective_col > 0);
    wire read_r2_even = accept_input && effective_row_odd && !effective_row_zero;
    wire read_r2_odd  = accept_input && !effective_row_odd && !effective_row_zero;
    wire read_r3_even = accept_input && effective_row_odd && !effective_row_zero && (effective_col < IMAGE_WIDTH-1);
    wire read_r3_odd  = accept_input && !effective_row_odd && !effective_row_zero && (effective_col < IMAGE_WIDTH-1);

    wire write_even = process_pending && !pending_row_odd;
    wire write_odd  = process_pending &&  pending_row_odd;

    sgm_path_agg_ram #(.DATA_WIDTH(COST_VECTOR_W), .DEPTH(IMAGE_WIDTH), .ADDR_WIDTH(RAM_ADDR_W)) u_r1_even (
        .clka(clk), .ena(1'b1), .wea(write_even), .addra(pending_addr), .dina(row1_write_word),
        .clkb(clk), .enb(read_r1_even), .addrb(read_addr_r1), .doutb(row1_even_q));
    sgm_path_agg_ram #(.DATA_WIDTH(COST_VECTOR_W), .DEPTH(IMAGE_WIDTH), .ADDR_WIDTH(RAM_ADDR_W)) u_r1_odd (
        .clka(clk), .ena(1'b1), .wea(write_odd), .addra(pending_addr), .dina(row1_write_word),
        .clkb(clk), .enb(read_r1_odd), .addrb(read_addr_r1), .doutb(row1_odd_q));

    sgm_path_agg_ram #(.DATA_WIDTH(COST_VECTOR_W), .DEPTH(IMAGE_WIDTH), .ADDR_WIDTH(RAM_ADDR_W)) u_r2_even (
        .clka(clk), .ena(1'b1), .wea(write_even), .addra(pending_addr), .dina(row2_write_word),
        .clkb(clk), .enb(read_r2_even), .addrb(read_addr_r2), .doutb(row2_even_q));
    sgm_path_agg_ram #(.DATA_WIDTH(COST_VECTOR_W), .DEPTH(IMAGE_WIDTH), .ADDR_WIDTH(RAM_ADDR_W)) u_r2_odd (
        .clka(clk), .ena(1'b1), .wea(write_odd), .addra(pending_addr), .dina(row2_write_word),
        .clkb(clk), .enb(read_r2_odd), .addrb(read_addr_r2), .doutb(row2_odd_q));

    sgm_path_agg_ram #(.DATA_WIDTH(COST_VECTOR_W), .DEPTH(IMAGE_WIDTH), .ADDR_WIDTH(RAM_ADDR_W)) u_r3_even (
        .clka(clk), .ena(1'b1), .wea(write_even), .addra(pending_addr), .dina(row3_write_word),
        .clkb(clk), .enb(read_r3_even), .addrb(read_addr_r3), .doutb(row3_even_q));
    sgm_path_agg_ram #(.DATA_WIDTH(COST_VECTOR_W), .DEPTH(IMAGE_WIDTH), .ADDR_WIDTH(RAM_ADDR_W)) u_r3_odd (
        .clka(clk), .ena(1'b1), .wea(write_odd), .addra(pending_addr), .dina(row3_write_word),
        .clkb(clk), .enb(read_r3_odd), .addrb(read_addr_r3), .doutb(row3_odd_q));

    // Timing optimization: split the four-path sum into two registered
    // partial sums. This removes two cascaded adders from the RAM-to-register
    // critical path while preserving the exact four-path sum and >>2 scaling.
    // Each partial sum is LPDI_WIDTH+1 bits (max 2*(2^LPDI_WIDTH-1)).
    reg [LPDI_WIDTH:0] agg_sum01_pipe [0:MAXDISPARITY-1];
    reg [LPDI_WIDTH:0] agg_sum23_pipe [0:MAXDISPARITY-1];
    wire [LPDI_WIDTH+1:0] agg_sum_final [0:MAXDISPARITY-1];
    reg agg_output_sof;
    reg agg_output_valid;
    integer agg_output_idx;

    // Expose path-state words and combine the registered partial sums.
    genvar k;
    generate
        for (k = 0; k < MAXDISPARITY; k = k + 1) begin : gen_sgm_ram_unpack
            assign CPDi[k] = pending_cost_vector[k*CPDI_WIDTH +: CPDI_WIDTH];
            assign Lr1PXminus1_Yminus1Di_2dArray_wire[k] = row1_previous_word[k*LPDI_WIDTH +: LPDI_WIDTH];
            assign Lr2PX_Yminus1Di_2dArray_wire[k] = row2_previous_word[k*LPDI_WIDTH +: LPDI_WIDTH];
            assign Lr3PXplus1_Yminus1Di_2dArray_wire[k] = row3_previous_word[k*LPDI_WIDTH +: LPDI_WIDTH];
            assign row1_write_word[(k+1)*LPDI_WIDTH-1:k*LPDI_WIDTH] = Lr1PX_YDi_2DArray_BeforeRegister_wire[k];
            assign row2_write_word[(k+1)*LPDI_WIDTH-1:k*LPDI_WIDTH] = Lr2PX_YDi_2DArray_BeforeRegister_wire[k];
            assign row3_write_word[(k+1)*LPDI_WIDTH-1:k*LPDI_WIDTH] = Lr3PX_YDi_2DArray_BeforeRegister_wire[k];
            assign agg_sum_final[k] = {1'b0, agg_sum01_pipe[k]} + {1'b0, agg_sum23_pipe[k]};
            assign LPDi[(k+1)*LPDI_WIDTH-1:k*LPDI_WIDTH] = agg_sum_final[k] >> 2;
        end
    endgenerate

    // Synchronous path-state update uses the completed RAM read for pending pixel.
    always @* begin
        for (i_comb = 0; i_comb < MAXDISPARITY; i_comb = i_comb + 1) begin
            prev0[i_comb] = Lr0PXminus1_YDi_2DArray_Reg_sclkrate[i_comb];
            prev1[i_comb] = Lr1PXminus1_Yminus1Di_2dArray_wire[i_comb];
            prev2[i_comb] = Lr2PX_Yminus1Di_2dArray_wire[i_comb];
            prev3[i_comb] = Lr3PXplus1_Yminus1Di_2dArray_wire[i_comb];
        end
    end

    // Balanced reduction trees for min(previous path vector).
    // MAXDISPARITY must be a power of two for this heap-style tree;
    // the current design uses MAXDISPARITY=64.
    wire [LPDI_WIDTH-1:0] min_prev0, min_prev1, min_prev2, min_prev3;
    wire [LPDI_WIDTH-1:0] min0_tree [1:(2*MAXDISPARITY)-1];
    wire [LPDI_WIDTH-1:0] min1_tree [1:(2*MAXDISPARITY)-1];
    wire [LPDI_WIDTH-1:0] min2_tree [1:(2*MAXDISPARITY)-1];
    wire [LPDI_WIDTH-1:0] min3_tree [1:(2*MAXDISPARITY)-1];
    reg [CAND_W-1:0] best_tmp, candidate_tmp, delta_tmp, total_tmp;

    genvar min_tree_idx;
    generate
        for (min_tree_idx = 0; min_tree_idx < MAXDISPARITY; min_tree_idx = min_tree_idx + 1) begin : gen_min_tree_leaves
            assign min0_tree[MAXDISPARITY + min_tree_idx] = prev0[min_tree_idx];
            assign min1_tree[MAXDISPARITY + min_tree_idx] = prev1[min_tree_idx];
            assign min2_tree[MAXDISPARITY + min_tree_idx] = prev2[min_tree_idx];
            assign min3_tree[MAXDISPARITY + min_tree_idx] = prev3[min_tree_idx];
        end
        for (min_tree_idx = 1; min_tree_idx < MAXDISPARITY; min_tree_idx = min_tree_idx + 1) begin : gen_min_tree_nodes
            assign min0_tree[min_tree_idx] = (min0_tree[2*min_tree_idx] < min0_tree[2*min_tree_idx+1]) ? min0_tree[2*min_tree_idx] : min0_tree[2*min_tree_idx+1];
            assign min1_tree[min_tree_idx] = (min1_tree[2*min_tree_idx] < min1_tree[2*min_tree_idx+1]) ? min1_tree[2*min_tree_idx] : min1_tree[2*min_tree_idx+1];
            assign min2_tree[min_tree_idx] = (min2_tree[2*min_tree_idx] < min2_tree[2*min_tree_idx+1]) ? min2_tree[2*min_tree_idx] : min2_tree[2*min_tree_idx+1];
            assign min3_tree[min_tree_idx] = (min3_tree[2*min_tree_idx] < min3_tree[2*min_tree_idx+1]) ? min3_tree[2*min_tree_idx] : min3_tree[2*min_tree_idx+1];
        end
    endgenerate

    assign min_prev0 = min0_tree[1];
    assign min_prev1 = min1_tree[1];
    assign min_prev2 = min2_tree[1];
    assign min_prev3 = min3_tree[1];

    function [LPDI_WIDTH-1:0] sgm_path_update;
        input [CPDI_WIDTH-1:0] cost;
        input [LPDI_WIDTH-1:0] same_prev;
        input [LPDI_WIDTH-1:0] lower_prev;
        input [LPDI_WIDTH-1:0] upper_prev;
        input [LPDI_WIDTH-1:0] min_prev;
        input lower_valid;
        input upper_valid;
        input reset_path;
        reg [CAND_W-1:0] best;
        reg [CAND_W-1:0] candidate;
        reg [CAND_W-1:0] delta;
        reg [CAND_W-1:0] total;
        begin
            best = {CAND_W{1'b0}};
            candidate = {CAND_W{1'b0}};
            delta = {CAND_W{1'b0}};
            total = {CAND_W{1'b0}};
            sgm_path_update = {LPDI_WIDTH{1'b0}};
            if (reset_path) begin
                sgm_path_update = {{(LPDI_WIDTH-CPDI_WIDTH){1'b0}}, cost};
            end else begin
                best = {{(CAND_W-LPDI_WIDTH){1'b0}}, same_prev};
                if (lower_valid) begin
                    candidate = {{(CAND_W-LPDI_WIDTH){1'b0}}, lower_prev} + PEN1_WIDE;
                    if (candidate < best) best = candidate;
                end
                if (upper_valid) begin
                    candidate = {{(CAND_W-LPDI_WIDTH){1'b0}}, upper_prev} + PEN1_WIDE;
                    if (candidate < best) best = candidate;
                end
                candidate = {{(CAND_W-LPDI_WIDTH){1'b0}}, min_prev} + PEN2_WIDE;
                if (candidate < best) best = candidate;
                delta = {{(CAND_W-LPDI_WIDTH){1'b0}}, min_prev};
                if (best >= delta) delta = best - delta;
                else delta = {CAND_W{1'b0}};
                total = {{(CAND_W-CPDI_WIDTH){1'b0}}, cost} + delta;
                if (total > LPDI_MAX_WIDE) sgm_path_update = {LPDI_WIDTH{1'b1}};
                else sgm_path_update = total[LPDI_WIDTH-1:0];
            end
        end
    endfunction

    // For recurrence neighbor candidates, ext[d+1] is prev[d].
    wire [LPDI_WIDTH-1:0] prev0_ext [0:MAXDISPARITY+1];
    wire [LPDI_WIDTH-1:0] prev1_ext [0:MAXDISPARITY+1];
    wire [LPDI_WIDTH-1:0] prev2_ext [0:MAXDISPARITY+1];
    wire [LPDI_WIDTH-1:0] prev3_ext [0:MAXDISPARITY+1];
    generate
        assign prev0_ext[0] = {LPDI_WIDTH{1'b0}};
        assign prev1_ext[0] = {LPDI_WIDTH{1'b0}};
        assign prev2_ext[0] = {LPDI_WIDTH{1'b0}};
        assign prev3_ext[0] = {LPDI_WIDTH{1'b0}};
        assign prev0_ext[MAXDISPARITY+1] = {LPDI_WIDTH{1'b0}};
        assign prev1_ext[MAXDISPARITY+1] = {LPDI_WIDTH{1'b0}};
        assign prev2_ext[MAXDISPARITY+1] = {LPDI_WIDTH{1'b0}};
        assign prev3_ext[MAXDISPARITY+1] = {LPDI_WIDTH{1'b0}};
        for (k = 0; k < MAXDISPARITY; k = k + 1) begin : gen_prev_ext
            assign prev0_ext[k+1] = prev0[k];
            assign prev1_ext[k+1] = prev1[k];
            assign prev2_ext[k+1] = prev2[k];
            assign prev3_ext[k+1] = prev3[k];
        end
    endgenerate

    // Balanced minimum trees above replace the former serial 64-value
    // comparator chains. The path recurrence and aggregate output are unchanged.
    always @* begin
        for (d_comb = 0; d_comb < MAXDISPARITY; d_comb = d_comb + 1) begin
            if (d_comb > pending_col) begin
                Lr0PX_YDi_2DArray_BeforeRegister_wire[d_comb] = {LPDI_WIDTH{1'b1}};
                Lr1PX_YDi_2DArray_BeforeRegister_wire[d_comb] = {LPDI_WIDTH{1'b1}};
                Lr2PX_YDi_2DArray_BeforeRegister_wire[d_comb] = {LPDI_WIDTH{1'b1}};
                Lr3PX_YDi_2DArray_BeforeRegister_wire[d_comb] = {LPDI_WIDTH{1'b1}};
            end else begin
                Lr0PX_YDi_2DArray_BeforeRegister_wire[d_comb] = sgm_path_update(
                    CPDi[d_comb], prev0_ext[d_comb+1], prev0_ext[d_comb], prev0_ext[d_comb+2],
                    min_prev0, (d_comb > 0), (d_comb < MAXDISPARITY-1), reset_r0);
                Lr1PX_YDi_2DArray_BeforeRegister_wire[d_comb] = sgm_path_update(
                    CPDi[d_comb], prev1_ext[d_comb+1], prev1_ext[d_comb], prev1_ext[d_comb+2],
                    min_prev1, (d_comb > 0), (d_comb < MAXDISPARITY-1), reset_r1);
                Lr2PX_YDi_2DArray_BeforeRegister_wire[d_comb] = sgm_path_update(
                    CPDi[d_comb], prev2_ext[d_comb+1], prev2_ext[d_comb], prev2_ext[d_comb+2],
                    min_prev2, (d_comb > 0), (d_comb < MAXDISPARITY-1), reset_r2);
                Lr3PX_YDi_2DArray_BeforeRegister_wire[d_comb] = sgm_path_update(
                    CPDi[d_comb], prev3_ext[d_comb+1], prev3_ext[d_comb], prev3_ext[d_comb+2],
                    min_prev3, (d_comb > 0), (d_comb < MAXDISPARITY-1), reset_r3);
            end
        end
    end

    // Register two independent pairs of path costs. The next combinational
    // stage adds these two 9-bit partial sums and applies the original >>2.
    // Sidebands are registered for the same sample as the partial sums.
    always @(posedge clk) begin
        if (!rst_n) begin
            agg_output_valid <= 1'b0;
            agg_output_sof   <= 1'b0;
            for (agg_output_idx = 0; agg_output_idx < MAXDISPARITY; agg_output_idx = agg_output_idx + 1) begin
                agg_sum01_pipe[agg_output_idx] <= {(LPDI_WIDTH+1){1'b0}};
                agg_sum23_pipe[agg_output_idx] <= {(LPDI_WIDTH+1){1'b0}};
            end
        end else begin
            agg_output_valid <= process_pending;
            if (process_pending) begin
                agg_output_sof <= pending_sof;
                for (agg_output_idx = 0; agg_output_idx < MAXDISPARITY; agg_output_idx = agg_output_idx + 1) begin
                    agg_sum01_pipe[agg_output_idx] <= {1'b0, Lr0PX_YDi_2DArray_BeforeRegister_wire[agg_output_idx]}
                                                    + {1'b0, Lr1PX_YDi_2DArray_BeforeRegister_wire[agg_output_idx]};
                    agg_sum23_pipe[agg_output_idx] <= {1'b0, Lr2PX_YDi_2DArray_BeforeRegister_wire[agg_output_idx]}
                                                    + {1'b0, Lr3PX_YDi_2DArray_BeforeRegister_wire[agg_output_idx]};
                end
            end else begin
                agg_output_sof <= 1'b0;
            end
        end
    end

    // SOF and valid are delayed with the registered aggregate vector.
    assign LPDi[OUTPUTDATAWID-1] = agg_output_sof;
    assign LPDi_valid = agg_output_valid;
    assign sof_out = agg_output_sof;
    assign eol_out = 1'b0;

    // Sequential pipeline and row/column tracker.
    always @(posedge clk) begin
        if (!rst_n) begin
            colCnt_sclkrate <= 10'd0;
            rowCnt_sclkrate <= 10'd0;
            pending_valid <= 1'b0;
            pending_sof <= 1'b0;
            pending_col <= 10'd0;
            pending_row <= 10'd0;
            pending_row_odd <= 1'b0;
            pending_row_zero <= 1'b1;
            pending_cost_vector <= {COST_INPUT_W{1'b0}};
            for (d_seq = 0; d_seq < MAXDISPARITY; d_seq = d_seq + 1)
                Lr0PXminus1_YDi_2DArray_Reg_sclkrate[d_seq] <= {LPDI_WIDTH{1'b0}};
        end else begin
            if (process_pending) begin
                for (d_seq = 0; d_seq < MAXDISPARITY; d_seq = d_seq + 1)
                    Lr0PXminus1_YDi_2DArray_Reg_sclkrate[d_seq] <= Lr0PX_YDi_2DArray_BeforeRegister_wire[d_seq];
            end

            if (accept_input) begin
                pending_valid <= 1'b1;
                pending_sof <= SOF_atPx_y_wire;
                pending_col <= effective_col;
                pending_row <= effective_row;
                pending_row_odd <= effective_row_odd;
                pending_row_zero <= effective_row_zero;
                pending_cost_vector <= CPDi_1D[COST_INPUT_W-1:0];

                if (SOF_atPx_y_wire) begin
                    colCnt_sclkrate <= 10'd1;
                    rowCnt_sclkrate <= 10'd0;
                end else if (colCnt_sclkrate == IMAGE_WIDTH-1) begin
                    colCnt_sclkrate <= 10'd0;
                    if (rowCnt_sclkrate == IMAGE_HEIGHT-1)
                        rowCnt_sclkrate <= 10'd0;
                    else
                        rowCnt_sclkrate <= rowCnt_sclkrate + 10'd1;
                end else begin
                    colCnt_sclkrate <= colCnt_sclkrate + 10'd1;
                end
            end else if (process_pending) begin
                pending_valid <= 1'b0;
                pending_sof <= 1'b0;
            end
        end
    end
endmodule


module sgm_wta
#(
    parameter  MAXDISPARITY = 64 ,
    parameter  LPDI_WIDTH = 8 ,
    parameter  INPUTDATAWID = 513,
    parameter  OUTPUTDATAWID = 9,
    parameter  IMAGE_WIDTH = 640,
    parameter  IMAGE_HEIGHT = 480
  )
(
    input wire                      clk                               ,
    input wire                      en                                ,
    input wire                      pixelEN                           ,
    input  wire[INPUTDATAWID-1:0]   LPDi                              ,
    output wire[OUTPUTDATAWID-1:0]  disparity                         ,
    output wire                     disparity_valid                   ,
    input  wire                     rst_n

);

wire [LPDI_WIDTH-1:0] LPDi_2dArray_wire [0:MAXDISPARITY-1] ;
wire SOF_atPxy_wire ;
reg SOF_atPxy_reg_dly_sclkrate [0:7] ;
integer i ;

// Per-cycle output-valid tracking. The WTA data pipeline advances every
// enabled clock, so validity must advance every enabled clock too. pixelEN
// marks whether the current LPDi sample is real; bubbles inject a zero valid
// bit instead of freezing the pipeline and repeating a prior valid output.
localparam FRAME_PIXELS = IMAGE_WIDTH * IMAGE_HEIGHT;
reg [5:0] valid_pipe;
reg frame_active;
reg [31:0] frame_pixel_count;

assign disparity_valid = valid_pipe[5];
assign SOF_atPxy_wire = LPDi[INPUTDATAWID-1] ;

always @(posedge clk) begin
    if (!rst_n) begin
        valid_pipe        <= 6'b0;
        frame_active      <= 1'b0;
        frame_pixel_count <= 32'd0;
    end else if (en) begin
        // Pipeline validity advances on every clock, not only valid samples.
        valid_pipe[5:1] <= valid_pipe[4:0];
        valid_pipe[0]   <= 1'b0;

        // Frame accounting advances only when aggregation presents a sample.
        if (pixelEN) begin
            if (SOF_atPxy_wire) begin
                valid_pipe[0]    <= 1'b1;
                frame_active     <= 1'b1;
                frame_pixel_count <= 32'd1;
            end else if (frame_active) begin
                valid_pipe[0] <= 1'b1;
                if (frame_pixel_count >= FRAME_PIXELS-1) begin
                    frame_active      <= 1'b0;
                    frame_pixel_count <= 32'd0;
                end else begin
                    frame_pixel_count <= frame_pixel_count + 1'b1;
                end
            end else begin
                valid_pipe[0] <= 1'b0;
                frame_pixel_count <= 32'd0;
            end
        end
        // When pixelEN==0, frame_active/count are intentionally held so a
        // bubble inside a frame does not terminate or restart frame tracking.
    end
end

genvar k ;
generate
    for (k = 0; k < MAXDISPARITY; k=k+1)
    begin : gen_wta_unpack
        assign LPDi_2dArray_wire[k] = LPDi[(k+1)*LPDI_WIDTH-1:k*LPDI_WIDTH];
    end
endgenerate

reg [LPDI_WIDTH-1:0] min_stg0_sclkrate [0:31] ;
reg [LPDI_WIDTH-1:0] min_stg1_sclkrate [0:15] ;
reg [LPDI_WIDTH-1:0] min_stg2_sclkrate [0:7] ;
reg [LPDI_WIDTH-1:0] min_stg3_sclkrate [0:3] ;
reg [LPDI_WIDTH-1:0] min_stg4_sclkrate [0:1] ;
reg [LPDI_WIDTH-1:0] min_stg5_sclkrate ;

reg [LPDI_WIDTH-1:0] min_Indx_stg0_sclkrate [0:31] ;
reg [LPDI_WIDTH-1:0] min_Indx_stg1_sclkrate [0:15] ;
reg [LPDI_WIDTH-1:0] min_Indx_stg2 [0:7] ;
reg [LPDI_WIDTH-1:0] min_Indx_stg3_sclkrate [0:3] ;
reg [LPDI_WIDTH-1:0] min_Indx_stg4_sclkrate [0:1] ;
reg [LPDI_WIDTH-1:0] min_Indx_stg5_sclkrate ;

always@(posedge clk)
if(en)
begin

    for(i=0;i<32;i=i+1)
    begin
        if(LPDi_2dArray_wire[2*i] > LPDi_2dArray_wire[2*i+1])
        begin
            min_stg0_sclkrate[i]      <= LPDi_2dArray_wire[2*i+1] ;
            min_Indx_stg0_sclkrate[i] <= 2*i+1 ;
        end
        else begin
            min_stg0_sclkrate[i]      <= LPDi_2dArray_wire[2*i] ;
            min_Indx_stg0_sclkrate[i] <= 2*i ;
        end
    end
    SOF_atPxy_reg_dly_sclkrate[0] <= SOF_atPxy_wire && pixelEN ;

    for(i=0;i<16;i=i+1)
    begin
        if(min_stg0_sclkrate[2*i] > min_stg0_sclkrate[2*i+1])
        begin
            min_Indx_stg1_sclkrate[i] <= min_Indx_stg0_sclkrate[2*i+1] ;
            min_stg1_sclkrate[i]      <= min_stg0_sclkrate[2*i+1] ;
        end
        else begin
            min_stg1_sclkrate[i]      <= min_stg0_sclkrate[2*i] ;
            min_Indx_stg1_sclkrate[i] <= min_Indx_stg0_sclkrate[2*i] ;
        end
    end
    SOF_atPxy_reg_dly_sclkrate[1] <= SOF_atPxy_reg_dly_sclkrate[0] ;

    for(i=0;i<8;i=i+1)
    begin
        if(min_stg1_sclkrate[2*i] > min_stg1_sclkrate[2*i+1])
        begin
            min_stg2_sclkrate[i]   <= min_stg1_sclkrate[2*i+1] ;
            min_Indx_stg2[i] <= min_Indx_stg1_sclkrate[2*i+1] ;
        end
        else begin
            min_stg2_sclkrate[i]   <= min_stg1_sclkrate[2*i] ;
            min_Indx_stg2[i] <= min_Indx_stg1_sclkrate[2*i] ;
        end
    end
    SOF_atPxy_reg_dly_sclkrate[2] <= SOF_atPxy_reg_dly_sclkrate[1] ;

    for(i=0;i<4;i=i+1)
    begin
        if(min_stg2_sclkrate[2*i] > min_stg2_sclkrate[2*i+1])
        begin
            min_Indx_stg3_sclkrate[i]   <= min_Indx_stg2[2*i+1] ;
            min_stg3_sclkrate[i]     <= min_stg2_sclkrate[2*i+1] ;
        end
        else begin
            min_stg3_sclkrate[i]    <= min_stg2_sclkrate[2*i] ;
            min_Indx_stg3_sclkrate[i]  <= min_Indx_stg2[2*i] ;
        end
    end
    SOF_atPxy_reg_dly_sclkrate[3] <= SOF_atPxy_reg_dly_sclkrate[2] ;

    for(i=0;i<2;i=i+1)
    begin
        if(min_stg3_sclkrate[2*i] > min_stg3_sclkrate[2*i+1])
        begin
            min_Indx_stg4_sclkrate[i] <= min_Indx_stg3_sclkrate[2*i+1] ;
            min_stg4_sclkrate[i] <= min_stg3_sclkrate[2*i+1] ;
        end
        else begin
            min_Indx_stg4_sclkrate[i] <= min_Indx_stg3_sclkrate[2*i] ;
            min_stg4_sclkrate[i] <= min_stg3_sclkrate[2*i] ;
        end
    end
    SOF_atPxy_reg_dly_sclkrate[4] <= SOF_atPxy_reg_dly_sclkrate[3] ;

    for(i=0;i<1;i=i+1)
    begin
        if(min_stg4_sclkrate[2*i] > min_stg4_sclkrate[2*i+1])
        begin
            min_Indx_stg5_sclkrate <= min_Indx_stg4_sclkrate[2*i+1] ;
            min_stg5_sclkrate  <= min_stg4_sclkrate[2*i+1] ;
        end
        else begin
            min_Indx_stg5_sclkrate <= min_Indx_stg4_sclkrate[2*i] ;
            min_stg5_sclkrate  <= min_stg4_sclkrate[2*i] ;
        end
    end
    SOF_atPxy_reg_dly_sclkrate[5] <= SOF_atPxy_reg_dly_sclkrate[4] ;
end

assign disparity[7:0] = min_Indx_stg5_sclkrate;
assign disparity[8]   = SOF_atPxy_reg_dly_sclkrate[5] ;

assign sof_out_wta = disparity[8] ;

endmodule
