//------------------------------------------------------------------------------
//Project Name      :   Algorithm_Framework_HDL
//Moudle Name       :   AlgoTrig.v
//Original Author   :   HDL-Auto
//Creation Date     :   2026.09.05
/*Description       :   正弦/余弦查找表（1024 点 ROM，组合输出）。
                        iAngle 16bit：0..65535 ↔ 0..2π；输出 signed Q15，满量程 ±1 ↔ ±32767。
                        例化后读 oSin/oCos；依赖 algo_sin_lut.mem（本目录，1024 行 hex）。
*/
//------------------------------------------------------------------------------
//Version           :   Rev 0.1
//modifier          :
//Modify Date       :
/*Modify Record     :
*/
//------------------------------------------------------------------------------
`include "algo_types.vh"

module AlgoTrig #(
    parameter integer LUT_ADDR = 10,   // 1024 点
    parameter integer A_W      = 16
) (
    input  wire [A_W-1:0]     iAngle,  // Q：全幅 2^16 ↔ 2π
    output reg  signed [15:0] oSin,
    output reg  signed [15:0] oCos
);
    reg signed [15:0] rom [0:1023];

    integer idx, sidx;

    initial $readmemh("../rtl/verilog/math/algo_sin_lut.mem", rom);   // vvp 在 build/ 下运行

    always @(*) begin
        // 用高 LUT_ADDR 位作索引；cos = sin(θ + π/2) 即 idx+256
        idx  = iAngle[A_W-1 -: LUT_ADDR];
        sidx = (idx + 256) & 1023;
        oSin = rom[idx];
        oCos = rom[sidx];
    end
endmodule
