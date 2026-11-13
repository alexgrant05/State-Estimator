// ============================================================================
// rig_link_arb: lets N frame sources share one rig_link_tx
//
//   [source 0] --start/src/type/len/byte-->  +--------------+
//   [source 1] --------------------------->  | rig_link_arb | --> [rig_link_tx] --> Pico
//   ...                                      |  round robin |
//   [source N-1] ------------------------->  +--------------+
//              <--done (to the granted source only)--
//
// Each port looks like rig_link_tx to its source: the source pulses start
// with src/type/len valid and holds them (and its payload) until its done.
// A start is remembered as a request, so sources never need to look at the
// link's busy. Requests are served one frame at a time, round robin from the
// port after the last one served: with every source busy, each gets every
// N-th frame, so a fast sensor can't starve a slow one.
//
// byte_idx from rig_link_tx goes to every source; only the granted source's
// byte is forwarded.
// ============================================================================

module rig_link_arb #(
    parameter int N = 4
) (
    input  logic                clk,
    input  logic                rst,

    // from / to the sources
    input  logic [N-1:0]        req_start,      // 1-cycle pulse per frame
    input  logic [N-1:0][3:0]   req_src,
    input  logic [N-1:0][7:0]   req_type,
    input  logic [N-1:0][5:0]   req_len,
    input  logic [N-1:0][7:0]   req_byte,
    output logic [N-1:0]        req_done,       // 1-cycle pulse, granted port only

    // to / from rig_link_tx
    output logic                tx_start,
    output logic [3:0]          tx_src,
    output logic [7:0]          tx_type,
    output logic [5:0]          tx_len,
    output logic [7:0]          tx_byte,
    input  logic                tx_done
);

    if (N < 1 || N > 16) begin : g_chk_n
        $error("rig_link_arb: N (%0d) must be 1..16", N);
    end

    localparam int IW = (N > 1) ? $clog2(N) : 1;

    logic [N-1:0]  pending;
    logic          active;          // a frame is on the link
    logic [IW-1:0] g;               // granted port
    logic [IW-1:0] rr;              // first port to look at next time

    // first pending port at or after rr
    logic [IW-1:0] pick;
    logic          any;
    always_comb begin
        pick = rr;
        any  = 1'b0;
        for (int i = N - 1; i >= 0; i--) begin
            int unsigned p;
            p = (int'(rr) + i) % N;
            if (pending[p]) begin
                pick = IW'(p);
                any  = 1'b1;
            end
        end
    end

    assign tx_src  = req_src[g];
    assign tx_type = req_type[g];
    assign tx_len  = req_len[g];
    assign tx_byte = req_byte[g];

    always_comb begin
        req_done = '0;
        if (active && tx_done) req_done[g] = 1'b1;
    end

    always_ff @(posedge clk) begin
        tx_start <= 1'b0;

        if (rst) begin
            pending <= '0;
            active  <= 1'b0;
            g       <= '0;
            rr      <= '0;
        end else begin
            if (active && tx_done) begin
                pending[g] <= 1'b0;
                active     <= 1'b0;
                rr         <= (int'(g) == N - 1) ? '0 : g + 1'b1;
            end else if (!active && any) begin
                g        <= pick;           // tx_src/type/len follow g, valid with tx_start
                tx_start <= 1'b1;
                active   <= 1'b1;
            end

            // after the clear above: a source can only restart after its done
            for (int i = 0; i < N; i++)
                if (req_start[i]) pending[i] <= 1'b1;
        end
    end

endmodule
