#include <ap_int.h>
#include <stdint.h>

// Whole A matrix
#define M   16384
#define K   16384

// Microtile
#define M_PE    16
#define K_PE    32
#define N_PE    8 // not used yet

// Macrotile
#define R_PE     8
#define C_PE     8
#define M_MACRO (M_PE * R_PE)
#define K_MACRO (K_PE * C_PE)

#define PORT_SIZE   128
using axi_word_t = ap_uint<PORT_SIZE>;

// K_PE is 32. If elements are bytes, K_PE bytes = 32 bytes.
// PORT_SIZE / 8 = 16 bytes per AXI word.
// 32 bytes / 16 bytes = 2 AXI words per row.
#define WORDS_PER_ROW (K_PE / (PORT_SIZE / 8)) // Exactly 2

void orchestrator(
    const axi_word_t *src0,
    const axi_word_t *src1,
    const axi_word_t *src2,
    const axi_word_t *src3,
    axi_word_t       *dst,
    uint32_t          num_words)
{
#pragma HLS INTERFACE m_axi port=src0 offset=slave bundle=gmem0 \
    max_read_burst_length=64 max_widen_bitwidth=PORT_SIZE
#pragma HLS INTERFACE m_axi port=src1 offset=slave bundle=gmem1 \
    max_read_burst_length=64 max_widen_bitwidth=PORT_SIZE
#pragma HLS INTERFACE m_axi port=src2 offset=slave bundle=gmem2 \
    max_read_burst_length=64 max_widen_bitwidth=PORT_SIZE
#pragma HLS INTERFACE m_axi port=src3 offset=slave bundle=gmem3 \
    max_read_burst_length=64 max_widen_bitwidth=PORT_SIZE
#pragma HLS INTERFACE m_axi port=dst  offset=slave bundle=gmem4 \
    max_write_burst_length=64 max_widen_bitwidth=PORT_SIZE

#pragma HLS INTERFACE s_axilite port=src0      bundle=control
#pragma HLS INTERFACE s_axilite port=src1      bundle=control
#pragma HLS INTERFACE s_axilite port=src2      bundle=control
#pragma HLS INTERFACE s_axilite port=src3      bundle=control
#pragma HLS INTERFACE s_axilite port=dst       bundle=control
#pragma HLS INTERFACE s_axilite port=num_words bundle=control
#pragma HLS INTERFACE s_axilite port=return    bundle=control

    static axi_word_t accum[C_PE][R_PE];

    // Iterate over the whole A matrix
    for (uint32_t j_outer = 0; j_outer < K; j_outer += K_MACRO) {
        for (uint32_t i_outer = 0; i_outer < M; i_outer += M_MACRO) {
            
            // Linear base tracking in terms of AXI words to keep hardware math clean
            uint32_t macrotile_base_bytes = i_outer * K + j_outer;

            // Loop over structural layout
            for (uint32_t i = 0; i < C_PE; ++i) {
                for (uint32_t j = 0; j < R_PE; ++j) {
                    
                    uint32_t microtile_base_bytes = macrotile_base_bytes + (j * K_PE * C_PE) + i;

                    for (uint32_t ii = 0; ii < M_PE; ii++) {
                        
                        // Calculate exact, un-divided start index in 128-bit words
                        uint32_t word_start_idx = (microtile_base_bytes + (ii * K_PE)) / (PORT_SIZE / 8);
                        
                        // Local cache to pipeline sequential reads and enforce a Burst of 2
                        axi_word_t read_buf[WORDS_PER_ROW];

                        // This loop reads 2 consecutive words sequentially, triggering an AXI Burst Length = 2
                        burst_read_loop: for (uint32_t jj = 0; jj < WORDS_PER_ROW; jj++) {
                            #pragma HLS PIPELINE II=1
                            uint32_t idx = word_start_idx + jj;
                            read_buf[jj] = (j == 0) ? src0[idx] :
                                           (j == 1) ? src1[idx] :
                                           (j == 2) ? src2[idx] :
                                                      src3[idx];
                        }//TODO: 

                        // Process the data out of local registers
                        accum_loop: for (uint32_t jj = 0; jj < WORDS_PER_ROW; jj++) {
                            #pragma HLS UNROLL
                            accum[i][j] += read_buf[jj];
                        }
                    }
                }
            }
        }
    }
}

