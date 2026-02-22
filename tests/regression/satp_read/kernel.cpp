#include <vx_intrinsics.h>
#include "common.h"

int main() {
	kernel_arg_t* arg = (kernel_arg_t*)csr_read(VX_CSR_MSCRATCH);

	uint64_t satp_value = csr_read(VX_CSR_SATP);

	volatile uint64_t* dst = (volatile uint64_t*)arg->dst_addr;
	*dst = satp_value;

	return 0;
}
