#include "vmsim.h";

int main()
{
	vmsim_addr_t v1 = swap_out();
	vmsim_addr_t v2;
	swap_in(v2, v1);
	return 0;
}
