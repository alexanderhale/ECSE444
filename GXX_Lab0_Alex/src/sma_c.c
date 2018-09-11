#include <sma.h>

void sma_c(float* x, int* xf, int N, int D) {
	// for odd D
	if (D % 2 == 1) {
		int i;
		for (i = 0; i < N; i++) {
			float sum = x[i];
			
			int j;
			for (j = 1; j <= D/2; j++) {
				int upper_index = i + j;
				int lower_index = i - j;
				
				if (upper_index < N) {
					sum += x[upper_index];
				}
				
				if (lower_index > 0) {
					sum += x[lower_index];
				}
			}
			xf[i] = sum/D;
		}
	} else {
		// for even D
		int i;
		for (i = 0; i < N; i++) {
			float sum = x[i];
			
			// the lower index gets one more value than the upper
			if (i - D/2 > 0) {
				sum += x[i - D/2];
			}
		
			int j;
			for (j = 1; j < D/2; j++) {
				int upper_index = i + j;
				int lower_index = i - j;
				
				if (upper_index < N) {
					sum += x[upper_index];
				}
				
				if (lower_index > 0) {
					sum += x[lower_index];
				}
			}
			xf[i] = sum/D;
		}
	}
}