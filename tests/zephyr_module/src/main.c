#include <zephyr/kernel.h>

#ifdef CONFIG_ZENBEDDED_RCL
int zenbedded_client_link_smoke(void);
#endif

int main(void)
{
#ifdef CONFIG_ZENBEDDED_RCL
  return zenbedded_client_link_smoke();
#else
  return 0;
#endif
}