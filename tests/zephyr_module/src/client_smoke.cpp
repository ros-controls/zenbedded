#include <zenbedded/rcl/zenbedded_client.hpp>

#include <zenbedded_transport/generated/interface_data.h>

using Client = ZenbeddedClient<RawCodec<zenbedded_state_t>, RawCodec<zenbedded_command_t>>;

static Client client;

extern "C" int zenbedded_client_link_smoke()
{
  zenbedded_state_t state{};
  zenbedded_command_t command{};

  int result = client.init(1);
  client.write_state(state);
  client.read_command(command);
  client.destroy();
  return result;
}