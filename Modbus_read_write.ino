#include "MODBUS_RTU_SLAVE.h"

#define RE_DE 8
#define SLAVE_ADDRS 1

void setup()
{
  // Inicializa o modbus
  modbus_init(9600, RE_DE, SLAVE_ADDRS);

  // Atualiza o 1º coil com o valor 1
  modbus_update_coil(0, 1);

  // Atualiza o 1º holding register com o valor 3000
  modbus_update_holding(0, 3000);
}

void loop()
{
  modbus_check();
}
