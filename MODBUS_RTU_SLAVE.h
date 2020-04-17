/*
 * Biblioteca Modbus RTU slave c/ RS485 
 * Desenvolvida por Fábio Guimarães
 */
#ifndef MODBUS_RTU_SLAVE_H
#define MODBUS_RTU_SLAVE_H

#define REG_SIZE 20

// Prototipo das funções
void modbus_update_coil(uint8_t pos, uint8_t val);
void modbus_update_holding(uint8_t pos, uint16_t val);

void modbus_init(uint16_t baudrate, uint8_t re_de, uint8_t slave_idd);
void modbus_check();
void modbus_process();
uint16_t CRC16(uint8_t msg[], uint8_t len);

#endif // MODBUS_RTU_SLAVE_H
