#include "modbus_rtu_slave.h"

//// Variaveis globais
// Pinos (possibilidade de comutação)
uint8_t slave_id = 1;
uint8_t re_de_pin = 2;

// Máximo tempo de espera (em ms) depois da mensagem terminar de ser recebida
uint16_t max_wait = 200;

// Variaveis uteis para o recebimento de mensagem
uint32_t tempo = 0; // "Timer" para saber se a msg acabou
uint8_t msg_rcv = 0; // Booleana para saber se a msg foi iniciada

struct{
    uint8_t id;
    uint8_t fc;
    uint16_t reg;
    uint16_t data;
    uint8_t ext1; // Posições extra da mensagem
    uint8_t ext2;
    uint8_t ext3;
    uint8_t ext4;
    uint8_t ext5;
    uint16_t crc; // CRC na ultima posição
}msg;

uint8_t *p_msg = &msg.id;

// Arrays dos registradores 
//     Mude o tamanho no cabeçalho para o valor desejado
uint8_t rg_outbits[REG_SIZE] = {0};
uint16_t rg_holding[REG_SIZE] = {0};


/*
 * Atualiza o valor de um coil
 * Parâmetros:
 *   - pos: posição relativa do registrador
 *   - val: valor para atualizar
 */
void modbus_update_coil(uint8_t pos, uint8_t val)
{
  pos %= REG_SIZE; // Força o valor a ser entre 0 e REG_SIZE
  
  rg_outbits[pos] = val;
}


/*
 * Atualiza o valor de um reg holding
 * Parâmetros:
 *   - pos: posição relativa do registrador
 *   - val: valor para atualizar
 */
void modbus_update_holding(uint8_t pos, uint16_t val)
{
  pos %= REG_SIZE; // Força o valor a ser entre 0 e REG_SIZE
  
  rg_holding[pos] = val;
}


/*
 * Configura a comunicação serial
 * Parâmetros:
 *   - baudrate: velocidade da comunicação
 *   - re_de: pino do re_de do RS485
 *   - slave_idd: id do escravo
 */
void modbus_init(uint16_t baudrate, uint8_t re_de, uint8_t slave_idd)
{
  // Comunicação serial
  if(!baudrate)
  {
    baudrate = 9600;
  }
  
  Serial.begin(baudrate);

  max_wait = (uint8_t)(20000.0/(float)baudrate); // 10bits/mensagem * 2 mensagens * 1000 ms

  slave_id = slave_idd;

  // Pino re_de_pin
  re_de_pin = re_de;
  pinMode(re_de_pin, OUTPUT);
  digitalWrite(re_de_pin, LOW); // Entra no modo de recepção
}


/*
 * Escuta o barramento e armazena a mensagem recebida em 'msg'
 */
void modbus_check()
{
  if(Serial.available())
  {
    // Armazena o caractere atual no struct mensagem e pula uma posição
    *(p_msg++) = Serial.read();

    // Reseta  o "timer" e indica que a mensagem foi iniciada
    tempo = millis();
    msg_rcv = true;
  }

  // Se está a 200ms sem receber nada e a mensagem foi iniciada
  if((millis() - tempo) > max_wait && msg_rcv)
  {
    // Armazena o CRC (última parte da mensagem)
    msg.crc = (*(p_msg-1) << 8) | *(p_msg-2);

    // Calcula o CRC esperado para ver se está correto
    uint8_t tam_msg = (uint8_t)(p_msg - &msg.id) - 2; // Tamanho da msg sem a parte do CRC
    uint16_t crc = CRC16(&msg.id, tam_msg);

    // Corrige os valores da mensagem, pois foi armazenado como se a arquitetura fosse Big Endian
    msg.reg = ((msg.reg & 0xFF)<<8) | (msg.reg>>8);
    msg.data = ((msg.data & 0xFF)<<8) | (msg.data>>8);

    // Força o valor a ser entre 0 e o valor máximo de registradores
    msg.reg %= (REG_SIZE+1);

    // Processa a mensagem se o CRC for igual
    if(crc == msg.crc)
    {
      modbus_process();
    }
    
    // Reseta os parâmetros
    p_msg = &msg.id;
    msg_rcv = false;
  }
}

/*
 * Processa a mensagem recebida
 * e envia de volta alguma resposta
 * caso seja necessário
 */
void modbus_process()
{
  // Verifica se é o alvo da mensagem
  if(msg.id == slave_id)
  {
    uint8_t msg_r[10 + msg.data*2] = {0}; // Cria a mensagem de resposta
    uint8_t tamanho = 3; // Tamanho final da mensagem

    // Inicio da resposta é padrao
    uint16_t registrador = msg.reg;
    msg_r[0] = msg.id;
    msg_r[1] = msg.fc;


    //// Verifica entre os diferentes códigos de função
    
    if(msg.fc == 1) // Leitura dos bits de saída
    {
      // Força o valor a ser entre 0 e o valor máximo de registradores
      msg.data %= (REG_SIZE+1);
    
      // Bytes a enviar na sequencia
      msg_r[2] = round(msg.data/8 + 0.5); // 0.5 para forçar arredondamento para cima

      for(uint8_t i = 0; i < msg_r[2]; i++){// Cria byte por byte
        uint8_t byte_atual = 0;

        // Cria bit por bit
        for(uint8_t j=0; j < 8 && (8*i + j) < msg.data; j++) 
        { 
          byte_atual |= rg_outbits[registrador] << j;
          registrador++;
        }
        
        msg_r[tamanho++] = byte_atual;
      }
    }
    
    else if (msg.fc == 3) // Leitura registrador holding
    {
      // Força o valor a ser entre 0 e o valor máximo de registradores
      msg.data %= (REG_SIZE+1);
       
      msg_r[2] = msg.data * 2; // Bytes a enviar na sequencia (reg_desejados * 2)
      for (uint8_t i = 0; i < msg.data; i++)
      {
        msg_r[tamanho++] = uint8_t(rg_holding[i + registrador] >> 8); // MSByte
        msg_r[tamanho++] = uint8_t(rg_holding[i + registrador] & 0xFF); // LSByte
      }
    }
    
    else if (msg.fc == 6) // Escrita registrador holding
    {
      msg_r[2] = uint8_t(registrador >> 8); // Endereço do registrador
      msg_r[tamanho++] = uint8_t(registrador & 0xFF);
      
      uint16_t val = msg.data; // Valor escrito
      msg_r[tamanho++] = uint8_t(val >> 8);
      msg_r[tamanho++] = uint8_t(val & 0xFF);

      modbus_update_holding(registrador, val); // Altera o valor do registrador
    }

    //// Calcula o CRC e envia a resposta
    uint16_t crc = CRC16(msg_r, tamanho);

    digitalWrite(re_de_pin, HIGH); // High envia dados, Low recebe dados

    for(uint8_t i = 0; i< tamanho; i++)
    {
      Serial.write(msg_r[i]);
    }
    Serial.write(crc&0xFF); // LSByte
    Serial.write(crc>>8); // MSByte
    Serial.flush();

    digitalWrite(re_de_pin, LOW); // Entra no modo de recepção
  }
}


// Calcula o CRC: Recebe a mensagem em formato de vetor e o tamanho da mensagem (quantos bytes)
uint16_t CRC16(uint8_t msg[], uint8_t len)
{  
  unsigned int crc = 0xFFFF;
  for (uint8_t pos = 0; pos < len; pos++)  {
    crc ^= msg[pos];    // Faz uma XOR entre o LSByte do CRC com o byte de dados atual
    
    for (uint8_t i = 8; i != 0; i--) {    // Itera sobre cada bit
      if ((crc & 0b1) != 0) {      // Se o LSB for 1:
        crc >>= 1;                  // Desloca para a direita 
        crc ^= 0xA001;              // E faz XOR com o polinômio 0xA001 (1010 0000 0000 0001 ): x16 + x15 + x2 + 1
      }else{                      // Senão:
        crc >>= 1;                  // Desloca para a direita 
      }
    }
  }

  // O formato retornado já sai invertido (LSByte primeiro que o MSByte)
  return crc;
}
