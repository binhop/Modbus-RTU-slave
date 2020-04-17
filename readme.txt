Biblioteca MODBUS RTU para escravo via RS485
Os pinos 0 e 1 são utilizados para a comunicação serial

A biblioteca possui resposta para os códigos:
1 - Leitura bits de saída
3 - Leitura registrador holding
6 - Escrita unica registrador holding

•Para utilizar, basta inicializar a comunicação:

void modbus_init(baudrate, pino re_de, endereço do slave)

• A biblioteca inicializa os coils e os holding registers com valor 0. Para alterar seus valores, basta utilizar:

modbus_update_coil(posiçao, valor desejado);
modbus_update_holding(posiçao, desejado);

• Para rodar a rotina de verificação, basta adicionar no loop principal:

modbus_check();

• Para obter qualquer dado da mensagem, utilize a estrutura msg:

msg.id; // ID da mensagem
msg.fc; // Código da função;
msg.reg; // Endereço do registrador requisitado
msg.data; // Dado recebido
msg.CRC; // CRC da mensagem
