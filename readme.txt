Biblioteca MODBUS RTU para escravo via RS485
Os pinos 0 e 1 s�o utilizados para a comunica��o serial

A biblioteca possui resposta para os c�digos:
1 - Leitura bits de sa�da
3 - Leitura registrador holding
6 - Escrita unica registrador holding

�Para utilizar, basta inicializar a comunica��o:

void modbus_init(baudrate, pino re_de, endere�o do slave)

� A biblioteca inicializa os coils e os registradores holding com valor 0. Para alterar seus valores, basta utilizar:

modbus_update_coil(posi�ao, valor desejado);
modbus_update_holding(posi�ao, desejado);

� Para rodar a rotina de verifica��o, basta adicionar no loop principal:

modbus_check();

� Para obter qualquer dado da mensagem, utilize a estrutura msg:

msg.id; // ID da mensagem
msg.fc; // C�digo da fun��o;
msg.reg; // Endere�o do registrador requisitado
msg.data; // Dado recebido
msg.CRC; // CRC da mensagem