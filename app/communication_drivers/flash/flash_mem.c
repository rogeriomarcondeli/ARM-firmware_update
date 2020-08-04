/******************************************************************************
 * Copyright (C) 2017 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file flash_mem.c
 * @brief Flash memory module.
 *
 * @author joao.rosa
 *
 * @date 14/07/2015
 *
 */

#include "flash_mem.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
//#include "inc/hw_ints.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
//#include "driverlib/interrupt.h"

#include <stdint.h>

//#include "set_pinout_udc_v2.0.h"
#include "board_drivers/hardware_def.h"


dataflash_t DataFlash;

// Memory Serial Number
uint64_t 	SerialNumber;



//**********************************************************************************************************************************
//        Fun��es de escrita e leitura de parametros da mem�ria FLASH
//**********************************************************************************************************************************
/*
// Faz a leitura do bit "BUSY" da mem�ria Flash para testar se o evento ja foi terminado
tBoolean
ReadFlashMemoryBusy(void)
{
	unsigned long int ulindex;
	uint8_t uChar = 0x01;
	while(SSIDataGetNonBlocking(SSI0_BASE, &ulindex))
	{
	}

	// Envia comando para ler o status register 1
	SSIDataPut(SSI0_BASE, 0x05);
	SSIDataPut(SSI0_BASE, 0x00); // Extrai o dado

	while(SSIBusy(SSI0_BASE))
	{
	}

	SSIDataGet(SSI0_BASE, &ulindex);
	SSIDataGet(SSI0_BASE, &ulindex);
	uChar &= ulindex;
	if(uChar)return(true);
	else return(false);
}

void
ReadFlashMemoryStsRegs(void)
{
	unsigned long int ulindex;
	while(SSIDataGetNonBlocking(SSI0_BASE, &ulindex))
	{
	}

	SSIDataPut(SSI0_BASE, 0x05);
	SSIDataPut(SSI0_BASE, 0x00);

	while(SSIBusy(SSI0_BASE))
		{
		}

	SSIDataPut(SSI0_BASE, 0x35);
	SSIDataPut(SSI0_BASE, 0x00);

	SSIDataGet(SSI0_BASE, &ulindex);
	SSIDataGet(SSI0_BASE, &ulindex);
	//ulDataRx[0] = ulindex;
	SSIDataGet(SSI0_BASE, &ulindex);
	SSIDataGet(SSI0_BASE, &ulindex);
	//ulDataRx[1] = ulindex;
}

void
WriteEnableFlash(void)
{
	SSIDataPut(SSI0_BASE, 0x06); // Envia comando de leitura de dados
}

void
WriteDisableFlash(void)
{
	SSIDataPut(SSI0_BASE, 0x04); // Envia comando de leitura de dados
}
*/

//**********************************************************************************************************************************

//**********************************************************************************************************************************
//        Fun��es de leitura de dados da mem�ria FLASH
//**********************************************************************************************************************************

// Faz a leitura do n�mero serial da mem�ria flash para usar como ID de rastreamento do cart�o de controle
// O SN � composto por 64bits, estes ser�o salvos em uma vari�vel de 64bits
void flash_mem_read_serial_number(void)
{
	unsigned long ulindex;
	uint8_t uChar;

	// Limpa buffer de entrada
	while(SSIDataGetNonBlocking(FLASH_MEM_BASE, &ulindex))
	{
	}

	// Envia comando de leitura do ID de f�brica da mem�ria Flash
	SSIDataPut(FLASH_MEM_BASE, 0x4B);

	// Bytes de Dummy
	SSIDataPut(FLASH_MEM_BASE, 0x00);
	SSIDataPut(FLASH_MEM_BASE, 0x00);
	SSIDataPut(FLASH_MEM_BASE, 0x00);
	SSIDataPut(FLASH_MEM_BASE, 0x00);



	// Extrai os 64bits do ID
	SSIDataPut(FLASH_MEM_BASE, 0x00);
	SSIDataPut(FLASH_MEM_BASE, 0x00);
	SSIDataPut(FLASH_MEM_BASE, 0x00);
	SSIDataPut(FLASH_MEM_BASE, 0x00);

	SSIDataGet(FLASH_MEM_BASE, &ulindex); // Limpa sugeira do comando enviado
	SSIDataGet(FLASH_MEM_BASE, &ulindex); // Limpa sugeira gerada pelos bytes de dummy

	SSIDataGet(FLASH_MEM_BASE, &ulindex); // Limpa sugeira gerada pelos bytes de dummy
	SSIDataGet(FLASH_MEM_BASE, &ulindex); // Limpa sugeira gerada pelos bytes de dummy
	SSIDataGet(FLASH_MEM_BASE, &ulindex); // Limpa sugeira gerada pelos bytes de dummy


	SSIDataPut(FLASH_MEM_BASE, 0x00);
	SSIDataPut(FLASH_MEM_BASE, 0x00);



	SSIDataPut(FLASH_MEM_BASE, 0x00);
	SSIDataPut(FLASH_MEM_BASE, 0x00);

	// Wait until SSI0 is done transferring all the data in the transmit FIFO.
	while(SSIBusy(FLASH_MEM_BASE))
	{
	}

	SSIDataGetNonBlocking(FLASH_MEM_BASE, &ulindex);
	uChar = ulindex;
	SerialNumber = uChar;

	SSIDataGetNonBlocking(FLASH_MEM_BASE, &ulindex);
	uChar = ulindex;
	SerialNumber = SerialNumber << 8;
	SerialNumber |= uChar;

	SSIDataGetNonBlocking(FLASH_MEM_BASE, &ulindex);
	uChar = ulindex;
	SerialNumber = SerialNumber << 8;
	SerialNumber |= uChar;

	SSIDataGetNonBlocking(FLASH_MEM_BASE, &ulindex);
	uChar = ulindex;
	SerialNumber = SerialNumber << 8;
	SerialNumber |= uChar;


	SSIDataGetNonBlocking(FLASH_MEM_BASE, &ulindex);
	uChar = ulindex;
	SerialNumber = SerialNumber << 8;
	SerialNumber |= uChar;

	SSIDataGetNonBlocking(FLASH_MEM_BASE, &ulindex);
	uChar = ulindex;
	SerialNumber = SerialNumber << 8;
	SerialNumber |= uChar;

	SSIDataGetNonBlocking(FLASH_MEM_BASE, &ulindex);
	uChar = ulindex;
	SerialNumber = SerialNumber << 8;
	SerialNumber |= uChar;

	SSIDataGetNonBlocking(FLASH_MEM_BASE, &ulindex);
	uChar = ulindex;
	SerialNumber = SerialNumber << 8;
	SerialNumber |= uChar;
}

uint64_t flash_device_id_read(void)
{
	return SerialNumber;
}

void
flash_mem_init(void)
{
	// Configuration SSI (ADCP)
	SSIConfigSetExpClk(FLASH_MEM_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED),
					   SSI_FRF_MOTO_MODE_3,
					   SSI_MODE_MASTER, 12500000, 8);

	// Enable the FLASH module
	SSIEnable(FLASH_MEM_BASE);

	flash_mem_read_serial_number();
}

/*
void
ReadFlashMemoryDataPage()
{
	uint8_t PagSec;
	uint16_t var = 0;
	unsigned long int ulindex;

	PagSec = DataFlash.Sector << 4;
	PagSec |= DataFlash.PageVector;

	while(SSIDataGetNonBlocking(SSI0_BASE, &ulindex))
	{
	}

	while(var < 256)
	{
		SSIDataPut(SSI0_BASE, 0x03); // Envia comando de leitura de dados
		SSIDataPut(SSI0_BASE, DataFlash.Block); // Envia MSB do endere�o
		SSIDataPut(SSI0_BASE, PagSec); // Envia MID do endere�o
		SSIDataPut(SSI0_BASE, var); // Envia o LSB do endere�o

		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endere�o
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endere�o
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endere�o
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endere�o

		// Limpa buffer
		SSIDataGet(SSI0_BASE, &ulindex);
		SSIDataGet(SSI0_BASE, &ulindex);
		SSIDataGet(SSI0_BASE, &ulindex);
		SSIDataGet(SSI0_BASE, &ulindex);

		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endere�o
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endere�o
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endere�o
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endere�o

		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;

		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endere�o
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endere�o
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endere�o
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endere�o

		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;

		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endere�o
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endere�o
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endere�o
		SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endere�o

		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;

		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
		SSIDataGet(SSI0_BASE, &ulindex);
		DataFlash.ucDataTx[var] = ulindex;
		var++;
	}
}


uint8_t
ReadFlashMemoryData(uint32_t End)
{
	uint8_t End2, End1, End0;
	unsigned long int ulindex;

	End0 = End;
	End1 = End >> 8;
	End2 = End >> 16;

	while(SSIDataGetNonBlocking(SSI0_BASE, &ulindex))
	{
	}

	SSIDataPut(SSI0_BASE, 0x03); // Envia comando de leitura de dados
	SSIDataPut(SSI0_BASE, End2); // Envia MSB do endere�o
	SSIDataPut(SSI0_BASE, End1); // Envia MID do endere�o
	SSIDataPut(SSI0_BASE, End0); // Envia o LSB do endere�o
	SSIDataPut(SSI0_BASE, 0x00); // Recebe o dado do endere�o

	SSIDataGet(SSI0_BASE, &ulindex);
	SSIDataGet(SSI0_BASE, &ulindex);
	SSIDataGet(SSI0_BASE, &ulindex);
	SSIDataGet(SSI0_BASE, &ulindex);
	SSIDataGet(SSI0_BASE, &ulindex);
	End0 = ulindex;

	return(End0);
}

void
ReadFlashMemoryDataBlock(void)
{
	ReadFlashMemoryDataPage();

	if(DataFlash.PageVector == 0x0F && DataFlash.Sector < 0x0F)
	{
		DataFlash.PageVector = 0x00;
		DataFlash.Sector++;
	}
	else if(DataFlash.PageVector == 0x0F && DataFlash.Sector == 0x0F)
	{
		DataFlash.PageVector = 0x00;
		DataFlash.Sector = 0x00;
		DataFlash.BlockReadFull = true; // Seta flag indicando que o bloco foi lido por completo
	}
	else
	{
		DataFlash.PageVector++; // Incrementa o vetor de p�gina
	}

}

// **********************************************************************************************************************************


// **********************************************************************************************************************************
//        Fun��es de escrita de dados na mem�ria FLASH
// **********************************************************************************************************************************

// Fun��o que escreve em uma p�gina completa da mem�ria flash
// Lembre-se, o setor deve ser apagado antes, caso contr�rio o dado n�o ser� gravado
void
WriteFlashMemoryDataPage(void)
{
	uint8_t PagSec, Cont;

	PagSec = DataFlash.Sector << 4;
	PagSec |= DataFlash.PageVector;

	// Testa se a mem�ria est� pronta para uma nova opera��o
	// Se a fun��o retornar "true" a mem�ria est� ocupada
	while(ReadFlashMemoryBusy());

	// Envia comando de "write enable"
	SSIDataPut(SSI0_BASE, 0x06);

	// Aguarda que todos os bytes sejam enviados
	while(SSIBusy(SSI0_BASE))
	{
	}

	// Testa se a mem�ria est� pronta para uma nova opera��o
	// Se a fun��o retornar "true" a mem�ria est� ocupada
	while(ReadFlashMemoryBusy());

	SSIDataPut(SSI0_BASE, 0x02); // Envia comando de escrita de dados
	SSIDataPut(SSI0_BASE, DataFlash.Block); // Envia MSB do endere�o, bloco
	SSIDataPut(SSI0_BASE, PagSec); // Envia MID do endere�o, sector e o page vector
	SSIDataPut(SSI0_BASE, 0x00); // Envia o LSB do endere�o, in�cio da p�gina

	// Envia os 256 bytes (1 p�gina da mem�ria)
	for(Cont = 0; Cont < 256; Cont++)
	{
		SSIDataPut(SSI0_BASE, DataFlash.ucDataRx[Cont] ); // Envia o dado do endere�o
	}

	// Aguarda que todos os bytes sejam enviados
	while(SSIBusy(SSI0_BASE))
	{
	}
}


// Fun��o que controla a escrita em um determinado Bloco de modo cont�nuo
// O incremento de pagina, setor e fim de bloco � feito automaticamente
void
WriteFlashMemoryDataBlock(void)
{
	WriteFlashMemoryDataPage();

	if(DataFlash.PageVector == 0x0F && DataFlash.Sector < 0x0F)
	{
		DataFlash.PageVector = 0x00;
		DataFlash.Sector++;
	}
	else if(DataFlash.PageVector == 0x0F && DataFlash.Sector == 0x0F)
	{
		DataFlash.PageVector = 0x00;
		DataFlash.Sector = 0x00;
		DataFlash.BlockWriteFull = true; // Seta flag indicando que o bloco foi escrito por completo
	}
	else
	{
		DataFlash.PageVector++; // Incrementa o vetor de p�gina
	}

}

// Fun��o que envia o comando para apagar um setor da mem�ria flash
void
EreaseFlashMemorySector(void)
{
	uint8_t Sec;
	Sec = DataFlash.Sector << 4;

	// Testa se a mem�ria est� pronta para uma nova opera��o
	// Se a fun��o retornar "true" a mem�ria est� ocupada
	while(ReadFlashMemoryBusy());

	// Envia comando de "write enable"
	SSIDataPut(SSI0_BASE, 0x06);

	// Aguarda que todos os bytes sejam enviados
	while(SSIBusy(SSI0_BASE))
	{
	}

	// Testa se a mem�ria est� pronta para uma nova opera��o
	// Se a fun��o retornar "true" a mem�ria est� ocupada
	while(ReadFlashMemoryBusy());

	SSIDataPut(SSI0_BASE, 0x20); // Envia comando de escrita de dados
	SSIDataPut(SSI0_BASE, DataFlash.Block); // Envia MSB do endere�o, bloco
	SSIDataPut(SSI0_BASE, Sec); // Envia MID do endere�o, setor
	SSIDataPut(SSI0_BASE, 0x00); // Envia o LSB do endere�o

	// Aguarda que todos os bytes sejam enviados
	while(SSIBusy(SSI0_BASE))
	{
	}

}

// Fun��o que envia o comando para apagar um bloco de 64K da mem�ria flash
// O endere�o do bloco a ser apagado � dado pela variavel DataFlash.Block
void
EreaseFlashMemoryBlock(void)
{
	// Testa se a mem�ria est� pronta para uma nova opera��o
	// Se a fun��o retornar "true" a mem�ria est� ocupada
	while(ReadFlashMemoryBusy());

	// Envia comando de "write enable"
	SSIDataPut(SSI0_BASE, 0x06);

	// Aguarda que todos os bytes sejam enviados
	while(SSIBusy(SSI0_BASE))
	{
	}

	// Testa se a mem�ria est� pronta para uma nova opera��o
	// Se a fun��o retornar "true" a mem�ria est� ocupada
	while(ReadFlashMemoryBusy());

	SSIDataPut(SSI0_BASE, 0xD8); // Envia comando de escrita de dados
	SSIDataPut(SSI0_BASE, DataFlash.Block); // Envia MSB do endere�o, o endere�o inicial do bloco
	SSIDataPut(SSI0_BASE, 0x00); // Envia MID do endere�o
	SSIDataPut(SSI0_BASE, 0x00); // Envia o LSB do endere�o

	// Aguarda que todos os bytes sejam enviados
	while(SSIBusy(SSI0_BASE))
	{
	}
}

// **********************************************************************************************************************************


// **********************************************************************************************************************************
//        Fun��es de escrita de gerenciamento da mem�ria FLASH
// **********************************************************************************************************************************

void
ManagFlashMemory(uint8_t Command)
{
	switch(Command)
	{
		case READ_BLOCK:
			ReadFlashMemoryDataBlock();
			break;
		case WRITE_BLOCK:
			WriteFlashMemoryDataBlock();
			break;
		case READ_BYTE:
			break;
		case READ_ID:
			ReadSerialNumberFlash();
			break;
		case EREASE_BLOCK:
			EreaseFlashMemoryBlock();
			break;
		case INIT_DATAFLASH:
			DataFlash.PageVector = 0x00; // Posicionamento da mem�ria
			DataFlash.Sector = 0x00; // Posicionamento da mem�ria
			DataFlash.Block = 0x00; // Posicionamento da mem�ria
			DataFlash.NewDataRx = false; // Flag de indica��o de novo dado no buffer Rx
			DataFlash.NewDataTx = false; // Flag de indica��o de novo dado no buffer Tx
			DataFlash.Cancel = false; // Flag de indica��o para cancelamento da rotina
			DataFlash.BlockReadFull = false; // Flag que indica que o Bloco foi escrito por completo
			DataFlash.BlockWriteFull = false; // Flag que indica que o Bloco foi escrito por completo
			break;
	}

}
*/
//**********************************************************************************************************************************

/*
 * Antes de executar qualquer rotina que altere o conteudo da mem�ria n�o vol�til, � necess�rio habilitar a escrita (Write Enable)
 * A escrita s� pode ser feita se a regi�o tiver sido apagada antes pela fun��o sector erase (apaga 4K de uma vez)
 *
 * ap�s qualquer edi��o do conteudo sendo necess�rio uma nova escrita, � necess�rio monitorar o BUSY da FLASH
 *
 * Ao salvar os dados na flash � necess�rio criar uma logistica de aloca��o dos dados devido a dificuldade de apagar pontualmente o dado.
 *
 */

