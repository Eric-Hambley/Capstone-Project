/*
  mcp_can.cpp
  2012 Copyright (c) Seeed Technology Inc.  All right reserved.
  2017 Copyright (c) Cory J. Fowler  All Rights Reserved.

  Author: Loovee
  Contributor: Cory J. Fowler
  2017-09-25
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General CNF3Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-
  1301  USA
*/
#include "mcp_can.h"
#include <SPI.h>


#define spi_read() spibus->transfer(0x00)

/*********************************************************************************************************
** Function name:           mcp2515_reset
** Descriptions:            Performs a software reset
*********************************************************************************************************/
void MCP_CAN::mcp2515_reset(void)                                      
{
    //SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    MCP2515_SELECT();
    spibus->transfer(MCP_RESET);
    MCP2515_UNSELECT();
    //SPI.endTransaction();
    //delayMicroseconds(10);
	delay(5); // If the MCP2515 was in sleep mode when the reset command was issued then we need to wait a while for it to reset properly
}

/*********************************************************************************************************
** Function name:           mcp2515_readRegister
** Descriptions:            Read data register
*********************************************************************************************************/
uint8_t MCP_CAN::mcp2515_readRegister(const uint8_t address)                                                                     
{
    uint8_t ret;

    //SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    MCP2515_SELECT();
    spibus->transfer(MCP_READ);
    spibus->transfer(address);
    ret = spi_read();
    MCP2515_UNSELECT();
    //SPI.endTransaction();

    return ret;
}

/*********************************************************************************************************
** Function name:           mcp2515_readRegisterS
** Descriptions:            Reads sucessive data registers
*********************************************************************************************************/
void MCP_CAN::mcp2515_readRegisterS(const uint8_t address, uint8_t values[], const uint8_t n)
{
    uint8_t i;
    //SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    MCP2515_SELECT();
    spibus->transfer(MCP_READ);
    spibus->transfer(address);
    // mcp2515 has auto-increment of address-pointer
    for (i=0; i<n; i++) 
        values[i] = spi_read();

    MCP2515_UNSELECT();
    //SPI.endTransaction();
}

/*********************************************************************************************************
** Function name:           mcp2515_setRegister
** Descriptions:            Sets data register
*********************************************************************************************************/
void MCP_CAN::mcp2515_setRegister(const uint8_t address, const uint8_t value)
{
    //SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    MCP2515_SELECT();
    spibus->transfer(MCP_WRITE);
    spibus->transfer(address);
    spibus->transfer(value);
    MCP2515_UNSELECT();
    //SPI.endTransaction();
}

/*********************************************************************************************************
** Function name:           mcp2515_setRegisterS
** Descriptions:            Sets sucessive data registers
*********************************************************************************************************/
void MCP_CAN::mcp2515_setRegisterS(const uint8_t address, const uint8_t values[], const uint8_t n)
{
    uint8_t i;
    //SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    MCP2515_SELECT();
    spibus->transfer(MCP_WRITE);
    spibus->transfer(address);
       
    for (i=0; i<n; i++) 
        spibus->transfer(values[i]);
	
    MCP2515_UNSELECT();
    //SPI.endTransaction();
}

/*********************************************************************************************************
** Function name:           mcp2515_modifyRegister
** Descriptions:            Sets specific bits of a register
*********************************************************************************************************/
void MCP_CAN::mcp2515_modifyRegister(const uint8_t address, const uint8_t mask, const uint8_t data)
{
    //SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    MCP2515_SELECT();
    spibus->transfer(MCP_BITMOD);
    spibus->transfer(address);
    spibus->transfer(mask);
    spibus->transfer(data);
    MCP2515_UNSELECT();
    //SPI.endTransaction();
}

/*********************************************************************************************************
** Function name:           mcp2515_readStatus
** Descriptions:            Reads status register
*********************************************************************************************************/
uint8_t MCP_CAN::mcp2515_readStatus(void)                             
{
    uint8_t i;
    //SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    MCP2515_SELECT();
    spibus->transfer(MCP_READ_STATUS);
    i = spi_read();
    MCP2515_UNSELECT();
    //SPI.endTransaction();
    return i;
}

/*********************************************************************************************************
+** Function name:           setSleepWakeup
+** Descriptions:            Enable or disable the wake up interrupt (If disabled the MCP2515 will not be woken up by CAN bus activity)
+*********************************************************************************************************/
void MCP_CAN::setSleepWakeup(const uint8_t enable)
{
    mcp2515_modifyRegister(MCP_CANINTE, MCP_WAKIF, enable ? MCP_WAKIF : 0);
}

/*********************************************************************************************************
** Function name:           setMode
** Descriptions:            Sets control mode
*********************************************************************************************************/
uint8_t MCP_CAN::setMode(const uint8_t opMode)
{
    mcpMode = opMode;
    return mcp2515_setCANCTRL_Mode(mcpMode);
}

/*********************************************************************************************************
** Function name:           mcp2515_setCANCTRL_Mode
** Descriptions:            Set control mode
*********************************************************************************************************/
uint8_t MCP_CAN::mcp2515_setCANCTRL_Mode(const uint8_t newmode)
{
 /*    uint8_t i;

    mcp2515_modifyRegister(MCP_CANCTRL, MODE_MASK, newmode);

    i = mcp2515_readRegister(MCP_CANCTRL);
    i &= MODE_MASK;

    if ( i == newmode ) 
        return MCP2515_OK;

    return MCP2515_FAIL; */
	// If the chip is asleep and we want to change mode then a manual wake needs to be done
	// This is done by setting the wake up interrupt flag
	// This undocumented trick was found at https://github.com/mkleemann/can/blob/master/can_sleep_mcp2515.c
	if((mcp2515_readRegister(MCP_CANSTAT) & MODE_MASK) == MCP_SLEEP && newmode != MCP_SLEEP)
	{
		// Make sure wake interrupt is enabled
		byte wakeIntEnabled = (mcp2515_readRegister(MCP_CANINTE) & MCP_WAKIF);
		if(!wakeIntEnabled)
			mcp2515_modifyRegister(MCP_CANINTE, MCP_WAKIF, MCP_WAKIF);

		// Set wake flag (this does the actual waking up)
		mcp2515_modifyRegister(MCP_CANINTF, MCP_WAKIF, MCP_WAKIF);

		// Wait for the chip to exit SLEEP and enter LISTENONLY mode.

		// If the chip is not connected to a CAN bus (or the bus has no other powered nodes) it will sometimes trigger the wake interrupt as soon
		// as it's put to sleep, but it will stay in SLEEP mode instead of automatically switching to LISTENONLY mode.
		// In this situation the mode needs to be manually set to LISTENONLY.

		if(mcp2515_requestNewMode(MCP_LISTENONLY) != MCP2515_OK)
			return MCP2515_FAIL;

		// Turn wake interrupt back off if it was originally off
		if(!wakeIntEnabled)
			mcp2515_modifyRegister(MCP_CANINTE, MCP_WAKIF, 0);
	}

	// Clear wake flag
	mcp2515_modifyRegister(MCP_CANINTF, MCP_WAKIF, 0);
	
	return mcp2515_requestNewMode(newmode);
}

/*********************************************************************************************************
+** Function name:           mcp2515_requestNewMode
+** Descriptions:            Set control mode
+*********************************************************************************************************/
uint8_t MCP_CAN::mcp2515_requestNewMode(const uint8_t newmode)
{
	byte startTime = millis();

	// Spam new mode request and wait for the operation  to complete
	while(1)
	{
		// Request new mode
		// This is inside the loop as sometimes requesting the new mode once doesn't work (usually when attempting to sleep)
		mcp2515_modifyRegister(MCP_CANCTRL, MODE_MASK, newmode); 

		byte statReg = mcp2515_readRegister(MCP_CANSTAT);
		if((statReg & MODE_MASK) == newmode) // We're now in the new mode
			return MCP2515_OK;
		else if((byte)(millis() - startTime) > 200) // Wait no more than 200ms for the operation to complete
			return MCP2515_FAIL;
	}
}

/*********************************************************************************************************
** Function name:           mcp2515_configRate
** Descriptions:            Set baudrate
*********************************************************************************************************/
uint8_t MCP_CAN::mcp2515_configRate(const uint8_t canSpeed, const uint8_t canClock)            
{
    uint8_t set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canClock)
    {
        case (MCP_8MHZ):
        switch (canSpeed) 
        {
            case (CAN_5KBPS):                                               //   5KBPS                  
            cfg1 = MCP_8MHz_5kBPS_CFG1;
            cfg2 = MCP_8MHz_5kBPS_CFG2;
            cfg3 = MCP_8MHz_5kBPS_CFG3;
            break;

            case (CAN_10KBPS):                                              //  10KBPS                  
            cfg1 = MCP_8MHz_10kBPS_CFG1;
            cfg2 = MCP_8MHz_10kBPS_CFG2;
            cfg3 = MCP_8MHz_10kBPS_CFG3;
            break;

            case (CAN_20KBPS):                                              //  20KBPS                  
            cfg1 = MCP_8MHz_20kBPS_CFG1;
            cfg2 = MCP_8MHz_20kBPS_CFG2;
            cfg3 = MCP_8MHz_20kBPS_CFG3;
            break;

            case (CAN_31K25BPS):                                            //  31.25KBPS                  
            cfg1 = MCP_8MHz_31k25BPS_CFG1;
            cfg2 = MCP_8MHz_31k25BPS_CFG2;
            cfg3 = MCP_8MHz_31k25BPS_CFG3;
            break;

            case (CAN_33K3BPS):                                             //  33.33KBPS                  
            cfg1 = MCP_8MHz_33k3BPS_CFG1;
            cfg2 = MCP_8MHz_33k3BPS_CFG2;
            cfg3 = MCP_8MHz_33k3BPS_CFG3;
            break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_8MHz_40kBPS_CFG1;
            cfg2 = MCP_8MHz_40kBPS_CFG2;
            cfg3 = MCP_8MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_8MHz_50kBPS_CFG1;
            cfg2 = MCP_8MHz_50kBPS_CFG2;
            cfg3 = MCP_8MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_8MHz_80kBPS_CFG1;
            cfg2 = MCP_8MHz_80kBPS_CFG2;
            cfg3 = MCP_8MHz_80kBPS_CFG3;
            break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_8MHz_100kBPS_CFG1;
            cfg2 = MCP_8MHz_100kBPS_CFG2;
            cfg3 = MCP_8MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_8MHz_125kBPS_CFG1;
            cfg2 = MCP_8MHz_125kBPS_CFG2;
            cfg3 = MCP_8MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_8MHz_200kBPS_CFG1;
            cfg2 = MCP_8MHz_200kBPS_CFG2;
            cfg3 = MCP_8MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_8MHz_250kBPS_CFG1;
            cfg2 = MCP_8MHz_250kBPS_CFG2;
            cfg3 = MCP_8MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_8MHz_500kBPS_CFG1;
            cfg2 = MCP_8MHz_500kBPS_CFG2;
            cfg3 = MCP_8MHz_500kBPS_CFG3;
            break;
        
            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_8MHz_1000kBPS_CFG1;
            cfg2 = MCP_8MHz_1000kBPS_CFG2;
            cfg3 = MCP_8MHz_1000kBPS_CFG3;
            break;  

            default:
            set = 0;
	    return MCP2515_FAIL;
            break;
        }
        break;

        case (MCP_10MHZ):
        switch (canSpeed) 
        {
            case (CAN_250KBPS):                                               //   5Kbps
            cfg1 = MCP_10MHz_250kBPS_CFG1;
            cfg2 = MCP_10MHz_250kBPS_CFG2;
            cfg3 = MCP_10MHz_250kBPS_CFG3;
            break;

         	default:
            set = 0;
	    return MCP2515_FAIL;
            break;
        }
        break;

        case (MCP_16MHZ):
        switch (canSpeed) 
        {
            case (CAN_5KBPS):                                               //   5Kbps
            cfg1 = MCP_16MHz_5kBPS_CFG1;
            cfg2 = MCP_16MHz_5kBPS_CFG2;
            cfg3 = MCP_16MHz_5kBPS_CFG3;
            break;

            case (CAN_10KBPS):                                              //  10Kbps
            cfg1 = MCP_16MHz_10kBPS_CFG1;
            cfg2 = MCP_16MHz_10kBPS_CFG2;
            cfg3 = MCP_16MHz_10kBPS_CFG3;
            break;

            case (CAN_20KBPS):                                              //  20Kbps
            cfg1 = MCP_16MHz_20kBPS_CFG1;
            cfg2 = MCP_16MHz_20kBPS_CFG2;
            cfg3 = MCP_16MHz_20kBPS_CFG3;
            break;

            case (CAN_33K3BPS):                                              //  20Kbps
            cfg1 = MCP_16MHz_33k3BPS_CFG1;
            cfg2 = MCP_16MHz_33k3BPS_CFG2;
            cfg3 = MCP_16MHz_33k3BPS_CFG3;
            break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_16MHz_40kBPS_CFG1;
            cfg2 = MCP_16MHz_40kBPS_CFG2;
            cfg3 = MCP_16MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg2 = MCP_16MHz_50kBPS_CFG2;
            cfg3 = MCP_16MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_16MHz_80kBPS_CFG1;
            cfg2 = MCP_16MHz_80kBPS_CFG2;
            cfg3 = MCP_16MHz_80kBPS_CFG3;
            break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_16MHz_100kBPS_CFG1;
            cfg2 = MCP_16MHz_100kBPS_CFG2;
            cfg3 = MCP_16MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_16MHz_125kBPS_CFG1;
            cfg2 = MCP_16MHz_125kBPS_CFG2;
            cfg3 = MCP_16MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_16MHz_200kBPS_CFG1;
            cfg2 = MCP_16MHz_200kBPS_CFG2;
            cfg3 = MCP_16MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_16MHz_250kBPS_CFG1;
            cfg2 = MCP_16MHz_250kBPS_CFG2;
            cfg3 = MCP_16MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_16MHz_500kBPS_CFG1;
            cfg2 = MCP_16MHz_500kBPS_CFG2;
            cfg3 = MCP_16MHz_500kBPS_CFG3;
            break;
        
            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_16MHz_1000kBPS_CFG1;
            cfg2 = MCP_16MHz_1000kBPS_CFG2;
            cfg3 = MCP_16MHz_1000kBPS_CFG3;
            break;  

            default:
            set = 0;
	    return MCP2515_FAIL;
            break;
        }
        break;
        
        case (MCP_20MHZ):
        switch (canSpeed) 
        {
            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_20MHz_40kBPS_CFG1;
            cfg2 = MCP_20MHz_40kBPS_CFG2;
            cfg3 = MCP_20MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_20MHz_50kBPS_CFG1;
            cfg2 = MCP_20MHz_50kBPS_CFG2;
            cfg3 = MCP_20MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_20MHz_80kBPS_CFG1;
            cfg2 = MCP_20MHz_80kBPS_CFG2;
            cfg3 = MCP_20MHz_80kBPS_CFG3;
            break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_20MHz_100kBPS_CFG1;
            cfg2 = MCP_20MHz_100kBPS_CFG2;
            cfg3 = MCP_20MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_20MHz_125kBPS_CFG1;
            cfg2 = MCP_20MHz_125kBPS_CFG2;
            cfg3 = MCP_20MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_20MHz_200kBPS_CFG1;
            cfg2 = MCP_20MHz_200kBPS_CFG2;
            cfg3 = MCP_20MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_20MHz_250kBPS_CFG1;
            cfg2 = MCP_20MHz_250kBPS_CFG2;
            cfg3 = MCP_20MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_20MHz_500kBPS_CFG1;
            cfg2 = MCP_20MHz_500kBPS_CFG2;
            cfg3 = MCP_20MHz_500kBPS_CFG3;
            break;
        
            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_20MHz_1000kBPS_CFG1;
            cfg2 = MCP_20MHz_1000kBPS_CFG2;
            cfg3 = MCP_20MHz_1000kBPS_CFG3;
            break;  

            default:
            set = 0;
            return MCP2515_FAIL;
            break;
        }
        break;
        
        default:
        set = 0;
	return MCP2515_FAIL;
        break;
    }

    if (set) {
        mcp2515_setRegister(MCP_CNF1, cfg1);
        mcp2515_setRegister(MCP_CNF2, cfg2);
        mcp2515_setRegister(MCP_CNF3, cfg3);
        return MCP2515_OK;
    }
     
    return MCP2515_FAIL;
}

/*********************************************************************************************************
** Function name:           mcp2515_initCANBuffers
** Descriptions:            Initialize Buffers, Masks, and Filters
*********************************************************************************************************/
void MCP_CAN::mcp2515_initCANBuffers(void)
{
    uint8_t i, a1, a2, a3;
    
    uint8_t std = 0;               
    uint8_t ext = 1;
    uint32_t ulMask = 0x00, ulFilt = 0x00;


    mcp2515_write_mf(MCP_RXM0SIDH, ext, ulMask);			/*Set both masks to 0           */
    mcp2515_write_mf(MCP_RXM1SIDH, ext, ulMask);			/*Mask register ignores ext bit */
    
                                                                        /* Set all filters to 0         */
    mcp2515_write_mf(MCP_RXF0SIDH, ext, ulFilt);			/* RXB0: extended               */
    mcp2515_write_mf(MCP_RXF1SIDH, std, ulFilt);			/* RXB1: standard               */
    mcp2515_write_mf(MCP_RXF2SIDH, ext, ulFilt);			/* RXB2: extended               */
    mcp2515_write_mf(MCP_RXF3SIDH, std, ulFilt);			/* RXB3: standard               */
    mcp2515_write_mf(MCP_RXF4SIDH, ext, ulFilt);
    mcp2515_write_mf(MCP_RXF5SIDH, std, ulFilt);

                                                                        /* Clear, deactivate the three  */
                                                                        /* transmit buffers             */
                                                                        /* TXBnCTRL -> TXBnD7           */
    a1 = MCP_TXB0CTRL;
    a2 = MCP_TXB1CTRL;
    a3 = MCP_TXB2CTRL;
    for (i = 0; i < 14; i++) {                                          /* in-buffer loop               */
        mcp2515_setRegister(a1, 0);
        mcp2515_setRegister(a2, 0);
        mcp2515_setRegister(a3, 0);
        a1++;
        a2++;
        a3++;
    }
    mcp2515_setRegister(MCP_RXB0CTRL, 0);
    mcp2515_setRegister(MCP_RXB1CTRL, 0);
}

/*********************************************************************************************************
** Function name:           mcp2515_init
** Descriptions:            Initialize the controller
*********************************************************************************************************/
uint8_t MCP_CAN::mcp2515_init(const uint8_t canIDMode, const uint8_t canSpeed, const uint8_t canClock)
{

  uint8_t res;

    mcp2515_reset();
    
    mcpMode = MCP_LOOPBACK;

    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0)
    {
#if DEBUG_MODE
      ////serial.print("Entering Configuration Mode Failure...\r\n"); 
#endif
      return res;
    }
#if DEBUG_MODE
    ////serial.print("Entering Configuration Mode Successful!\r\n");
#endif

    // Set Baudrate
    if(mcp2515_configRate(canSpeed, canClock))
    {
#if DEBUG_MODE
      ////serial.print("Setting Baudrate Failure...\r\n");
#endif
      return res;
    }
#if DEBUG_MODE
    ////serial.print("Setting Baudrate Successful!\r\n");
#endif

    if ( res == MCP2515_OK ) {

                                                                        /* init canbuffers              */
        mcp2515_initCANBuffers();

                                                                        /* interrupt mode               */
        mcp2515_setRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF);

	//Sets BF pins as GPO
	mcp2515_setRegister(MCP_BFPCTRL,MCP_BxBFS_MASK | MCP_BxBFE_MASK);
	//Sets RTS pins as GPI
	mcp2515_setRegister(MCP_TXRTSCTRL,0x00);

        switch(canIDMode)
        {
            case (MCP_ANY):
            mcp2515_modifyRegister(MCP_RXB0CTRL,
            MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
            MCP_RXB_RX_ANY | MCP_RXB_BUKT_MASK);
            mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
            MCP_RXB_RX_ANY);
            break;
/*          The followingn two functions of the MCP2515 do not work, there is a bug in the silicon.
            case (MCP_STD): 
            mcp2515_modifyRegister(MCP_RXB0CTRL,
            MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
            MCP_RXB_RX_STD | MCP_RXB_BUKT_MASK );
            mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
            MCP_RXB_RX_STD);
            break;

            case (MCP_EXT): 
            mcp2515_modifyRegister(MCP_RXB0CTRL,
            MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
            MCP_RXB_RX_EXT | MCP_RXB_BUKT_MASK );
            mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
            MCP_RXB_RX_EXT);
            break;
*/
            case (MCP_STDEXT): 
            mcp2515_modifyRegister(MCP_RXB0CTRL,
            MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
            MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK );
            mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
            MCP_RXB_RX_STDEXT);
            break;
    
            default:
#if DEBUG_MODE        
            ////serial.print("`Setting ID Mode Failure...\r\n");
#endif           
            return MCP2515_FAIL;
            break;
}    


        res = mcp2515_setCANCTRL_Mode(mcpMode);                                                                
        if(res)
        {
#if DEBUG_MODE        
          ////serial.print("Returning to Previous Mode Failure...\r\n");
#endif           
          return res;
        }

    }
    return res;

}

/*********************************************************************************************************
** Function name:           mcp2515_write_id
** Descriptions:            Write CAN ID
*********************************************************************************************************/
void MCP_CAN::mcp2515_write_id( const uint8_t mcp_addr, const uint8_t ext, const uint32_t id )
{
    uint16_t canid;
    uint8_t tbufdata[4];

    canid = (uint16_t)(id & 0x0FFFF);

    if ( ext == 1) 
    {
        tbufdata[MCP_EID0] = (uint8_t) (canid & 0xFF);
        tbufdata[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (uint8_t) (canid & 0x03);
        tbufdata[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 5 );
    }
    else 
    {
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 3 );
        tbufdata[MCP_SIDL] = (uint8_t) ((canid & 0x07 ) << 5);
        tbufdata[MCP_EID0] = 0;
        tbufdata[MCP_EID8] = 0;
    }
    
    mcp2515_setRegisterS( mcp_addr, tbufdata, 4 );
}

/*********************************************************************************************************
** Function name:           mcp2515_write_mf
** Descriptions:            Write Masks and Filters
*********************************************************************************************************/
void MCP_CAN::mcp2515_write_mf( const uint8_t mcp_addr, const uint8_t ext, const uint32_t id )
{
    uint16_t canid;
    uint8_t tbufdata[4];

    canid = (uint16_t)(id & 0x0FFFF);

    if ( ext == 1) 
    {
        tbufdata[MCP_EID0] = (uint8_t) (canid & 0xFF);
        tbufdata[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (uint8_t) (canid & 0x03);
        tbufdata[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 5 );
    }
    else 
    {
        tbufdata[MCP_EID0] = (uint8_t) (canid & 0xFF);
        tbufdata[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (uint8_t) ((canid & 0x07) << 5);
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 3 );
    }
    
    mcp2515_setRegisterS( mcp_addr, tbufdata, 4 );
}

/*********************************************************************************************************
** Function name:           mcp2515_read_id
** Descriptions:            Read CAN ID
*********************************************************************************************************/
void MCP_CAN::mcp2515_read_id( const uint8_t mcp_addr, uint8_t* ext, uint32_t* id )
{
    uint8_t tbufdata[4];

    *ext = 0;
    *id = 0;

    mcp2515_readRegisterS( mcp_addr, tbufdata, 4 );

    *id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

    if ( (tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) ==  MCP_TXB_EXIDE_M ) 
    {
                                                                        /* extended id                  */
        *id = (*id<<2) + (tbufdata[MCP_SIDL] & 0x03);
        *id = (*id<<8) + tbufdata[MCP_EID8];
        *id = (*id<<8) + tbufdata[MCP_EID0];
        *ext = 1;
    }
}

/*********************************************************************************************************
** Function name:           mcp2515_write_canMsg
** Descriptions:            Write message
*********************************************************************************************************/
void MCP_CAN::mcp2515_write_canMsg( const uint8_t buffer_sidh_addr)
{
    uint8_t mcp_addr;
    mcp_addr = buffer_sidh_addr;
    mcp2515_setRegisterS(mcp_addr+5, m_nDta, m_nDlc );                  /* write data bytes             */
	
    if ( m_nRtr == 1)                                                   /* if RTR set bit in byte       */
        m_nDlc |= MCP_RTR_MASK;  

    mcp2515_setRegister((mcp_addr+4), m_nDlc );                         /* write the RTR and DLC        */
    mcp2515_write_id(mcp_addr, m_nExtFlg, m_nID );                      /* write CAN id                 */

}

/*********************************************************************************************************
** Function name:           mcp2515_read_canMsg
** Descriptions:            Read message
*********************************************************************************************************/
void MCP_CAN::mcp2515_read_canMsg( const uint8_t buffer_sidh_addr)        /* read can msg                 */
{
    uint8_t mcp_addr, ctrl;

    mcp_addr = buffer_sidh_addr;

    mcp2515_read_id( mcp_addr, &m_nExtFlg,&m_nID );

    ctrl = mcp2515_readRegister( mcp_addr-1 );
    m_nDlc = mcp2515_readRegister( mcp_addr+4 );

    if (ctrl & 0x08)
        m_nRtr = 1;
    else
        m_nRtr = 0;

    m_nDlc &= MCP_DLC_MASK;
    mcp2515_readRegisterS( mcp_addr+5, &(m_nDta[0]), m_nDlc );
}

/*********************************************************************************************************
** Function name:           mcp2515_getNextFreeTXBuf
** Descriptions:            Send message
*********************************************************************************************************/
uint8_t MCP_CAN::mcp2515_getNextFreeTXBuf(uint8_t *txbuf_n)                 /* get Next free txbuf          */
{
    uint8_t res, i, ctrlval;
    uint8_t ctrlregs[MCP_N_TXBUFFERS] = { MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL };

    res = MCP_ALLTXBUSY;
    *txbuf_n = 0x00;

                                                                        /* check all 3 TX-Buffers       */
    for (i=0; i<MCP_N_TXBUFFERS; i++) {
        ctrlval = mcp2515_readRegister( ctrlregs[i] );
        if ( (ctrlval & MCP_TXB_TXREQ_M) == 0 ) {
            *txbuf_n = ctrlregs[i]+1;                                   /* return SIDH-address of Buffer*/
            
            res = MCP2515_OK;
            return res;                                                 /* ! function exit              */
        }
    }
    return res;
}

/*********************************************************************************************************
** Function name:           MCP_CAN
** Descriptions:            Public function to declare CAN class and the /CS pin.
*********************************************************************************************************/
MCP_CAN::MCP_CAN(uint8_t _CS)
{
    MCPCS = _CS;
  	spibus = &SPI;
    MCP2515_UNSELECT();
    pinMode(MCPCS, OUTPUT);
}

/*********************************************************************************************************
** Function name:           begin
** Descriptions:            Public function to declare controller initialization parameters.
*********************************************************************************************************/
uint8_t MCP_CAN::begin(uint8_t idmodeset, uint8_t speedset, uint8_t clockset)
{
    uint8_t res;

    SPI.begin();

    spibus->transfer(0);  // Strawman transfer, fixes USCI issue on G2553


    res = mcp2515_init(idmodeset, speedset, clockset);
    if (res == MCP2515_OK)
        return CAN_OK;
    
    return CAN_FAILINIT;
}

/*********************************************************************************************************
** Function name:           init_Mask
** Descriptions:            Public function to set mask(s).
*********************************************************************************************************/
uint8_t MCP_CAN::init_Mask(uint8_t num, uint8_t ext, uint32_t ulData)
{
    uint8_t res = MCP2515_OK;
#if DEBUG_MODE
    ////serial.print("Starting to Set Mask!\r\n");
#endif
    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0){
#if DEBUG_MODE
	////serial.print("Entering Configuration Mode Failure...\r\n"); 
#endif
	return res;
     }
    
    if (num == 0){
        mcp2515_write_mf(MCP_RXM0SIDH, ext, ulData);

    }
    else if(num == 1){
        mcp2515_write_mf(MCP_RXM1SIDH, ext, ulData);
    }
    else res =  MCP2515_FAIL;
    
    res = mcp2515_setCANCTRL_Mode(mcpMode);
    if(res > 0){
#if DEBUG_MODE
	////serial.print("Entering Previous Mode Failure...\r\nSetting Mask Failure...\r\n"); 
#endif
	return res;
    }
#if DEBUG_MODE
    ////serial.print("Setting Mask Successful!\r\n");
#endif
    return res;
}

/*********************************************************************************************************
** Function name:           init_Mask
** Descriptions:            Public function to set mask(s).
*********************************************************************************************************/
uint8_t MCP_CAN::init_Mask(uint8_t num, uint32_t ulData)
{
    uint8_t res = MCP2515_OK;
    uint8_t ext = 0;
#if DEBUG_MODE
    ////serial.print("Starting to Set Mask!\r\n");
#endif
    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0){
#if DEBUG_MODE
    //serial.print("Entering Configuration Mode Failure...\r\n"); 
#endif
  return res;
}
    
    if((num & 0x80000000) == 0x80000000)
        ext = 1;
    
    if (num == 0){
        mcp2515_write_mf(MCP_RXM0SIDH, ext, ulData);

    }
    else if(num == 1){
        mcp2515_write_mf(MCP_RXM1SIDH, ext, ulData);
    }
    else res =  MCP2515_FAIL;
    
    res = mcp2515_setCANCTRL_Mode(mcpMode);
    if(res > 0){
#if DEBUG_MODE
    //serial.print("Entering Previous Mode Failure...\r\nSetting Mask Failure...\r\n"); 
#endif
    return res;
  }
#if DEBUG_MODE
    //serial.print("Setting Mask Successful!\r\n");
#endif
    return res;
}

/*********************************************************************************************************
** Function name:           init_Filt
** Descriptions:            Public function to set filter(s).
*********************************************************************************************************/
uint8_t MCP_CAN::init_Filt(uint8_t num, uint8_t ext, uint32_t ulData)
{
    uint8_t res = MCP2515_OK;
#if DEBUG_MODE
    //serial.print("Starting to Set Filter!\r\n");
#endif
    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0)
    {
#if DEBUG_MODE
      //serial.print("Enter Configuration Mode Failure...\r\n"); 
#endif
      return res;
    }
    
    switch( num )
    {
        case 0:
        mcp2515_write_mf(MCP_RXF0SIDH, ext, ulData);
        break;

        case 1:
        mcp2515_write_mf(MCP_RXF1SIDH, ext, ulData);
        break;

        case 2:
        mcp2515_write_mf(MCP_RXF2SIDH, ext, ulData);
        break;

        case 3:
        mcp2515_write_mf(MCP_RXF3SIDH, ext, ulData);
        break;

        case 4:
        mcp2515_write_mf(MCP_RXF4SIDH, ext, ulData);
        break;

        case 5:
        mcp2515_write_mf(MCP_RXF5SIDH, ext, ulData);
        break;

        default:
        res = MCP2515_FAIL;
    }
    
    res = mcp2515_setCANCTRL_Mode(mcpMode);
    if(res > 0)
    {
#if DEBUG_MODE
      //serial.print("Entering Previous Mode Failure...\r\nSetting Filter Failure...\r\n"); 
#endif
      return res;
    }
#if DEBUG_MODE
    //serial.print("Setting Filter Successfull!\r\n");
#endif
    
    return res;
}

/*********************************************************************************************************
** Function name:           init_Filt
** Descriptions:            Public function to set filter(s).
*********************************************************************************************************/
uint8_t MCP_CAN::init_Filt(uint8_t num, uint32_t ulData)
{
    uint8_t res = MCP2515_OK;
    uint8_t ext = 0;
    
#if DEBUG_MODE
    //serial.print("Starting to Set Filter!\r\n");
#endif
    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0)
    {
#if DEBUG_MODE
      //serial.print("Enter Configuration Mode Failure...\r\n"); 
#endif
      return res;
    }
    
    if((num & 0x80000000) == 0x80000000)
        ext = 1;
    
    switch( num )
    {
        case 0:
        mcp2515_write_mf(MCP_RXF0SIDH, ext, ulData);
        break;

        case 1:
        mcp2515_write_mf(MCP_RXF1SIDH, ext, ulData);
        break;

        case 2:
        mcp2515_write_mf(MCP_RXF2SIDH, ext, ulData);
        break;

        case 3:
        mcp2515_write_mf(MCP_RXF3SIDH, ext, ulData);
        break;

        case 4:
        mcp2515_write_mf(MCP_RXF4SIDH, ext, ulData);
        break;

        case 5:
        mcp2515_write_mf(MCP_RXF5SIDH, ext, ulData);
        break;

        default:
        res = MCP2515_FAIL;
    }
    
    res = mcp2515_setCANCTRL_Mode(mcpMode);
    if(res > 0)
    {
#if DEBUG_MODE
      //serial.print("Entering Previous Mode Failure...\r\nSetting Filter Failure...\r\n"); 
#endif
      return res;
    }
#if DEBUG_MODE
    //serial.print("Setting Filter Successfull!\r\n");
#endif
    
    return res;
}

/*********************************************************************************************************
** Function name:           setMsg
** Descriptions:            Set can message, such as dlc, id, dta[] and so on
*********************************************************************************************************/
uint8_t MCP_CAN::setMsg(uint32_t id, uint8_t rtr, uint8_t ext, uint8_t len, uint8_t *pData)
{
    int i = 0;
    m_nID     = id;
    m_nRtr    = rtr;
    m_nExtFlg = ext;
    m_nDlc    = len;
    for(i = 0; i<MAX_CHAR_IN_MESSAGE; i++)
        m_nDta[i] = *(pData+i);
	
    return MCP2515_OK;
}

/*********************************************************************************************************
** Function name:           clearMsg
** Descriptions:            Set all messages to zero
*********************************************************************************************************/
uint8_t MCP_CAN::clearMsg()
{
    m_nID       = 0;
    m_nDlc      = 0;
    m_nExtFlg   = 0;
    m_nRtr      = 0;
    m_nfilhit   = 0;
    for(int i = 0; i<m_nDlc; i++ )
      m_nDta[i] = 0x00;

    return MCP2515_OK;
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            Send message
*********************************************************************************************************/
uint8_t MCP_CAN::sendMsg()
{
    uint8_t res, res1, txbuf_n;
    uint16_t uiTimeOut = 0;

    do {
        res = mcp2515_getNextFreeTXBuf(&txbuf_n);                       /* info = addr.                 */
        uiTimeOut++;
    } while (res == MCP_ALLTXBUSY && (uiTimeOut < TIMEOUTVALUE));

    if(uiTimeOut == TIMEOUTVALUE) 
    {   
        return CAN_GETTXBFTIMEOUT;                                      /* get tx buff time out         */
    }
    uiTimeOut = 0;
    mcp2515_write_canMsg( txbuf_n);
    mcp2515_modifyRegister( txbuf_n-1 , MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M );
    
    do
    {
        uiTimeOut++;        
        res1 = mcp2515_readRegister(txbuf_n-1);                         /* read send buff ctrl reg 	*/
        res1 = res1 & 0x08;                               		
    } while (res1 && (uiTimeOut < TIMEOUTVALUE));   
    
    if(uiTimeOut == TIMEOUTVALUE)                                       /* send msg timeout             */	
        return CAN_SENDMSGTIMEOUT;
    
    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            Send message to transmitt buffer
*********************************************************************************************************/
uint8_t MCP_CAN::sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf)
{
    uint8_t res;
	
    setMsg(id, 0, ext, len, buf);
    res = sendMsg();
    
    return res;
}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            Send message to transmitt buffer
*********************************************************************************************************/
uint8_t MCP_CAN::sendMsgBuf(uint32_t id, uint8_t len, uint8_t *buf)
{
    uint8_t ext = 0, rtr = 0;
    uint8_t res;
    
    if((id & 0x80000000) == 0x80000000)
        ext = 1;
 
    if((id & 0x40000000) == 0x40000000)
        rtr = 1;
        
    setMsg(id, rtr, ext, len, buf);
    res = sendMsg();
    
    return res;
}

/*********************************************************************************************************
** Function name:           readMsg
** Descriptions:            Read message
*********************************************************************************************************/
uint8_t MCP_CAN::readMsg()
{
    uint8_t stat, res;

    stat = mcp2515_readStatus();

    if ( stat & MCP_STAT_RX0IF )                                        /* Msg in Buffer 0              */
    {
        mcp2515_read_canMsg( MCP_RXBUF_0);
        mcp2515_modifyRegister(MCP_CANINTF, MCP_RX0IF, 0);
        res = CAN_OK;
    }
    else if ( stat & MCP_STAT_RX1IF )                                   /* Msg in Buffer 1              */
    {
        mcp2515_read_canMsg( MCP_RXBUF_1);
        mcp2515_modifyRegister(MCP_CANINTF, MCP_RX1IF, 0);
        res = CAN_OK;
    }
    else 
        res = CAN_NOMSG;
    
    return res;
}

/*********************************************************************************************************
** Function name:           readMsgBuf
** Descriptions:            Public function, Reads message from receive buffer.
*********************************************************************************************************/
uint8_t MCP_CAN::readMsgBuf(uint32_t *id, uint8_t *ext, uint8_t *len, uint8_t buf[])
{
    if(readMsg() == CAN_NOMSG)
	return CAN_NOMSG;
	
    *id  = m_nID;
    *len = m_nDlc;
    *ext = m_nExtFlg;
    for(int i = 0; i<m_nDlc; i++)
        buf[i] = m_nDta[i];

    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           readMsgBuf
** Descriptions:            Public function, Reads message from receive buffer.
*********************************************************************************************************/
uint8_t MCP_CAN::readMsgBuf(uint32_t *id, uint8_t *len, uint8_t buf[])
{
    if(readMsg() == CAN_NOMSG)
	return CAN_NOMSG;

    if (m_nExtFlg)
        m_nID |= 0x80000000;

    if (m_nRtr)
        m_nID |= 0x40000000;
	
    *id  = m_nID;
    *len = m_nDlc;
    
    for(int i = 0; i<m_nDlc; i++)
        buf[i] = m_nDta[i];

    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           checkReceive
** Descriptions:            Public function, Checks for received data.  (Used if not using the interrupt output)
*********************************************************************************************************/
uint8_t MCP_CAN::checkReceive(void)
{
    uint8_t res;
    res = mcp2515_readStatus();                                         /* RXnIF in Bit 1 and 0         */
    if ( res & MCP_STAT_RXIF_MASK )
        return CAN_MSGAVAIL;
    else 
        return CAN_NOMSG;
}

/*********************************************************************************************************
** Function name:           checkError
** Descriptions:            Public function, Returns error register data.
*********************************************************************************************************/
uint8_t MCP_CAN::checkError(void)
{
    uint8_t eflg = mcp2515_readRegister(MCP_EFLG);

    if ( eflg & MCP_EFLG_ERRORMASK ) 
        return CAN_CTRLERROR;
    else
        return CAN_OK;
}

/*********************************************************************************************************
** Function name:           getError
** Descriptions:            Returns error register value.
*********************************************************************************************************/
uint8_t MCP_CAN::getError(void)
{
    return mcp2515_readRegister(MCP_EFLG);
}

/*********************************************************************************************************
** Function name:           mcp2515_errorCountRX
** Descriptions:            Returns REC register value
*********************************************************************************************************/
uint8_t MCP_CAN::errorCountRX(void)                             
{
    return mcp2515_readRegister(MCP_REC);
}

/*********************************************************************************************************
** Function name:           mcp2515_errorCountTX
** Descriptions:            Returns TEC register value
*********************************************************************************************************/
uint8_t MCP_CAN::errorCountTX(void)                             
{
    return mcp2515_readRegister(MCP_TEC);
}

/*********************************************************************************************************
** Function name:           mcp2515_enOneShotTX
** Descriptions:            Enables one shot transmission mode
*********************************************************************************************************/
uint8_t MCP_CAN::enOneShotTX(void)                             
{
    mcp2515_modifyRegister(MCP_CANCTRL, MODE_ONESHOT, MODE_ONESHOT);
    if((mcp2515_readRegister(MCP_CANCTRL) & MODE_ONESHOT) != MODE_ONESHOT)
	    return CAN_FAIL;
    else
	    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           mcp2515_disOneShotTX
** Descriptions:            Disables one shot transmission mode
*********************************************************************************************************/
uint8_t MCP_CAN::disOneShotTX(void)                             
{
    mcp2515_modifyRegister(MCP_CANCTRL, MODE_ONESHOT, 0);
    if((mcp2515_readRegister(MCP_CANCTRL) & MODE_ONESHOT) != 0)
        return CAN_FAIL;
    else
        return CAN_OK;
}

/*********************************************************************************************************
** Function name:           mcp2515_abortTX
** Descriptions:            Aborts any queued transmissions
*********************************************************************************************************/
uint8_t MCP_CAN::abortTX(void)                             
{
    mcp2515_modifyRegister(MCP_CANCTRL, ABORT_TX, ABORT_TX);
	
    // Maybe check to see if the TX buffer transmission request bits are cleared instead?
    if((mcp2515_readRegister(MCP_CANCTRL) & ABORT_TX) != ABORT_TX)
	    return CAN_FAIL;
    else
	    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           setGPO
** Descriptions:            Public function, Checks for r
*********************************************************************************************************/
uint8_t MCP_CAN::setGPO(uint8_t data)
{
    mcp2515_modifyRegister(MCP_BFPCTRL, MCP_BxBFS_MASK, (data<<4));
	    
    return 0;
}

/*********************************************************************************************************
** Function name:           getGPI
** Descriptions:            Public function, Checks for r
*********************************************************************************************************/
uint8_t MCP_CAN::getGPI(void)
{
    uint8_t res;
    res = mcp2515_readRegister(MCP_TXRTSCTRL) & MCP_BxRTS_MASK;
    return (res >> 3);
}

/*********************************************************************************************************
** Function name:           MCPwakeup
** Descriptions:            Public function, Wakes up from sleep
*********************************************************************************************************/
void MCP_CAN::MCPwakeup(void)
{
	mcp2515_setRegister(MCP_CANINTE, WAKFIL_ENABLE | MCP_RX0IF | MCP_RX1IF);
	mcp2515_setRegister(MCP_CANINTF, WAKFIL_ENABLE);
	mcp2515_setRegister(MCP_CANINTF, 0);
	mcp2515_setRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF);
	mcp2515_setCANCTRL_Mode(MCP_NORMAL);
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
