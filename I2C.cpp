/*
MIT License

Copyright (c) 2016 Roman Chomik romanchom@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "I2C.h"
#include <mk20dx128.h>
#include <core_pins.h>
#include <Arduino.h>

void I2CWriteBase::schedule(){
	unsigned int nextEnd = (I2C::endPos + 1) % I2C_QUEUE_SIZE;
	while(nextEnd == I2C::beginPos) Serial.print('.'); // wait until queue has free space

	I2C::operations[I2C::endPos].data = getStorage();
	I2C::operations[I2C::endPos].dataLength = storageSize;

	I2C::endPos = nextEnd;
	if(I2C::idle) {
        I2C::idle = false;
        I2C::startOp();
    }
}

void I2CWriteBase::execute(){
	schedule();
	I2C::flush();
}

void I2CReadBase::schedule(){
	unsigned int nextEnd = (I2C::endPos + 2) % I2C_QUEUE_SIZE;
	while(nextEnd == I2C::beginPos || nextEnd == I2C::beginPos + 1) Serial.print(','); // wait until queue has free space

	I2C::operations[I2C::endPos].data = getStorage();
	I2C::operations[I2C::endPos].dataLength = 2;

	nextEnd = (I2C::endPos + 1) % I2C_QUEUE_SIZE;
	I2C::operations[nextEnd].data = getStorage() + 2;
	I2C::operations[nextEnd].dataLength = storageSize - 2;

	I2C::endPos = (I2C::endPos + 2) % I2C_QUEUE_SIZE;
    if(I2C::idle) {
        I2C::idle = false;
        I2C::startOp();
    }
}

void I2CReadBase::execute(){
	schedule();
	I2C::flush();
}

void I2C::flush(){
	while(!idle);
}

void I2C::init(){
	// Enable I2C internal clock
	SIM_SCGC4 |= SIM_SCGC4_I2C0;

	I2C0_C2 = I2C_C2_HDRS; // Set high drive select
    I2C0_A1 = 0;
    I2C0_RA = 0;

    uint32_t pinConfig0 = PORT_PCR_MUX(2)|PORT_PCR_ODE|PORT_PCR_SRE|PORT_PCR_DSE;

	CORE_PIN16_CONFIG = pinConfig0;
    CORE_PIN17_CONFIG = pinConfig0;

	#if F_BUS == 48000000
		I2C0_F = 0x1B;  // clock divider 240, clock frequency 400k
		//I2C0_F = 0x85;  // clock divider 30, mul 4, clock frequency 400k
		I2C0_FLT = 4;
    #else
        #error "F_BUS must be 48 MHz"
    #endif

    I2C0_C1 = I2C_C1_IICEN;

	NVIC_ENABLE_IRQ(IRQ_I2C0);
	attachInterruptVector(IRQ_I2C0, readIsr);

    idle = true;
}

void I2C::nextOp(){
	beginPos = (beginPos + 1) % I2C_QUEUE_SIZE;
	if(beginPos != endPos) {
		delayMicroseconds(3); // must be so
		startOp();
	}else{
        idle = true;
    }
}

void I2C::startOp(){
	// clear the status flags
	I2C0_S = I2C_S_IICIF | I2C_S_ARBL;

    // become the bus master in transmit mode (send start)
    I2C0_C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;
    // send 1st data and enable interrupts
    I2C0_C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TX; // enable intr

    currentOp = operations + beginPos;

    uint8_t slaveAddress = currentOp->data[0];
    I2C0_D = slaveAddress;
	attachInterruptVector(IRQ_I2C0, (slaveAddress & 1) ? requestIsr : writeIsr);
    currentByte = 0;
}

void I2C::requestIsr(){
	if(I2C0_S & I2C_S_RXAK)
    {
        // Slave addr NAK
        // send STOP, change to Rx mode, intr disabled
        I2C0_C1 = I2C_C1_IICEN;
		// I think it should be here
        nextOp();
    }
    else
    {
        // Slave addr ACK, change to Rx mode
        if(currentOp->dataLength == 2)
            I2C0_C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TXAK; // no STOP, Rx, NAK on recv
        else
            I2C0_C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST; // no STOP, change to Rx

        uint8_t data = I2C0_D; // dummy read

        attachInterruptVector(IRQ_I2C0, readIsr);
    }
	++currentByte;
	I2C0_S = I2C_S_IICIF;
}

void I2C::readIsr(){
    // check if 2nd to last byte or timeout
    if(currentByte + 2 == currentOp->dataLength){
        I2C0_C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_MST | I2C_C1_TXAK; // no STOP, Rx, NAK on recv
    }
    // if last byte or timeout send STOP
    if(currentByte + 1 >= currentOp->dataLength)
    {
        // send STOP, change to Rx mode, intr disabled
        I2C0_C1 = I2C_C1_IICEN;
    	currentOp->data[currentByte] = I2C0_D;
        nextOp();
    }else{
    	currentOp->data[currentByte] = I2C0_D;
		++currentByte;
	}
	I2C0_S = I2C_S_IICIF; // clear intr
}

void I2C::writeIsr(){
	if(I2C0_S & I2C_S_RXAK){
        // send STOP, change to Rx mode, intr disabled
        I2C0_C1 = I2C_C1_IICEN;
		// I think it should be here
		nextOp();
    }else{
        // check if last byte transmitted
        ++currentByte;

        if(currentByte >= currentOp->dataLength){
            // send STOP, change to Rx mode, intr disabled
            I2C0_C1 = I2C_C1_IICEN;
            nextOp();
        }else{
            // transmit next byte
            I2C0_D = currentOp->data[currentByte];
        }
    }
    I2C0_S = I2C_S_IICIF; // clear intr
}

I2C::I2CAtomicOp I2C::operations[I2C_QUEUE_SIZE] = {};
volatile unsigned int I2C::beginPos = 0;
volatile unsigned int I2C::endPos = 0;

volatile I2C::I2CAtomicOp * I2C::currentOp = 0;
volatile unsigned int I2C::currentByte = 0;

volatile bool I2C::idle = false;
