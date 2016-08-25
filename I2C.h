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
#ifndef I2C_H
#define I2C_H

#define I2C_QUEUE_SIZE 16

#include <stdint.h>

class I2CReadBase{
protected:
	unsigned int storageSize;
	inline uint8_t * getStorage(){ return ((uint8_t *) &storageSize) + sizeof(storageSize); }
	I2CReadBase(){};
public:
	inline uint8_t * data(){ return getStorage() + 3; }
	inline uint8_t & operator[](int index){ return data()[index]; }
	void schedule();
	void execute();
};

class I2CWriteBase{
protected:
	unsigned int storageSize;
	inline uint8_t * getStorage(){ return ((uint8_t *) &storageSize) + sizeof(storageSize); }
	I2CWriteBase(){};
public:
	inline uint8_t * data(){ return getStorage() + 2; }
	inline uint8_t & operator[](int index){ return data()[index]; }
	void schedule();
	void execute();
};

template<int size>
class I2CWrite : public I2CWriteBase{
private:
	uint8_t storage[size + 2];
public:
	I2CWrite(uint8_t slaveAddress, uint8_t registerAddress){
		storageSize = size + 2;
		storage[0] = slaveAddress << 1;
		storage[1] = registerAddress;
	}
};

template<int size>
class I2CRead : public I2CReadBase{
private:
	uint8_t storage[size + 3];
public:
	I2CRead(uint8_t slaveAddress, uint8_t registerAddress){
		storageSize = size + 3;
		storage[0] = slaveAddress << 1;
		storage[1] = registerAddress;
		storage[2] = storage[0] | 1;
	}
};

class I2C{
public:
	static void init();
	static void flush();
	static void writeSync(uint8_t slaveAddress, uint8_t registerAddress, uint8_t value){
		I2CWrite<1> write(slaveAddress, registerAddress);
		write[0] = value;
		write.execute();
	}
	static uint8_t readSync(uint8_t slaveAddress, uint8_t registerAddress){
		I2CRead<1> read(slaveAddress, registerAddress);
		read.execute();
		return read[0];
	}
	static bool isIdle(){ return idle; }
private:
	struct I2CAtomicOp{
		uint8_t * data;
		unsigned int dataLength;
	};
	static void nextOp();
	static void startOp();
	static void readIsr();
	static void requestIsr();
	static void writeIsr();

	static I2CAtomicOp operations[I2C_QUEUE_SIZE];
	static volatile unsigned int beginPos;
	static volatile unsigned int endPos;

	static volatile I2CAtomicOp * currentOp;
	static volatile unsigned int currentByte;

	static volatile bool idle;

	friend class I2CReadBase;
	friend class I2CWriteBase;
};

#endif // I2C_H
