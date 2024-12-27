#include <blackbox/WinbondW25N.h>
#include <TimeTick.h>

W25N::W25N(){

}


int W25N::init(SPI_Config config) {
	m_hSpi = config.hspi;
	m_csPort = config.csPort;
	m_csPin = config.csPin;

//	uint16_t bbm[40];

	reset();
	uint8_t jedec[5] = { W25N_JEDEC_ID, 0x00, 0x00, 0x00, 0x00 };
	readReg(jedec, &jedec[2], 3, 2);
	if (jedec[2] == WINBOND_MAN_ID) {
		if ((uint16_t) (jedec[3] << 8 | jedec[4]) == W25N01GV_DEV_ID) {
			uint8_t config = getStatusReg(W25N_CONFIG_REG);
			setStatusReg(W25N_PROT_REG, 0x00);
			TimeTick::delay_ms(1);
//			uint8_t reg = getStatusReg(W25N_CONFIG_REG);
			_model = W25N01GV;
//			readBBM(bbm);
			return 0;
		}

		if ((uint16_t) (jedec[3] << 8 | jedec[4]) == W25M02GV_DEV_ID) {
			_model = W25M02GV;
			dieSelect(0);
			setStatusReg(W25N_PROT_REG, 0x00);
			dieSelect(1);
			setStatusReg(W25N_PROT_REG, 0x00);
			dieSelect(0);
			return 0;
		}
	}




	return 1;
}

//#
void W25N::sendData(uint8_t* buf, uint32_t len){
	HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(m_hSpi, buf, len, 1);
	HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);
}

//#
void W25N::reset(){
	//TODO check WIP in case of reset during write
	uint8_t buf[] = {W25N_RESET};
	sendData(buf, sizeof(buf));
	TimeTick::delay_us(1000);	//Trst max time 500uS
}

int W25N::dieSelect(char die){
	//TODO add some type of input validation
	uint8_t buf[2] = {W25M_DIE_SELECT, die};
	sendData(buf, sizeof(buf));
	_dieSelect = die;
	return 0;
}

void W25N::setReadMode(ReadMode mode) {
	uint8_t reg = getStatusReg(W25N_CONFIG_REG);
	if((reg & 0x01) == (uint8_t)mode) {
		return;
	}
	reg &= (0xFE | mode);
	setStatusReg(W25N_CONFIG_REG, reg);
}

int W25N::dieSelectOnAdd(uint32_t pageAdd){
	return 0;
	if(pageAdd > getMaxPage()) return 1;
	return dieSelect(pageAdd / W25N01GV_MAX_PAGE);
}
//#
uint8_t W25N::getStatusReg(uint8_t reg){
	uint8_t txbuf[3] = {W25N_READ_STATUS_REG, reg};
	uint8_t regValue;
	readReg(txbuf,&regValue, 1, 2);
	return regValue;
}
//#
void W25N::setStatusReg(uint8_t reg, uint8_t data){
	uint8_t txbuf[3] = {W25N_WRITE_STATUS_REG, reg, data};
	sendData(txbuf, 3);
}
//#
uint32_t W25N::getMaxPage(){
	if (_model == W25M02GV) return W25M02GV_MAX_PAGE;
	if (_model == W25N01GV) return W25N01GV_MAX_PAGE;
	return 0;
}
//#
void W25N::writeEnable(){
	uint8_t buf[] = {W25N_WRITE_ENABLE};
	sendData(buf, 1);
}
//#
void W25N::writeDisable(){
	uint8_t buf[] = {W25N_WRITE_DISABLE};
	sendData(buf, 1);
}
//#
bool W25N::blockErase(uint32_t pageAdd){
	if(pageAdd > getMaxPage()) return 1;
	//  dieSelectOnAdd(pageAdd);
	uint8_t pageHigh = (uint8_t)((pageAdd & 0xFF00) >> 8);
	uint8_t pageLow = (uint8_t)(pageAdd & 0xFF);
	uint8_t txbuf[4] = {W25N_BLOCK_ERASE, 0x00, pageHigh, pageLow};
	// What to do if wip times out ????
	block_WIP();
	writeEnable();
	sendData(txbuf, 4);
//	TimeTick::delay_ms(15);
	block_WIP();
	return !eraseFailed();
}
//#
bool W25N::bulkErase(){
	for(uint32_t i = 0; i < getMaxPage(); i+=64){
		if(!blockErase(i)) return false;
	}
	return true;
}

bool W25N::bulkErase(uint32_t startAddr) {
	for(uint32_t i = startAddr; i < getMaxPage(); i+=64){
		if(!blockErase(i)) {
//			TimeTick::delay_ms(15);
			if(!blockErase(i)) {
				return false;
			}
		}
	}
	return true;
}
//#
int W25N::loadProgData(uint16_t columnAdd, uint8_t* buf, uint32_t dataLen){
	if(columnAdd > (uint32_t)W25N_MAX_COLUMN) return 1;
	if(dataLen > (uint32_t)W25N_MAX_COLUMN - columnAdd) return 1;
	uint8_t columnHigh = (columnAdd & 0xFF00) >> 8;
	uint8_t columnLow = columnAdd & 0xff;
	uint8_t txbuf[3] = {W25N_PROG_DATA_LOAD, columnHigh, columnLow};
	// What to do if wip times out ????
	block_WIP();
	writeEnable();
	multiWrite(txbuf, buf,3, dataLen);
	return 0;
}
//#
int W25N::loadProgData(uint16_t columnAdd, uint8_t* buf, uint32_t dataLen, uint32_t pageAdd){
//	if(dieSelectOnAdd(pageAdd)) return 1;
	return loadProgData(columnAdd, buf, dataLen);
}
//#
int W25N::loadRandProgData(uint16_t columnAdd, uint8_t* buf, uint32_t dataLen){
	if(columnAdd > (uint32_t)W25N_MAX_COLUMN) return 1;
	if(dataLen > (uint32_t)W25N_MAX_COLUMN - columnAdd) return 1;
	char columnHigh = (columnAdd & 0xFF00) >> 8;
	char columnLow = columnAdd & 0xff;
	char txbuf[3] = {W25N_RAND_PROG_DATA_LOAD, columnHigh, columnLow};
	block_WIP();
	writeEnable();
	multiWrite(txbuf, buf,3, dataLen);
	return 0;
}
//#
int W25N::loadRandProgData(uint16_t columnAdd, uint8_t* buf, uint32_t dataLen, uint32_t pageAdd){
//	if(dieSelectOnAdd(pageAdd)) return 1;
	return loadRandProgData(columnAdd, buf, dataLen);
}
//#
int W25N::ProgramExecute(uint32_t pageAdd){
	if(pageAdd > getMaxPage()) return 1;
//	dieSelectOnAdd(pageAdd);
	uint8_t pageHigh = (uint8_t)((pageAdd & 0xFF00) >> 8);
	uint8_t pageLow = (uint8_t)(pageAdd);
	block_WIP();
	writeEnable();
	uint8_t txbuf[4] = {W25N_PROG_EXECUTE, 0x00, pageHigh, pageLow};
	sendData(txbuf, 4);
	return 0;
}
//#
int W25N::pageDataRead(uint32_t pageAdd){
	if(pageAdd > getMaxPage()) return 1;
//	dieSelectOnAdd(pageAdd);
	uint8_t pageHigh = (uint8_t)((pageAdd & 0xFF00) >> 8);
	uint8_t pageLow = (uint8_t)(pageAdd);
	uint8_t txbuf[4] = {W25N_PAGE_DATA_READ, 0x00, pageHigh, pageLow};
	block_WIP();
	sendData(txbuf, 4);
	return 0;

}

void W25N::readBBM(uint16_t* bbm) {
	uint8_t reg[] = {W25N_READ_BBM, 0x00};
	uint8_t* _bbm = (uint8_t*)bbm;
	block_WIP();
	readReg(reg, _bbm, 80, 2);

	for(uint8_t i = 0; i < 80; i += 2) {
		uint8_t tmp = _bbm[i];
		_bbm[i] = _bbm[i+1];
		_bbm[i+1] = tmp;
	}
}

uint16_t W25N::getBadPageAddress() {
	uint8_t data[2];
	uint8_t reg[] = {W25N_LAST_ECC_FAIL, 0x00};
	block_WIP();
	readReg(reg, data, 2, 2);
	uint16_t addr = data[0] << 8 | data[1];
	return addr;
}

int W25N::read(uint16_t columnAdd, uint8_t* buf, uint32_t dataLen){
	if(columnAdd > (uint32_t)W25N_MAX_COLUMN) return 1;
	if(dataLen > (uint32_t)W25N_MAX_COLUMN - columnAdd) return 1;
	uint8_t columnHigh = (columnAdd & 0xFF00) >> 8;
	uint8_t columnLow = columnAdd & 0xff;
	uint8_t txbuf[4] = {W25N_READ, columnHigh, columnLow, 0x00};
	block_WIP();
	readReg(txbuf, buf, dataLen, 4);
	return 0;
}
//Returns the Write In Progress bit from flash.
int W25N::check_WIP(){
	char status = getStatusReg(W25N_STAT_REG);
	if(status & 0x01){
		return 1;
	}
	return 0;
}

int W25N::block_WIP(){
	//Max WIP time is 10ms for block erase so 15 should be a max.
	timetick_us tstamp = TimeTick::getTimeUs();
//	uint32_t tstamp = HAL_GetTick();
	while(check_WIP()){
		TimeTick::delay_us(100);
//		HAL_Delay(1);
		if (TimeTick::getTimeUs() > tstamp + 15000) return 1;
//		if (HAL_GetTick() > tstamp + 15) return 1;
	}
	return 0;
}

int W25N::check_status(){
	return(getStatusReg(W25N_STAT_REG));
}

uint8_t W25N::getECCStatus() {
	uint8_t status = check_status();
	status = (status >> 4) & 0x03;
	return status;
}

bool W25N::eraseFailed() {
	uint8_t status = check_status();
	return (status & 0x04) > 0;
}

bool W25N::lutFull() {
	uint8_t status = check_status();
	return (status & 0x40) > 0;
}

bool W25N::programFailed() {
	uint8_t status = check_status();
	return (status & 0x08) > 0;
}

uint16_t W25N::getBadBlocks(uint16_t* badBlocks) {
	uint8_t data;
	uint16_t numBadBlocks = 0;

	for(uint16_t blk = 0; blk < 1024; blk++) {
		uint16_t addr = blk * 64;
		pageDataRead(addr);
		uint8_t eec = getECCStatus();
		TimeTick::delay_us(100);
		read(0, &data, 1);
		if(data != 0xFF) {
			badBlocks[numBadBlocks++] = blk;
		}
	}

	return numBadBlocks;
}

void W25N::loadProgAndExecute(uint16_t page, uint16_t startAddr, uint8_t* buff, uint16_t dataLen) {
	loadProgData(startAddr, buff, dataLen);
//	TimeTick::delay_ms(1);
	ProgramExecute(page);
//	TimeTick::delay_ms(1);
}

bool W25N::readPage(uint16_t page, uint16_t startAddr, uint8_t* buff, uint16_t dataLen) {
	pageDataRead(page);
//	TimeTick::delay_ms(1);
	read(startAddr, buff, dataLen);
	return getECCStatus() <= 1;
}

void W25N::writeReg(uint8_t reg, void *pBuf, size_t size)
{
	HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET);
//	TimeTick::delay_us(2);
	HAL_SPI_Transmit(m_hSpi,&reg,1,10);
	HAL_SPI_Transmit(m_hSpi,(uint8_t*)pBuf,size,10);
//	TimeTick::delay_us(2);
	HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);
}

uint8_t W25N::readReg(uint8_t* reg, void *pBuf, uint16_t rxSize, uint16_t txSize)
{
	HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET);
//	TimeTick::delay_us(2);
	HAL_SPI_Transmit(m_hSpi, reg, txSize, 10);
	HAL_SPI_Receive(m_hSpi, (uint8_t *)pBuf, rxSize, 10);
//	TimeTick::delay_us(2);
	HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);
	return rxSize;
}

void W25N::multiWrite(void *pBuf1, void *pBuf2, size_t size1, size_t size2)
{
	HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET);
//	TimeTick::delay_us(2);
	HAL_SPI_Transmit(m_hSpi,(uint8_t*)pBuf1,size1,10);
	HAL_SPI_Transmit(m_hSpi,(uint8_t*)pBuf2,size2,10);
//	TimeTick::delay_us(2);
	HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);
}
