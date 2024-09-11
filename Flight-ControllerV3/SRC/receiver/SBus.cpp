/*
 * SBus.cpp
 *
 *  Created on: Jul 12, 2024
 *      Author: Ikenna
 */

#include <receiver/SBus.h>

//void (*SBus::m_callback)(SbusData);

SBus::SBus() {
	// TODO Auto-generated constructor stub

}

SBus::~SBus() {
	// TODO Auto-generated destructor stub
}

bool SBus::parse() {
  /* Parse messages */
  while (m_uartRx.dataAvailable()) {
    m_currByte_ = m_uartRx.getData();
    if (m_state == 0) {
      if ((m_currByte_ == HEADER_) && ((m_prevByte_ == FOOTER_) ||
         ((m_prevByte_ & 0x0F) == FOOTER2_))) {
        m_Buff[m_state++] = m_currByte_;
      } else {
        m_state = 0;
      }
    } else if (m_state < PAYLOAD_LEN_ + HEADER_LEN_) {
        m_Buff[m_state++] = m_currByte_;
    } else if (m_state < PAYLOAD_LEN_ + HEADER_LEN_ + FOOTER_LEN_) {
      m_state = 0;
      m_prevByte_ = m_currByte_;
      if ((m_currByte_ == FOOTER_) || ((m_currByte_ & 0x0F) == FOOTER2_)) {
        /* Grab the channel data */
        m_sbusData.ch[0]  = static_cast<int16_t>(m_Buff[1] |
                                            ((m_Buff[2] << 8) & 0x07FF));
        m_sbusData.ch[1]  = static_cast<int16_t>((m_Buff[2] >> 3) |
                                            ((m_Buff[3] << 5) & 0x07FF));
        m_sbusData.ch[2]  = static_cast<int16_t>((m_Buff[3] >> 6) |
                                            (m_Buff[4] << 2) |
                                            ((m_Buff[5] << 10) & 0x07FF));
        m_sbusData.ch[3]  = static_cast<int16_t>((m_Buff[5] >> 1) |
                                            ((m_Buff[6] << 7) & 0x07FF));
        m_sbusData.ch[4]  = static_cast<int16_t>((m_Buff[6] >> 4) |
                                            ((m_Buff[7] << 4) & 0x07FF));
        m_sbusData.ch[5]  = static_cast<int16_t>((m_Buff[7] >> 7) |
                                            (m_Buff[8] << 1) |
                                            ((m_Buff[9] << 9) & 0x07FF));
        m_sbusData.ch[6]  = static_cast<int16_t>((m_Buff[9] >> 2) |
                                            ((m_Buff[10] << 6) & 0x07FF));
        m_sbusData.ch[7]  = static_cast<int16_t>((m_Buff[10] >> 5) |
                                            ((m_Buff[11] << 3) & 0x07FF));
        m_sbusData.ch[8]  = static_cast<int16_t>(m_Buff[12] |
                                            ((m_Buff[13] << 8) & 0x07FF));
        m_sbusData.ch[9]  = static_cast<int16_t>((m_Buff[13] >> 3) |
                                            ((m_Buff[14] << 5) & 0x07FF));
        m_sbusData.ch[10] = static_cast<int16_t>((m_Buff[14] >> 6) |
                                            (m_Buff[15] << 2) |
                                            ((m_Buff[16] << 10) & 0x07FF));
        m_sbusData.ch[11] = static_cast<int16_t>((m_Buff[16] >> 1) |
                                            ((m_Buff[17] << 7) & 0x07FF));
        m_sbusData.ch[12] = static_cast<int16_t>((m_Buff[17] >> 4) |
                                            ((m_Buff[18] << 4) & 0x07FF));
        m_sbusData.ch[13] = static_cast<int16_t>((m_Buff[18] >> 7) |
                                            (m_Buff[19] << 1) |
                                            ((m_Buff[20] << 9) & 0x07FF));
        m_sbusData.ch[14] = static_cast<int16_t>((m_Buff[20] >> 2) |
                                            ((m_Buff[21] << 6) & 0x07FF));
        m_sbusData.ch[15] = static_cast<int16_t>((m_Buff[21] >> 5) |
                                            ((m_Buff[22] << 3) & 0x07FF));
        /* CH 17 */
        m_sbusData.ch17 = m_Buff[23] & CH17_MASK_;
        /* CH 18 */
        m_sbusData.ch18 = m_Buff[23] & CH18_MASK_;
        /* Grab the lost frame */
        m_sbusData.lost_frame = m_Buff[23] & LOST_FRAME_MASK_;
        /* Grab the failsafe */
        m_sbusData.failsafe = m_Buff[23] & FAILSAFE_MASK_;
        return true;
      } else {
        return false;
      }
    } else {
      m_state = 0;
    }
    m_prevByte_ = m_currByte_;
  }
  return false;
}



void SBus::taskFunc(timetick_us currenTimeUs) {
  /* Read through all available packets to get the newest */
//  new_data_ = false;
  do {
    if (parse()) {
      m_callback->handleChannelData(m_sbusData);
    }
  } while (m_uartRx.dataAvailable());
//  return new_data_;
}

