#include <WiFi.h>
#include <WiFiUdp.h>

uint8_t checksum(uint8_t *data, size_t length);
void sendConfigPacket(uint8_t* buffer, uint16_t len);
void checkUartRx();
void sendToUart(uint8_t* buffer, uint16_t size);

typedef struct __attribute__((packed)) {
  float pp;
  float p;
  float i;
  float d;
}PidParam_t;

typedef struct __attribute__((packed)) {
  PidParam_t roll;
  PidParam_t pitch;
  PidParam_t yaw;
  PidParam_t px;
  PidParam_t py;
  PidParam_t alt;
}PID_t;


typedef struct __attribute__((packed)) {
  const uint32_t startByte = 0xDEADFACE;
  uint8_t cmd;
  uint8_t len;
  PID_t payload;
  uint8_t CRC;
}Packet_t;

typedef struct __attribute__((packed)) {
	uint32_t Header 		= 0xDEADFACE;
	uint8_t Cmd 			= 0xE8;
	uint8_t Len;
	uint8_t* config;
	uint8_t crc;
}Config_Packet;

typedef enum {
  CMD_CONF_REQ = 0x01,
  CMD_CONF_RESP,
  CMD_PID_RESP,
  CMD_STATUS,
} ConfigCMD;

// typedef struct __attribute__((packed)) {
//   const uint32_t startByte = 0xDEADFACE;
//   uint8_t cmd;
//   uint18_t len = 1;
//   uint8_t resp;
//   uint8_t CRC;
// }Response_Packet_t;

// Access Point (AP) network credentials
const char* ssid = "ESP32_AP";
const char* password = "12345678";  // Minimum 8 characters

// UDP settings
WiFiUDP udp;
const int udpPort = 4445;           // Port to listen for incoming UDP packets
char incomingPacket[255];            // Buffer for incoming packets

IPAddress remoteIp;
int remotePort = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1,20,21);
  Serial1.onReceive(checkUartRx);

  // Set up the ESP32 as an Access Point
  Serial.print("Setting up AP...");
  WiFi.softAP(ssid, password);
  Serial.println("done");
  
  // Print the IP address of the ESP32
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Start the UDP server
  udp.begin(udpPort);
  Serial.print("UDP server started on port ");
  Serial.println(udpPort);
}

void checkUartRx() {
	static uint16_t payloadSize = 0;
	static uint8_t cmd;
  static uint8_t m_state;
  static uint8_t m_buffer[256];
  uint8_t data;
  static uint32_t t = 0;

  // if(millis()-t > 1000) {
  //   t = millis();
  //   Serial.print("Checking uart\n");
  // }

  while(Serial1.available())
  {
    m_buffer[m_state] = Serial1.read();
    // Serial.print(m_buffer[m_state],16);
    switch(m_state) {
      case 0:
        if(m_buffer[0] == 0xCE) {
          m_state = 1;
        }
        break;
      case 1:
        if(m_buffer[1] == 0xFA) {
          m_state = 2;
        }
        else {
          m_state = 0;
        }
        break;
      case 2:
        if(m_buffer[2] == 0xAD) {
          m_state = 3;
        }
        else {
          m_state = 0;
        }
        break;
      case 3:
        if(m_buffer[3] == 0xDE) {
          m_state = 4;
        }
        else {
          m_state = 0;
        }
        break;
      case 4:
        cmd = m_buffer[4];
        m_state = 5;
        // if(cmd  == CMD_PID_RESP || cmd == CMD_CONF_REQ) {
        //   m_state = 5;
        // }
        // else {
        //   m_state = 0;
        // }
        break;
      case 5:
        payloadSize = m_buffer[5];
        m_state = 6;
        break;
      default:
        if(m_state >= payloadSize + 5) {
          // Serial.println();
          // Serial.print("Complete config packet received\n");
          uint8_t crc = m_buffer[payloadSize + 5];
          uint8_t crc_eval = checksum(m_buffer, payloadSize + 5);

          // char buf[20];
          // snprintf(buf,20,"crc: %d\n", crc);
          // Serial.print(buf);

          // snprintf(buf,20,"crc_eval: %d\n", crc_eval);
          // Serial.print(buf);

          if(crc == crc_eval){
            // Serial.print("CRC of Config packet correct\n");
            // if(cmd == CMD_CONF_REQ) {
              sendConfigPacket(m_buffer, payloadSize + 5);
            // }
          }
          else {;
            // Serial.print("Config packet failed crc check\n");
          }   
          m_state = 0;
          payloadSize = 0;
          break;           
        }    
        ++m_state;
    }
  }
}

void loop() {
  uint8_t crc, est_crc;
  // Check if there's an incoming packet
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Read the incoming packet into the buffer
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      // char buf[64];
      // snprintf(buf,64,"Packet Size: %d\n", packetSize);
      // Serial.print(buf);
      // snprintf(buf,64,"LEN: %d\n", len);
      // Serial.print(buf);


      // Print the received packet to the Serial Monitor
      // Serial.print("Received packet: ");
      // for(int i=0; i<len; i++){
      //   Serial.printf("0x%X ", incomingPacket[i]);
      // }
      // Serial.print("\n");

      crc = (uint8_t)(incomingPacket[len-1]);
      est_crc = checksum((uint8_t*)incomingPacket, len-1);

      if(crc == est_crc) {
        // Serial.print("CRC is correct\n");
        // Serial.print("Forwarding received packet\n");
        Serial1.write((uint8_t*)incomingPacket, len);
        // Serial.print("Received packet forwarded\n");

        // if(incomingPacket[4] == CMD_CONF_REQ) {
        //   Serial.print("Request for configuration\n");
        // }
        // else if(incomingPacket[4] == CMD_PID_RESP) {
        //   Packet_t* packet = (Packet_t*)incomingPacket;
        //   crc = packet->CRC;
        //   est_crc = checksum((uint8_t*)incomingPacket, sizeof(Packet_t)-1);

        //   Serial.printf("Roll ==> pp: %.4f;    p: %.4f;    i: %.4f    d: %.4f\n",
        //   packet->payload.roll.pp, packet->payload.roll.p, packet->payload.roll.i, packet->payload.roll.d); 
      
        // }

        // Get the sender's IP and port
        remoteIp = udp.remoteIP();
        remotePort = udp.remotePort();

        // Optional: send a response back to the client
        uint8_t response[8] = {0xDE, 0xAD, 0xFA, 0xCE, CMD_STATUS, 0x01, 0x09};
        udp.beginPacket(remoteIp, remotePort);  // Start the UDP response packet
        udp.write((const uint8_t *)response, 8);            // Write the response message
        udp.endPacket();    

      }
      // else {
      //   Serial.print("CRC is incorrect\n");
      // } 

      // // Get the sender's IP and port
      // remoteIp = udp.remoteIP();
      // remotePort = udp.remotePort();
      // // Serial.print("From IP: ");
      // // Serial.print(remoteIp);
      // // Serial.print(", Port: ");
      // // Serial.println(remotePort);

      // // Optional: send a response back to the client
      // uint8_t response[8] = {0xDE, 0xAD, 0xFA, 0xCE, CMD_STATUS, 0x01, 0x09};
      // udp.beginPacket(remoteIp, remotePort);  // Start the UDP response packet
      // udp.write((const uint8_t *)response, 8);            // Write the response message
      // udp.endPacket();    
    }                    // Send the response packet
  }
  // else {
  //   void checkUartRx();
  // }
}

void sendConfigPacket(uint8_t* buffer, uint16_t len) {
  // if(remoteIP != nullptr && remotePort != 0){
    // Serial.print("Sending config packet\n");
    udp.beginPacket(remoteIp, remotePort);  // Start the UDP response packet
    udp.write((const uint8_t *)buffer, len);            // Write the response message
    udp.endPacket(); 
    // Serial.print("Config packet sent\n");
  // }
}

void sendToUart(uint8_t* buffer, uint16_t size) {
  for(int i = 0; i < size; i++) {
    Serial1.write(buffer[i]);
  }
}

uint8_t checksum(uint8_t *data, size_t length) {
    unsigned char checksum = 0;

    for (size_t i = 0; i < length; i++) {
        checksum += data[i];
    }

    return checksum;
}
