// file: w5500.defs.h
#pragma once

// W5500 SPI block select bits
#define W5500_COMMON_BLOCK      0x00                // Common registers
#define W5500_SOCKET_BLOCK(s)   (0x08 + (s << 1))   // Socket registers (0-7)
#define W5500_TX_BLOCK(s)       (0x10 + (s << 1))   // Socket TX buffer (0-7)
#define W5500_RX_BLOCK(s)       (0x18 + (s << 1))   // Socket RX buffer (0-7)

// Common Register block
#define _MR         0x0000, W5500_COMMON_BLOCK // 1 byte Mode
#define GAR         0x0001, W5500_COMMON_BLOCK // 4 byte Gateway IP address
#define SUBR        0x0005, W5500_COMMON_BLOCK // 4 byte Subnet mask address
#define SHAR        0x0009, W5500_COMMON_BLOCK // 6 byte Source MAC address
#define SIPR        0x000F, W5500_COMMON_BLOCK // 4 byte Source IP address
#define INTLEVEL    0x0013, W5500_COMMON_BLOCK // 2 byte Interrupt Low Level Timer
#define IR          0x0015, W5500_COMMON_BLOCK // 1 byte Interrupt
#define IMR         0x0016, W5500_COMMON_BLOCK // 1 byte Interrupt Mask
#define SIR         0x0017, W5500_COMMON_BLOCK // 1 byte Socket Interrupt
#define SIMR        0x0018, W5500_COMMON_BLOCK // 1 byte Socket Interrupt Mask
#define RTR         0x0019, W5500_COMMON_BLOCK // 2 byte RTR address
#define RCR         0x001B, W5500_COMMON_BLOCK // 1 byte Retry count
#define PTIMER      0x001C, W5500_COMMON_BLOCK // 1 byte PPP LCP Request Timer
#define PMAGIC      0x001D, W5500_COMMON_BLOCK // 1 byte PPP LCP Magic number
#define PHAR        0x001E, W5500_COMMON_BLOCK // 6 byte PPP Destination MAC Address
#define PSID        0x0024, W5500_COMMON_BLOCK // 2 byte PPP Session Identification 
#define PMRU        0x0026, W5500_COMMON_BLOCK // 2 byte PPP Maximum Segment Size 
#define UIPR        0x0028, W5500_COMMON_BLOCK // 4 byte Unreachable IP address in UDP mode
#define UPORT       0x002C, W5500_COMMON_BLOCK // 2 byte Unreachable Port address in UDP mode
#define PHYCFGR     0x002E, W5500_COMMON_BLOCK // 1 byte PHY Configuration register, default value: 0b 1011 1xxx
#define VERSIONR    0x0039, W5500_COMMON_BLOCK // 1 byte Chip Version Register

// Socket register block
#define SnMR(s)        0x0000, W5500_SOCKET_BLOCK(s) // 1 byte Mode
#define SnCR(s)        0x0001, W5500_SOCKET_BLOCK(s) // 1 byte Command
#define SnIR(s)        0x0002, W5500_SOCKET_BLOCK(s) // 1 byte Interrupt
#define SnSR(s)        0x0003, W5500_SOCKET_BLOCK(s) // 1 byte Status
#define SnPORT(s)      0x0004, W5500_SOCKET_BLOCK(s) // 2 byte Source Port
#define SnDHAR(s)      0x0006, W5500_SOCKET_BLOCK(s) // 6 byte Destination Hardw Addr
#define SnDIPR(s)      0x000C, W5500_SOCKET_BLOCK(s) // 4 byte Destination IP Addr
#define SnDPORT(s)     0x0010, W5500_SOCKET_BLOCK(s) // 2 byte Destination Port
#define SnMSSR(s)      0x0012, W5500_SOCKET_BLOCK(s) // 2 byte Max Segment Size
#define SnTOS(s)       0x0015, W5500_SOCKET_BLOCK(s) // 1 byte IP TOS
#define SnTTL(s)       0x0016, W5500_SOCKET_BLOCK(s) // 1 byte IP TTL
#define SnRXBUFSIZE(s) 0x001E, W5500_SOCKET_BLOCK(s) // 1 byte Receive Buffer Size 
#define SnTXBUFSIZE(s) 0x001F, W5500_SOCKET_BLOCK(s) // 1 byte Transmit Buffer Size 
#define SnTX_FSR(s)    0x0020, W5500_SOCKET_BLOCK(s) // 2 byte TX Free Size
#define SnTX_RD(s)     0x0022, W5500_SOCKET_BLOCK(s) // 2 byte TX Read Pointer
#define SnTX_WR(s)     0x0024, W5500_SOCKET_BLOCK(s) // 2 byte TX Write Pointer
#define SnRX_RSR(s)    0x0026, W5500_SOCKET_BLOCK(s) // 2 byte RX Free Size
#define SnRX_RD(s)     0x0028, W5500_SOCKET_BLOCK(s) // 2 byte RX Read Pointer
#define SnRX_WR(s)     0x002A, W5500_SOCKET_BLOCK(s) // 2 byte RX Write Pointer (supported?)
#define SnIMR(s)       0x002C, W5500_SOCKET_BLOCK(s) // 1 byte Interrupt Mask
#define SnFRAG(s)      0x002D, W5500_SOCKET_BLOCK(s) // 2 byte Fragment Offset in IP header 
#define SnKPALVTR(s)   0x002F, W5500_SOCKET_BLOCK(s) // 1 byte Keep alive timer

// Mode register bits
#define MR_RESET    0b10000000 // Reset
#define MR_WOL      0b00100000 // Wake on LAN
#define MR_PB       0b00010000 // Power down mode
#define MR_PPPOE    0b00001000 // PPPoE mode
#define MR_FARP     0b00000010 // Force ARP

// Socket n command register
#define SnCR_OPEN        0x01
#define SnCR_LISTEN      0x02
#define SnCR_CONNECT     0x04
#define SnCR_DISCON      0x08
#define SnCR_CLOSE       0x10
#define SnCR_SEND        0x20
#define SnCR_SEND_MAC    0x21
#define SnCR_SEND_KEEP   0x22
#define SnCR_RECV        0x40

// Socket n mode register
#define SnMR_CLOSE       0x00
#define SnMR_TCP         0x21
#define SnMR_UDP         0x02
#define SnMR_IPRAW       0x03
#define SnMR_MACRAW      0x04
#define SnMR_PPPOE       0x05
#define SnMR_ND          0x20
#define SnMR_MULTI       0x80

// Socket n status register
#define SnSR_CLOSED      0x00
#define SnSR_INIT        0x13
#define SnSR_LISTEN      0x14
#define SnSR_SYNSENT     0x15
#define SnSR_SYNRECV     0x16
#define SnSR_ESTABLISHED 0x17
#define SnSR_FIN_WAIT    0x18
#define SnSR_CLOSING     0x1A
#define SnSR_TIME_WAIT   0x1B
#define SnSR_CLOSE_WAIT  0x1C
#define SnSR_LAST_ACK    0x1D
#define SnSR_UDP         0x22
#define SnSR_IPRAW       0x32
#define SnSR_MACRAW      0x42
#define SnSR_PPPOE       0x5F

// Socket n interrupt mask register
#define SnIMR_CON        0b00000001 // Connect interrupt
#define SnIMR_DISCON     0b00000010 // Disconnect interrupt
#define SnIMR_RECV       0b00000100 // Receive interrupt
#define SnIMR_TIMEOUT    0b00001000 // Timeout interrupt
#define SnIMR_SENDOK     0b00010000 // Send OK interrupt
#define SnIMR_ALL        0b00011111 // All interrupts

#define W5500_MODE_VDM   0b00  // Variable Data Length
#define W5500_MODE_FDM1  0b01  // Fixed 1 byte
#define W5500_MODE_FDM2  0b10  // Fixed 2 bytes
#define W5500_MODE_FDM4  0b11  // Fixed 4 bytes

#define W5500_READ       0b000  // SPI read   
#define W5500_WRITE      0b100  // SPI write
