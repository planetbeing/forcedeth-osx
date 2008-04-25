/*
 * forcedeth: A Darwin ethernet driver for nVidia nForce4 media access controllers.
 * Copyright (C) 2006  Yiduo Wang
 *
 * Note: This driver is a cleanroom reimplementation based on reverse
 *       engineered documentation written by Carl-Daniel Hailfinger
 *       and Andrew de Quincey. It is based on the forcedeth driver
 *       for Linux by Manfred Spraul, Andrew de Quincey, and
 *       Carl-Daniel Hailfinger. It's neither supported nor endorsed
 *       by nVidia Corp. Use at your own risk.
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <IOKit/IOLib.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/network/IOEthernetController.h>
#include <IOKit/network/IOGatedOutputQueue.h>
#include <sys/kpi_mbuf.h>
#include "forcedeth.h"
/*extern "C" {
#include <pexpert/pexpert.h>//This is for debugging purposes ONLY
}*/
 
// Define my superclass
#define super IOEthernetController

static inline UInt32 descrGetLength(RingDesc *prd) {
	return (OSSwapLittleToHostInt32(prd->FlagLen) & LEN_MASK_V2);
}

volatile UInt32 com_triton_forcedeth::readRegister(UInt16 offset)
{
    return OSReadLittleInt32((void *) baseAddress, offset);
}

void com_triton_forcedeth::writeRegister(UInt16 offset, UInt32 data)
{
    OSWriteLittleInt32((void *) baseAddress, offset, data);
}

bool com_triton_forcedeth::regDelay(int offset, UInt32 mask, UInt32 target, int delay, int delayMax, const char *msg) {
	do {
		IODelay(delay);
		delayMax -= delay;
		if( delayMax < 0 ) {
			if( msg )
				IOLog(msg);
			return true;
		}
	} while( (readRegister(offset) & mask) != target );
	return false;
}

int com_triton_forcedeth::miiRW(int addr, int miireg, int value)
{
	UInt32 reg;

	writeRegister(NvRegMIIStatus, NVREG_MIISTAT_MASK);
	reg = readRegister(NvRegMIIControl);

	if( reg & NVREG_MIICTL_INUSE ){
		writeRegister(NvRegMIIControl, NVREG_MIICTL_INUSE);
		IODelay(NV_MIIBUSY_DELAY);
	}

	reg = (addr << NVREG_MIICTL_ADDRSHIFT) | miireg;
	if( value != MII_READ ) {
		writeRegister(NvRegMIIData, value);
		reg |= NVREG_MIICTL_WRITE;
	}

	writeRegister(NvRegMIIControl, reg);

	if( regDelay(NvRegMIIControl, NVREG_MIICTL_INUSE, 0, NV_MIIPHY_DELAY, NV_MIIPHY_DELAYMAX, NULL) ) {
		return -1;
	} else if( value != MII_READ ) {
		return 0;
	} else if( readRegister(NvRegMIIStatus) & NVREG_MIISTAT_ERROR ) {
		return -1;
	} else {
		return readRegister(NvRegMIIData);
	}
}

bool com_triton_forcedeth::phyReset() {
	UInt32 miiControl;
	unsigned int tries = 0;

	miiControl = miiRW(phyAddr, MII_BMSR, MII_READ);
	miiControl |= BMCR_RESET;

	if( miiRW(phyAddr, MII_BMCR, miiControl) ) {
		return false;
	}

	IOSleep(500);

	while( miiControl & BMCR_RESET ) {
		IOSleep(10);
		miiControl = miiRW(phyAddr, MII_BMCR, MII_READ);

		/* FIXME: 100 tries seem excessive */
		if (tries++ > 100)
			return false;
	}

	return true;
}


void com_triton_forcedeth::copyMacToHW()
{
	UInt32 mac[2];
	
	mac[0] = (macAddr.bytes[0] << 0) + (macAddr.bytes[1] << 8) + (macAddr.bytes[2] << 16) + (macAddr.bytes[3] << 24);
	mac[1] = (macAddr.bytes[4] << 0) + (macAddr.bytes[5] << 8);
	writeRegister(NvRegMacAddrA, mac[0]);
	writeRegister(NvRegMacAddrB, mac[1]);
}


void com_triton_forcedeth::setBufSize() {
	UInt32 mtu;
	
	mtu = interface->getMaxTransferUnit();
	IOLog("forcedeth: setBufSize/Apple wants %d bytes\n", mtu);
	
	if(mtu > NV_PKTLIMIT_2) {
		mtu = NV_PKTLIMIT_2;
	}
	
	IOLog("forcedeth: setBufSize/We can give them %d bytes\n", mtu);
	
	if( mtu <= kIOEthernetMaxPacketSize  && kIOEthernetMaxPacketSize <= NV_PKTLIMIT_2 ) {
		rxBufSz = kIOEthernetMaxPacketSize + NV_RX_HEADERS;
		IOLog("forcedeth: setBufSize/Setting with regards to kIOEthernetMaxPacketSize(%d) -- %d bytes\n", kIOEthernetMaxPacketSize, rxBufSz);
	} else {
		rxBufSz = mtu + NV_RX_HEADERS;
		IOLog("forcedeth: setBufSize/Setting with regards to mtu(%d) -- %d bytes\n", mtu, rxBufSz);
	}
}

bool com_triton_forcedeth::updateLinkSpeed() {
	int miiStatus;
	int adv, lpa;
	int newls = linkspeed;
	int newdup = duplex;
	bool retval;
	
	UInt32 controlGigabit, statusGigabit, phyReg;
	
	miiRW(phyAddr, MII_BMSR, MII_READ);
	miiStatus = miiRW(phyAddr, MII_BMSR, MII_READ);
	
	do {
		if( !(miiStatus & BMSR_LSTATUS) ) {
			newls = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_10;
			newdup = 0;
			retval = false;
			break;
		}
		
		if( !autoneg ) {
			if( fixedMode & LPA_100FULL ) {
				
			} else if (fixedMode & LPA_100HALF) {
				newls = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_100;
				newdup = 0;
			} else if (fixedMode & LPA_10FULL) {
				newls = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_10;
				newdup = 1;
			} else {
				newls = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_10;
				newdup = 0;
			}
			retval = true;
			break;
		}
		
		if( !(miiStatus & BMSR_ANEGCOMPLETE) ) {
			newls = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_10;
			newdup = 0;
			retval = false;
			break;
		}
		
		retval = true;
		
		if( gigabit == PHY_GIGABIT ) {
			controlGigabit = miiRW(phyAddr, MII_1000BT_CR, MII_READ);
			statusGigabit = miiRW(phyAddr, MII_1000BT_SR, MII_READ);
			
			if( (controlGigabit & ADVERTISE_1000FULL) && (statusGigabit & LPA_1000FULL) ) {
				newls = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_1000;
				newdup = 1;
				break;
			}
		}
		
		adv = miiRW(phyAddr, MII_ADVERTISE, MII_READ);
		lpa = miiRW(phyAddr, MII_LPA, MII_READ);
		
		// FIXME: handle parallel detection properly
		lpa = lpa & adv;
		if (lpa & LPA_100FULL) {
			newls = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_100;
			newdup = 1;
		} else if (lpa & LPA_100HALF) {
			newls = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_100;
			newdup = 0;
		} else if (lpa & LPA_10FULL) {
			newls = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_10;
			newdup = 1;
		} else if (lpa & LPA_10HALF) {
			newls = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_10;
			newdup = 0;
		} else {
			newls = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_10;
			newdup = 0;
		}
	} while (false);
	
	if( duplex == newdup && linkspeed == newls )
		return retval;
	
	duplex = newdup;
	linkspeed = newls;
	
	if( gigabit == PHY_GIGABIT ) {
		phyReg = readRegister(NvRegRandomSeed);
		phyReg &= ~(0x3FF00);
		
		if( (linkspeed & 0xFFF) == NVREG_LINKSPEED_10 )
			phyReg |= NVREG_RNDSEED_FORCE3;
		else if( (linkspeed & 0xFFF) == NVREG_LINKSPEED_100 )
			phyReg |= NVREG_RNDSEED_FORCE2;
		else if( (linkspeed & 0xFFF) == NVREG_LINKSPEED_1000 )
			phyReg |= NVREG_RNDSEED_FORCE;
		
		writeRegister(NvRegRandomSeed, phyReg);
	}
	
	phyReg = readRegister(NvRegPhyInterface);
	phyReg &= ~(PHY_HALF|PHY_100|PHY_1000);
	
	if( !duplex )
		phyReg != PHY_HALF;
	
	if( (linkspeed & NVREG_LINKSPEED_MASK) == NVREG_LINKSPEED_100 )
		phyReg |= PHY_100;
	else if( (linkspeed & NVREG_LINKSPEED_MASK) == NVREG_LINKSPEED_1000 )
		phyReg |= PHY_1000;
		
	writeRegister(NvRegPhyInterface, phyReg);
	writeRegister(NvRegMisc1, NVREG_MISC1_FORCE | ( duplex ? false : NVREG_MISC1_HD ) );
	writeRegister(NvRegLinkSpeed, linkspeed);
	
	int speed;
	IOMediumType type;
	
	if( (linkspeed & NVREG_LINKSPEED_MASK) == NVREG_LINKSPEED_10 ) {
		type = kIOMediumEthernet10BaseT;
		speed = 10;
	}
	
	if( (linkspeed & NVREG_LINKSPEED_MASK) == NVREG_LINKSPEED_100 ) {
		type = kIOMediumEthernet100BaseTX;
		speed = 100;
	}
	
	if( (linkspeed & NVREG_LINKSPEED_MASK) == NVREG_LINKSPEED_1000 ) {
		type = kIOMediumEthernet1000BaseT;
		speed = 1000;
	}
		
	if( duplex )
		type |= kIOMediumOptionFullDuplex;
	else
		type |= kIOMediumOptionHalfDuplex;
		
	if( retval ) {
		IOLog("forcedeth: Link speed now %dMbps, code 0x%x.\n", speed, linkspeed);
		setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive, IONetworkMedium::getMediumWithType(dictionary, type));
		startRx();
	} else {
		IOLog("forcedeth: Network link down.\n");
		setLinkStatus(kIONetworkLinkValid, 0);
		stopRx();
	}
	
	return retval;
}

void com_triton_forcedeth::linkChange() {
	if( updateLinkSpeed() ) {
	} else {
	}
}

void com_triton_forcedeth::linkIRQ() {
	UInt32 miiStat;
	miiStat = readRegister(NvRegMIIStatus);
	writeRegister(NvRegMIIStatus, NVREG_MIISTAT_MASK);
	if( miiStat & NVREG_MIISTAT_LINKCHANGE ) {
		linkChange();
	}
}

void com_triton_forcedeth::rxProcess() {
	UInt32 flags;
	UInt32 ckResult;
	UInt32 len;
	int i;
	
	IOPhysicalSegment vector;
	bool replaced;
	mbuf_t	pktBuf;
	
	for( ;; ) {	
		i = curRx % RX_RING;
		
		flags = OSSwapLittleToHostInt32(rxRing[i].FlagLen);
		len = descrGetLength(&rxRing[i]);

		if( debugMode )
			IOLog("forcedeth: packet %x - %x\n", flags & NV_RX_AVAIL, flags);

		if( flags & NV_RX_AVAIL )
			break;
		
		do {
			if( !(flags & NV_RX2_DESCRIPTORVALID) )
				break;
			
			if( flags & (NV_RX2_ERROR1|NV_RX2_ERROR2|NV_RX2_ERROR3) ) {
				stats->inputErrors++;
				break;
			}
			
			if( flags & NV_RX2_CRCERR ) {
				stats->inputErrors++;
				break;
			}
			
			if( flags & NV_RX2_OVERFLOW ) {
				stats->inputErrors++;
				break;
			}
			
			if( flags & NV_RX2_ERROR4 ) {
				len = getLength(rxBuf[i], len);
				if( len < 0 ) {
					stats->inputErrors++;
					break;
				}
			}
			
			if( flags & NV_RX2_FRAMINGERR ) {
				if( flags & NV_RX2_SUBTRACT1 ) {
					len--;
				}
			}
			
			if( rxCRC ) {
				flags &= NV_RX2_CHECKSUMMASK;
				
				ckResult = 0;
				
				if( flags & NV_RX2_CHECKSUMOK3 )
					ckResult |= kChecksumUDP;
					
				if( flags & NV_RX2_CHECKSUMOK2 )
					ckResult |= kChecksumTCP;
				
				if( flags & NV_RX2_CHECKSUMOK1 )
					ckResult |= kChecksumIP;
				
				/*IOLog("Ck: ");
				if( !(flags & (NV_RX2_CHECKSUMOK1)) )
					IOLog("!1 ");
				if( !(flags & (NV_RX2_CHECKSUMOK2)) )
					IOLog("!2 ");
				if( !(flags & (NV_RX2_CHECKSUMOK3)) )
					IOLog("!3");
				
				IOLog("\n");*/
				
				setChecksumResult(rxBuf[i], kChecksumFamilyTCPIP, kChecksumIP | kChecksumTCP | kChecksumUDP, ckResult);
			}
			
			pktBuf = replaceOrCopyPacket(&rxBuf[i], len, &replaced);
			if( !pktBuf ) {
				IOLog("forcedeth: failed to replaceOrCopyPacket packet!\n");
				stats->inputErrors++;
				break;
			}
			
			interface->inputPacket(pktBuf, len);
			
			stats->inputPackets++;
			
			if( replaced ) {
				if( rxMbufCursor->getPhysicalSegmentsWithCoalesce(rxBuf[i], &vector, 1) != 1 ) {
					IOLog("forcedeth: failed to replace received packet!\n");
					break;
				}
				
				rxDma[i] = (UInt32)vector.location;
				rxRing[i].PacketBuffer = OSSwapHostToLittleInt32(rxDma[i]);
			}
		} while( false );
		
		rxRing[i].FlagLen = OSSwapHostToLittleInt32(rxBufSz | NV_RX_AVAIL);
		curRx++;
	}
	interface->flushInputQueue();
}


int com_triton_forcedeth::phyInit() {
	UInt32 phyInterface, phyReserved, miiStatus, miiControl, miiControlGigabit, reg;

	reg = miiRW(phyAddr, MII_ADVERTISE, MII_READ);
	reg |= (ADVERTISE_10HALF|ADVERTISE_10FULL|ADVERTISE_100HALF|ADVERTISE_100FULL|0x800|0x400);

	if( miiRW(phyAddr, MII_ADVERTISE, reg) ) {
		IOLog("forcedeth: PHY write to advertise failed.\n");
		return PHY_ERROR;
	}

	/* get phy interface type */
	phyInterface = readRegister(NvRegPhyInterface);
	miiStatus = miiRW(phyAddr, MII_BMSR, MII_READ);
	if( miiStatus & PHY_GIGABIT ) {
		gigabit = PHY_GIGABIT;
		miiControlGigabit = miiRW(phyAddr, MII_1000BT_CR, MII_READ);
		miiControlGigabit &= ~ADVERTISE_1000HALF;
		if( phyInterface & PHY_RGMII )
			miiControlGigabit |= ADVERTISE_1000FULL;
		else
			miiControlGigabit &= ~ADVERTISE_1000FULL;

		if( miiRW(phyAddr, MII_1000BT_CR, miiControlGigabit) ) {
			IOLog("forcedeth: PHY init failed.\n");
			return PHY_ERROR;
		}
	} else {
		gigabit = 0;
	}

	// reset the phy
	if( !phyReset() ){
		IOLog("forcedeth: PHY reset failed.\n");
		return PHY_ERROR;
	}

	if( (phyOui == PHY_OUI_CICADA) && (phyInterface & PHY_RGMII) ) {
		phyReserved = miiRW(phyAddr, MII_RESV1, MII_READ);
		phyReserved &= ~(PHY_INIT1 | PHY_INIT2);
		phyReserved |= (PHY_INIT3 | PHY_INIT4);

		if( miiRW(phyAddr, MII_RESV1, phyReserved) ) {
			IOLog("forcedeth: PHY init failed.\n");
			return PHY_ERROR;
		}

		phyReserved = miiRW(phyAddr, MII_NCONFIG, MII_READ);
		phyReserved |= PHY_INIT5;

		if( miiRW(phyAddr, MII_NCONFIG, phyReserved) ) {
			IOLog("forcedeth: PHY init failed.\n");
			return PHY_ERROR;
		}
	}
	if( phyOui == PHY_OUI_CICADA ) {
		phyReserved = miiRW(phyAddr, MII_SREVISION, MII_READ);
		phyReserved |= PHY_INIT6;
		if( miiRW(phyAddr, MII_SREVISION, phyReserved) ) {
			IOLog("forcedeth: PHY init failed.\n");
			return PHY_ERROR;
		}
	}

	// restart auto negotiation
	miiControl = miiRW(phyAddr, MII_BMCR, MII_READ);
	miiControl |= (BMCR_ANRESTART | BMCR_ANENABLE);
	if( miiRW(phyAddr, MII_BMCR, miiControl) ) {
		return PHY_ERROR;
	}
	
	IONetworkMedium *medium;
	
	dictionary = OSDictionary::withCapacity(5);
	
	medium = IONetworkMedium::medium(kIOMediumEthernetAuto, 1000 * 1000000);
	IONetworkMedium::addMedium(dictionary, medium);

	if( miiStatus & BMSR_100FULL2 ) {
		medium = IONetworkMedium::medium(kIOMediumEthernet100BaseT2 | kIOMediumOptionFullDuplex, 100 * 1000000);
		IONetworkMedium::addMedium(dictionary, medium);
	}
	
	if( miiStatus & BMSR_100HALF2 ) {
		medium = IONetworkMedium::medium(kIOMediumEthernet100BaseT2 | kIOMediumOptionHalfDuplex, 100 * 1000000);
		IONetworkMedium::addMedium(dictionary, medium);
	}

	if( miiStatus & BMSR_10HALF ) {
		medium = IONetworkMedium::medium(kIOMediumEthernet10BaseT | kIOMediumOptionHalfDuplex, 10 * 1000000);
		IONetworkMedium::addMedium(dictionary, medium);
	}
	
	if( miiStatus & BMSR_10FULL ) {
		medium = IONetworkMedium::medium(kIOMediumEthernet10BaseT | kIOMediumOptionFullDuplex, 10 * 1000000);
		IONetworkMedium::addMedium(dictionary, medium);
	}
	
	if( miiStatus & BMSR_100FULL ) {
		medium = IONetworkMedium::medium(kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex, 100 * 1000000);
		IONetworkMedium::addMedium(dictionary, medium);
	}
	
	if( miiStatus & BMSR_100HALF ) {
		medium = IONetworkMedium::medium(kIOMediumEthernet100BaseTX | kIOMediumOptionHalfDuplex, 100 * 1000000);
		IONetworkMedium::addMedium(dictionary, medium);
	}
	
	if( miiStatus & BMSR_100BASE4 ) {
		medium = IONetworkMedium::medium(kIOMediumEthernet100BaseT4, 100 * 1000000);
		IONetworkMedium::addMedium(dictionary, medium);
	}
	
	if( gigabit ) {
		medium = IONetworkMedium::medium(kIOMediumEthernet1000BaseT, 1000 * 1000000);
		IONetworkMedium::addMedium(dictionary, medium);
	}
	
	publishMediumDictionary(dictionary);
	setSelectedMedium(IONetworkMedium::getMediumWithType(dictionary, kIOMediumEthernetAuto));
	
	return 0;
}

bool com_triton_forcedeth::initRx() {
	IOPhysicalSegment vector;
	mbuf_t newPacket;

	curRx = RX_RING;
	for( int i = 0; i < RX_RING; i++ ) {
		newPacket = allocatePacket(rxBufSz + NV_RX_ALLOC_PAD);
			
		if( !newPacket ) {
			return false;
		}
		
		if( rxMbufCursor->getPhysicalSegmentsWithCoalesce(newPacket, &vector, 1) != 1 ) {
			freePacket(newPacket);
			rxBuf[i] = NULL;
			return false;
		}
		
		rxBuf[i] = newPacket;
		rxDma[i] = (UInt32)vector.location;
		
		rxRing[i].PacketBuffer = OSSwapHostToLittleInt32(rxDma[i]);
		rxRing[i].FlagLen = OSSwapHostToLittleInt32(rxBufSz | NV_RX_AVAIL);
	}
	
	return true;
}

void com_triton_forcedeth::initTx() {
	nextTx = nicTx;
	for( int i = 0; i < TX_RING; i++ ) {
		txRing[i].FlagLen = 0;
		txRing[i].PacketBuffer = 0;
		txBuf[i] = NULL;
		txDma[i] = NULL;
	}
}

bool com_triton_forcedeth::initRing() {
	initTx();
	return initRx();
}

bool com_triton_forcedeth::releaseTxPkt(int index) {
	txDma[index] = 0;
	
	if( txBuf[index] ) {
		freePacket(txBuf[index]);
		txBuf[index] = NULL;
		return true;
	} else {
		return false;
	}
}

void com_triton_forcedeth::drainTx() {
	for( int i = 0; i < TX_RING; i++ ) {
		txRing[i].FlagLen = 0;
		
		if( releaseTxPkt(i) ) {
			outputStats->dropCount++;
		}
	}
}

void com_triton_forcedeth::drainRx() {
	for( int i = 0; i < RX_RING; i++ ) {
		rxRing[i].FlagLen = 0;
		freePacket(rxBuf[i]);
		rxBuf[i] = NULL;
	}
}

void com_triton_forcedeth::drainRing() {
	drainTx();
	drainRx();
}

void com_triton_forcedeth::startRx() {
	if( readRegister(NvRegReceiverControl) & NVREG_RCVCTL_START ) {
		writeRegister(NvRegReceiverControl, 0);
	}
	
	writeRegister(NvRegLinkSpeed, linkspeed);
	writeRegister(NvRegReceiverControl, NVREG_RCVCTL_START);
}

void com_triton_forcedeth::stopRx() {
	writeRegister(NvRegReceiverControl, 0);
	regDelay(NvRegReceiverStatus, NVREG_RCVSTAT_BUSY, 0, NV_RXSTOP_DELAY1, NV_RXSTOP_DELAY1MAX, "forcedethnet: Receiver status remained busy during attempted shutdown");
	IODelay(NV_RXSTOP_DELAY2);
	writeRegister(NvRegLinkSpeed, 0);
}

void com_triton_forcedeth::startTx() {
	writeRegister(NvRegTransmitterControl, NVREG_XMITCTL_START);
}

void com_triton_forcedeth::stopTx() {
	writeRegister(NvRegTransmitterControl, 0);
	regDelay(NvRegTransmitterStatus, NVREG_XMITSTAT_BUSY, 0, NV_TXSTOP_DELAY1, NV_TXSTOP_DELAY1MAX, "forcedethnet: Transmitter status remained busy during attempted shutdown");
	IODelay(NV_TXSTOP_DELAY2);
	writeRegister(NvRegUnknownTransmitterReg, 0);
}

void com_triton_forcedeth::txrxReset() {
	writeRegister(NvRegTxRxControl, NVREG_TXRXCTL_BIT2 | NVREG_TXRXCTL_RESET | txrxCtlBits);
	IODelay(NV_TXRX_RESET_DELAY);
	writeRegister(NvRegTxRxControl, NVREG_TXRXCTL_BIT2 | txrxCtlBits);
}

int com_triton_forcedeth::getLength(mbuf_t buf, int len) {
	return len;
	
	/*int hdrLen;
	
	hdrLen = mbuf_pkthdr_len(buf);
	
	if( hdrLen <= len ) {
		return hdrLen;
	} else {
		return -1;
	}*/
}


void com_triton_forcedeth::txDone() {
	unsigned int i;
	UInt32 flags;
	mbuf_t pktBuf;
	
	while( nicTx != nextTx ) {
		i = nicTx % TX_RING;
		
		flags = OSSwapLittleToHostInt32(txRing[i].FlagLen);
		
		if( flags & NV_TX_VALID )
			break;
			
		if( flags & NV_TX2_LASTPACKET ) {
			pktBuf = txBuf[i];
			if( flags & (NV_TX2_RETRYERROR|NV_TX2_CARRIERLOST|NV_TX2_LATECOLLISION|NV_TX2_UNDERFLOW|NV_TX2_ERROR)) {
				stats->outputErrors++;
				
				if( flags & NV_TX2_UNDERFLOW ) {

				}
				
				if( flags & NV_TX2_CARRIERLOST ) {

				}
				
				if( flags & NV_TX2_LATECOLLISION ) {
					stats->collisions++;
				}
			} else {
				stats->outputPackets++;
			}

			if( debugMode )
				IOLog("forcedeth: Done with packet with flags %x\n", flags);
		} else {
			if( debugMode )
				IOLog("forcedeth: Continuing packet with flags %x\n", flags);
		}
		
		releaseTxPkt(i);
		nicTx++;
	}
	
	if( (nextTx - nicTx) < TX_LIMIT_START )
		outputQueue->service();
}

void com_triton_forcedeth::updateMulticast(UInt32 pff, UInt32 addrA, UInt32 addrB, UInt32 maskA, UInt32 maskB) {

	addrA |= NVREG_MCASTADDRA_FORCE;
	pff |= NVREG_PFF_ALWAYS;
	//IOLockLock(lock);
	stopRx();
	writeRegister(NvRegMulticastAddrA, addrA);
	writeRegister(NvRegMulticastAddrA, addrB);
	writeRegister(NvRegMulticastMaskA, maskA);
	writeRegister(NvRegMulticastMaskB, maskB);
	writeRegister(NvRegPacketFilterFlags, pff);
	startRx();
	
	//IOLockUnlock(lock);
}
