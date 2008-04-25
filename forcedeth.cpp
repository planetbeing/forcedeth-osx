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
 
// REQUIRED! This macro defines the class's constructors, destructors,
// and several other methods I/O Kit requires. Do NOT use super as the
// second parameter. You must use the literal name of the superclass.
OSDefineMetaClassAndStructors(com_triton_forcedeth, IOEthernetController)

void com_triton_forcedeth::nicIRQ() {
	int i;
	UInt32 events;
	uint32_t sec;
	uint32_t usec;
	UInt64 curTime;
	
	clock_get_system_microtime(&sec, &usec);
	curTime = (sec * 1000000UL) + usec;
	
	if(needLinkTimer && (curTime >= linkTimeout)) {
		linkChange();
		linkTimeout = curTime + LINK_TIMEOUT;
	}
	
	for( i = 0; ; i++ ) {
		events = readRegister(NvRegIrqStatus) & NVREG_IRQSTAT_MASK;
		writeRegister(NvRegIrqStatus, NVREG_IRQSTAT_MASK);
		
		if( !(events & irqMask) )
			break;
		
		//IOLockLock(lock);
		txDone();
		//IOLockUnlock(lock);
		
		//IOLockLock(lock);
		rxProcess();
		//IOLockUnlock(lock);
		
		if( events & NVREG_IRQ_LINK ) {
			//IOLockLock(lock);
			linkIRQ();
			//IOLockUnlock(lock);
		}
		
		
		if( events & NVREG_IRQ_TX_ERR ) {
			IOLog("forcedeth: Received irq with events 0x%x. Probably TX fail.\n", events);
		}
		
		if( events & NVREG_IRQ_UNKNOWN ) {
			IOLog("forcedeth: Received irq with unknown events 0x%x. Not great.\n", events);
		}
		
		if( i > maxInterruptWork ) {
			//IOLockLock(lock);
			writeRegister(NvRegIrqMask, 0);
			
			if( !inShutdown )
				nicPollTimer->setTimeoutMS(POLL_WAIT);
			
			//IOLockUnlock(lock);
			break;
		}
	}
}
void com_triton_forcedeth::doNicPoll() {
	interrupt->disable();
	writeRegister(NvRegIrqMask, irqMask);
	nicIRQ();
	interrupt->enable();
}

UInt32 com_triton_forcedeth::outputPacket(mbuf_t buf, void *param) {
	int maxSegments;
	int segments;
	int i;
	int totalLength;
	UInt32 checksumDemanded;
	unsigned int startNr;
	unsigned int nr;
	IOPhysicalSegment vector[TX_LIMIT_STOP];
	
	//IOLockLock(lock);

	if( (nextTx - nicTx) > TX_LIMIT_STOP ) {
		//IOLockUnlock(lock);
		IOLog("forcedeth: NIC ring full, stalling.\n");
		return kIOReturnOutputStall;
	}

	if( txCRC )
		maxSegments = TX_LIMIT_STOP + nicTx - nextTx;
	else
		maxSegments = 1;

	if( (segments = txMbufCursor->getPhysicalSegmentsWithCoalesce(buf, &vector[0], maxSegments)) == 0 ) {
		//IOLockUnlock(lock);
		IOLog("forcedeth: NIC ring full, stalling.\n");
		return kIOReturnOutputStall;
	}
	
	startNr = nextTx % TX_RING;
	totalLength = 0;

	for( i = 0; i < segments; i++ ) {
		nr = nextTx % TX_RING;
		
		txBuf[nr] = NULL;
		txDma[nr] = (UInt32)vector[i].location;
		txRing[nr].PacketBuffer = OSSwapHostToLittleInt32(txDma[nr]);

		txRing[nr].FlagLen = OSSwapHostToLittleInt32((vector[i].length-1) | txFlags);
		
		if( i > 0 )
			totalLength += vector[i].length;
			
		nextTx++;
	}
	
	txBuf[nr] = buf;
	txRing[nr].FlagLen |=  NV_TX2_LASTPACKET;

	if( txCRC ) {
		getChecksumDemand(buf, kChecksumFamilyTCPIP, &checksumDemanded);
		
		if( checksumDemanded & kChecksumIP )
			txRing[startNr].FlagLen |= NV_TX2_CHECKSUM_L3;

		if( (checksumDemanded & kChecksumTCP) || (checksumDemanded & kChecksumUDP) )
			txRing[startNr].FlagLen |= NV_TX2_CHECKSUM_L4;
	} else {
		checksumDemanded = 0;
	}
		
	if( debugMode )
		IOLog("forcedeth: output, seg: %d/%d, demand ck: %x, start: %x, end: %x - %x\n", segments, maxSegments, checksumDemanded, txRing[startNr].FlagLen, txRing[nr].FlagLen, totalLength << NV_TX2_TSO_SHIFT);

	//IOLockUnlock(lock);
	
	writeRegister(NvRegTxRxControl, NVREG_TXRXCTL_KICK | txrxCtlBits);
	return kIOReturnOutputSuccess;
}

bool com_triton_forcedeth::init(OSDictionary *dict)
{
	OSBoolean *boolVal;
    bool res = super::init(dict);
	IOLog("forcedeth: Version 0.3c\n");
    IOLog("forcedeth: Initializing.\n");
	
	//lock = IOLockAlloc();
	ifEnabled = false;

	interface = NULL;
	rxMbufCursor = NULL;
	txMbufCursor = NULL;
	rxRing = NULL;
	map = NULL;
	device = NULL;
	nicPollTimer = NULL;
	workLoop = NULL;
	dictionary = NULL;
	
	if( (boolVal = (OSBoolean *)getProperty("ChecksumReceive")) && boolVal->isTrue() ) {
		rxCRC = true;
	} else {
		rxCRC = false;
	}
	
	if( (boolVal = (OSBoolean *)getProperty("ChecksumTransmit")) && boolVal->isTrue() ) {
		txCRC = true;
	} else {
		txCRC = false;
	}
	
	if( (boolVal = (OSBoolean *)getProperty("MultipleSegments")) && boolVal->isTrue() ) {
		multSeg = true;
	} else {
		multSeg = false;
	}
	
	if( (boolVal = (OSBoolean *)getProperty("IRQTimer")) && boolVal->isTrue() ) {
		timerIRQ = true;
	} else {
		timerIRQ = false;
	}
	
	if( (boolVal = (OSBoolean *)getProperty("Debug")) && boolVal->isTrue() ) {
		debugMode = true;
	} else {
		debugMode = false;
	}
	
    return res;
}
 
void com_triton_forcedeth::free(void)
{
    IOLog("forcedeth: Freeing.\n");
	
	//IOLockFree(lock);
	
    super::free();
}

IOService *com_triton_forcedeth::probe(IOService *provider, SInt32
*score)
{
	int i;
	bool found;
	UInt32 valid[][2] = {{0x0057,0x10DE},{0x0037,0x10DE},{0x0038,0x10DE},{0x0056,0x10DE},{0x0057,0x10DE},{0x0086,0x10DE},{0x008C,0x10DE},{0x00DF,0x10DE},{0x00E6,0x10DE},{0x0372,0x10DE},{0x0373,0x10DE},{0x03EF,0x10DE},{0,0}};
	IOService *res = super::probe(provider, score);

    IOLog("forcedeth: Probing.\n");

	do {
		if( (device = OSDynamicCast(IOPCIDevice, provider)) == 0 ) {
			IOLog("forcedeth: Cannot retrieve PCI device.\n");
			break;
		}
		
		device->retain();
		
		if( (device->open(this)) == 0 ) {
			IOLog("forcedeth: Failed to open PCI device for probing.\n");
			break;
		}
		
		vendorID = device->configRead16(kIOPCIConfigVendorID);
		deviceID = device->configRead16(kIOPCIConfigDeviceID);
		subVendID = device->configRead16(kIOPCIConfigSubSystemVendorID);
		subDevID = device->configRead16(kIOPCIConfigSubSystemID);
		revisionID = device->configRead8(kIOPCIConfigRevisionID);
			
		i = 0;
		found = false;
		while( valid[i][0] ) {
			if( valid[i][0] == deviceID && valid[i][1] == vendorID ) {
				found = true;
				break;
			}
			i++;
		}
		
		if( !found ) {
			if( vendorID == 0x10de ) {
				if( deviceID == 0x01c3 || deviceID == 0x0066 || deviceID == 0x00d6 || deviceID == 0x0268 || deviceID == 0x0269 ) {
					IOLog("forcedeth: WARNING - your nVidia device 0x%04X:0x%04X is not known to work, but it could be supported in the future. Please contact the coder. Continuing anyway...\n", vendorID, deviceID);
				} else {
					IOLog("forcedeth: WARNING - your nVidia device 0x%04X:0x%04X is not known to work. Continuing anyway...\n", vendorID, deviceID);
				}
			} else {
				IOLog("forcedeth: WARNING - your device 0x%04X:0x%04X is not known to work, it's not even made by nVidia so it's HIGHLY UNLIKELY that it will work. Continuing anyway...\n", vendorID, deviceID);
			}
		} else {
			*score = 100;
		}

		device->close(this);

		return res;
	} while(false);
	
	return res;
}

bool com_triton_forcedeth::start(IOService *provider)
{
	int i;
	IOMemoryDescriptor *mem;
	int memCount;
	int memSize;
	int powerstate;

    bool res = super::start(provider);
    IOLog("forcedeth: Starting.\n");

	do {
		interrupt = NULL;
		nicPollTimer = NULL;

		if( (device->open(this)) == 0 ) {
			IOLog("forcedeth: Failed to open PCI device.\n");
			break;
		}
		
		device->setBusMasterEnable(true);
		device->setMemoryEnable(true);
		device->setIOEnable(false);

		pktLimit = NV_PKTLIMIT_2;
		txrxCtlBits = NVREG_TXRXCTL_DESC_2 | NVREG_TXRXCTL_RXCHECK;

		IOLog("forcedeth: PCI system 0x%04X:0x%04X, subsystem 0x%04X:0x%04X revision 0x%02X opened.\n", vendorID, deviceID, subVendID, subDevID, revisionID);

		memSize = NV_PCI_REGSZ;
		if (deviceID == 0x03EF || deviceID == 0x0373 || deviceID == 0x0372)
			memSize = NV_PCI_REGSZ_VER3;

		memCount = device->getDeviceMemoryCount();
	
		for( i = 0; i < memCount; i++ ) {
			mem = device->getDeviceMemoryWithIndex(i);
			
			if( mem->getLength() >= memSize ) {
				if( (map = device->mapDeviceMemoryWithIndex(i)) == 0 ) {
					IOLog("forcedeth: Cannot map device memory.\n");
					i = memCount; 
					break;
				} else {
					break;
				}
			}
		}
		
		if( i >= memCount ) {
			IOLog("forcedeth: Unable to find appropriate register window.\n");
			break;
		}
		
		IOLog("forcedeth: Mapped from 0x%X of length %d.\n", device->configRead32(kIOPCIConfigBaseAddress0), map->getLength());
		
		map->retain();
		
		baseAddress = (char*)map->getVirtualAddress();
		
		rxMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(kIOEthernetMaxPacketSize, 1);
		txMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(NV_TX2_TSO_MAX_SIZE, TX_LIMIT_STOP);
		
		IOPhysicalAddress addr;
		if( !(rxRing = (RingDesc *)IOMallocContiguous(sizeof(RingDesc) * (RX_RING + TX_RING), 4, &addr)) ) {
			IOLog("forcedeth: Could not allocate %d bytes of contiguous memory for DMA.\n",  sizeof(struct RingDesc) * (RX_RING + TX_RING));
			break;
		}

		ringAddr = (UInt32) addr;
		txRing = &rxRing[RX_RING];
		
		IOLog("forcedeth: Allocated %d bytes of contiguous memory for DMA: rx at 0x%x, tx at 0x%x, wired at 0x%x.\n",  sizeof(struct RingDesc) * (RX_RING + TX_RING), rxRing, txRing, ringAddr);
		
		origMac[0] = readRegister(NvRegMacAddrA);
		origMac[1] = readRegister(NvRegMacAddrB);
		
		if (memSize == NV_PCI_REGSZ_VER3) {
			/* mac address is already in correct order */
			macAddr.bytes[0] = (origMac[0] >> 0) & 0xff;
			macAddr.bytes[1] = (origMac[0] >> 8) & 0xff;
			macAddr.bytes[2] = (origMac[0] >> 16) & 0xff;
			macAddr.bytes[3] = (origMac[0] >> 24) & 0xff;
			macAddr.bytes[4] = (origMac[1] >> 0) & 0xff;
			macAddr.bytes[5] = (origMac[1] >> 8) & 0xff;
		} else {
			macAddr.bytes[0] = (origMac[1] >> 8) & 0xff;
			macAddr.bytes[1] = (origMac[1] >> 0) & 0xff;
			macAddr.bytes[2] = (origMac[0] >> 24) & 0xff;
			macAddr.bytes[3] = (origMac[0] >> 16) & 0xff;
			macAddr.bytes[4] = (origMac[0] >> 8) & 0xff;
			macAddr.bytes[5] = (origMac[0] >> 0) & 0xff;
		}
		
		if( (origMac[0] == 0 && macAddr.bytes[0] == 0 && macAddr.bytes[1] == 0) || (origMac[0] == 0xffffffff && macAddr.bytes[0] == 0xff && macAddr.bytes[1] == 0xff) ) {
			IOLog("forcedeth: MAC address %02X:%02X:%02X:%02X:%02X:%02X given by device is not valid.\n",	macAddr.bytes[0],
																											macAddr.bytes[1],
																											macAddr.bytes[2],
																											macAddr.bytes[3],
																											macAddr.bytes[4],
																											macAddr.bytes[5]);
			break;
		}
		
		IOLog("forcedeth: Found nForce4 LAN with MAC: %02X:%02X:%02X:%02X:%02X:%02X.\n",	macAddr.bytes[0],
																						macAddr.bytes[1],
																						macAddr.bytes[2],
																						macAddr.bytes[3],
																						macAddr.bytes[4],
																						macAddr.bytes[5]);	
		
		
		// disable Wakeup on LAN
		writeRegister(NvRegWakeUpFlags, 0);
		wolEnabled = false;
		
		if (deviceID == 0x03EF || deviceID == 0x0373 || deviceID == 0x0372 || deviceID == 0x0269 || deviceID == 0x0268) 
		{
			IOLog("forcedeth: Taking PHY and NIC out of low power mode\n");
			powerstate = readRegister(NvRegPowerState2);
			powerstate &= ~NVREG_POWERSTATE2_POWERUP_MASK;
			if((deviceID == 0x0269 || deviceID == 0x0268) && revisionID >= 0xA3) {
				powerstate |= NVREG_POWERSTATE2_POWERUP_REV_A3;
			}
			writeRegister(NvRegPowerState2, (UInt32) powerstate);
		}

		txFlags = NV_TX2_VALID;

		irqMask = NVREG_IRQMASK_THROUGHPUT;
		
		if( timerIRQ )
			irqMask |= NVREG_IRQ_TIMER;


		needLinkTimer = true;
		
		{
			uint32_t sec;
			uint32_t usec;
			UInt64 curTime;
			
			clock_get_system_microtime(&sec, &usec);
			curTime = (sec * 1000000UL) + usec;
			
			if(needLinkTimer && (curTime >= linkTimeout)) {
				linkIRQ();
				linkTimeout = curTime + LINK_TIMEOUT;
			}
		}

		phyAddr = 0;
		phyOui = 0;

		int i;
		for( i = 1; i <= 31; i++ ) {
			int id1, id2;
			int nPhyAddr = i & 0x1F;
			
			//IOLockLock(lock);
			id1 = miiRW(nPhyAddr, MII_PHYSID1, MII_READ);
			//IOLockUnlock(lock);

			if( id1 < 0 || id1 == 0xffff )
				continue;
			
			//IOLockLock(lock);
			id2 = miiRW(nPhyAddr, MII_PHYSID2, MII_READ);;
			//IOLockUnlock(lock);

			if( id2 < 0 || id2 == 0xffff )
				continue;

			id1 = (id1 & PHYID1_OUI_MASK) << PHYID1_OUI_SHFT;
			id2 = (id2 & PHYID2_OUI_MASK) >> PHYID2_OUI_SHFT;

			IOLog("forcedeth: Found PHY 0x%04x:0x%04x at address %d.\n", id1, id2, nPhyAddr);
			phyAddr = nPhyAddr;
			phyOui = id1 | id2;
		}

		if( phyOui == 0 && phyAddr == 0 ) {
			IOLog("forcedeth: Could not find a valid PHY.\n");
			break;
		}

		if( phyInit() != 0 ) {
			IOLog("forcedeth: Failed to initialize PHY.\n");
			break;
		}

		// Set some defaults
		linkspeed = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_10;
		duplex = false;
		autoneg = true;
		
		workLoop = getWorkLoop();
		workLoop->retain();
		
		if(!workLoop) {
			IOLog("forcedeth: Failed to get workloop.\n");
		}

		nicPollTimer = IOTimerEventSource::timerEventSource(this, OSMemberFunctionCast(IOTimerEventSource::Action, this, &com_triton_forcedeth::doNicPoll));
		if( workLoop->addEventSource(nicPollTimer) != kIOReturnSuccess ) {
			IOLog("forcedeth: Cannot set up nic poll timer event.\n");
			break;
		}

		interrupt = IOInterruptEventSource::interruptEventSource(this, OSMemberFunctionCast(IOInterruptEventSource::Action, this, &com_triton_forcedeth::nicIRQ), device, 0);
		if( workLoop->addEventSource(interrupt) != kIOReturnSuccess ) {
			IOLog("forcedeth: Could not set up interrupt!\n");
			break;
		}

		attachInterface(&interface);

		return res;
	} while (false);
	
	com_triton_forcedeth::stop(provider);
	return res;
}
 
IOReturn com_triton_forcedeth::enable(IOService *provider)
{
	bool ret;
	UInt32 i;
	
	IOLog("forcedeth: Enabling... ");
	
	do {	
		// 1) erase previous misconfiguration
		// 4.1-1: stop adapter: ignored, 4.3 seems to be overkill
		promiscuousMode = false;
		multicastMode = false;
		writeRegister(NvRegMulticastAddrA, NVREG_MCASTADDRA_FORCE);
		writeRegister(NvRegMulticastMaskA, 0);
		writeRegister(NvRegMulticastMaskB, 0);
		writeRegister(NvRegPacketFilterFlags, 0);
		writeRegister(NvRegTransmitterControl, 0);
		writeRegister(NvRegReceiverControl, 0);
		writeRegister(NvRegAdapterControl, 0);

		IOLog("1");

		// 2) initialize descriptor rings
		nicTx = 0;
		curRx = 0;
		linkspeed = 0;
		
		setBufSize();
		if( !initRing() )
			break;

		writeRegister(NvRegLinkSpeed, 0);
		writeRegister(NvRegUnknownTransmitterReg, 0);
		txrxReset();
		writeRegister(NvRegUnknownSetupReg6, 0);
		inShutdown = 0;
		
		
		IOLog(" 2");
		
		// 3) set mac address
		copyMacToHW();
		
		IOLog(" 3");
		
		// 4) give hw rings
		writeRegister(NvRegRxRingPhysAddr, (UInt32) ringAddr);
		writeRegister(NvRegTxRingPhysAddr, (UInt32)(ringAddr) + RX_RING * sizeof(struct RingDesc));
		writeRegister(NvRegRingSizes, ((RX_RING-1) << NVREG_RINGSZ_RXSHIFT) + ((TX_RING-1) << NVREG_RINGSZ_TXSHIFT));
		
		IOLog(" 4");
		
		// 5) continue setup
		writeRegister(NvRegLinkSpeed, linkspeed);
		writeRegister(NvRegUnknownSetupReg3, NVREG_UNKSETUP3_VAL1);
		writeRegister(NvRegTxRxControl, txrxCtlBits);
		writeRegister(NvRegTxRxControl, NVREG_TXRXCTL_BIT1 | txrxCtlBits);
		regDelay(NvRegUnknownSetupReg5, NVREG_UNKSETUP5_BIT31, NVREG_UNKSETUP5_BIT31, NV_SETUP5_DELAY, NV_SETUP5_DELAYMAX, "forcedeth: SetupReg5, Bit 31 remained off\n");
		writeRegister(NvRegUnknownSetupReg4, 0);
		writeRegister(NvRegIrqStatus, NVREG_IRQSTAT_MASK);
		writeRegister(NvRegMIIStatus, NVREG_MIISTAT_MASK2);
		
		IOLog(" 5");
		
		// 6) continue setup
		writeRegister(NvRegMisc1, NVREG_MISC1_FORCE | NVREG_MISC1_HD);
		writeRegister(NvRegTransmitterStatus, readRegister(NvRegTransmitterStatus));
		writeRegister(NvRegPacketFilterFlags, NVREG_PFF_ALWAYS);
		writeRegister(NvRegOffloadConfig, rxBufSz);
		
		writeRegister(NvRegReceiverStatus, readRegister(NvRegReceiverStatus));
		writeRegister(NvRegRandomSeed, NVREG_RNDSEED_FORCE | (random() & NVREG_RNDSEED_MASK));
		writeRegister(NvRegUnknownSetupReg1, NVREG_UNKSETUP1_VAL);
		writeRegister(NvRegUnknownSetupReg2, NVREG_UNKSETUP2_VAL);
		writeRegister(NvRegPollingInterval, NVREG_POLL_DEFAULT_THROUGHPUT);
		writeRegister(NvRegUnknownSetupReg6, NVREG_UNKSETUP6_VAL);
		writeRegister(NvRegAdapterControl, (phyAddr << NVREG_ADAPTCTL_PHYSHIFT) | NVREG_ADAPTCTL_PHYVALID | NVREG_ADAPTCTL_RUNNING);
		writeRegister(NvRegMIISpeed, NVREG_MIISPEED_BIT8 | NVREG_MIIDELAY);
		writeRegister(NvRegUnknownSetupReg4, NVREG_UNKSETUP4_VAL);
		writeRegister(NvRegWakeUpFlags, NVREG_WAKEUPFLAGS_VAL);
		
		IOLog(" 6");
		
		i = readRegister(NvRegPowerState);
		if( (i & NVREG_POWERSTATE_POWEREDUP) == 0 )
			writeRegister(NvRegPowerState, NVREG_POWERSTATE_POWEREDUP | i);
		IODelay(10);
		writeRegister(NvRegPowerState, readRegister(NvRegPowerState) | NVREG_POWERSTATE_VALID);
		
		writeRegister(NvRegIrqMask, 0);
		writeRegister(NvRegMIIStatus, NVREG_MIISTAT_MASK2);
		writeRegister(NvRegIrqStatus, NVREG_IRQSTAT_MASK);
		
		IONetworkData *statsData = interface->getNetworkData(kIONetworkStatsKey);
		statsData->setAccessTypes(kIONetworkDataAccessTypeRead | kIONetworkDataAccessTypeWrite | kIONetworkDataAccessTypeReset | kIONetworkDataAccessTypeSerialize);
		stats = (IONetworkStats*)statsData->getBuffer();
		
		IONetworkData *outputStatsData = outputQueue->getStatisticsData();
		outputStatsData->setAccessTypes(kIONetworkDataAccessTypeRead | kIONetworkDataAccessTypeWrite | kIONetworkDataAccessTypeReset | kIONetworkDataAccessTypeSerialize);
		outputStats = (IOOutputQueueStats*)outputStatsData->getBuffer();
		
		IOLog(" 7\n");
		
		// Enable timer

		// Enable interrupts;
		interrupt->enable();
		
		writeRegister(NvRegIrqMask, irqMask);
		
		//IOLockLock(lock);
		writeRegister(NvRegMulticastAddrA, NVREG_MCASTADDRA_FORCE);
		writeRegister(NvRegMulticastAddrB, 0);
		writeRegister(NvRegMulticastMaskA, 0);
		writeRegister(NvRegMulticastMaskB, 0);
		writeRegister(NvRegPacketFilterFlags, NVREG_PFF_ALWAYS | NVREG_PFF_MYADDR);
		
		{
			UInt32 miistat;
			miistat = readRegister(NvRegMIIStatus);
			writeRegister(NvRegMIIStatus, NVREG_MIISTAT_MASK);
		}
		
		IOLog("forcedeth: Starting transmit/receive engines\n");
		
		linkspeed = 0;
		ret = updateLinkSpeed();
		startRx();
		startTx();
		
		outputQueue->start();
		
		//IOLockUnlock(lock);
		
		ifEnabled = true;
		
		return kIOReturnSuccess;
	} while (false);
	
	drainRing();
	return kIOReturnError;
}

IOReturn com_triton_forcedeth::disable(IOService *provider)
{
	IOLog("forcedeth: Disabling\n");
	
	setLinkStatus(kIONetworkLinkValid, 0);
	
	//IOLockLock(lock);
	inShutdown = 1;
	// Need to synchronize irq?
	//IOLockUnlock(lock);
	
	outputQueue->flush();
	outputQueue->stop();
	
	//IOLockLock(lock);
	stopTx();
	stopRx();
	txrxReset();
	
	writeRegister(NvRegIrqMask, 0);
	//IOLockUnlock(lock);
	
	if( nicPollTimer ) {
		nicPollTimer->cancelTimeout();
	}

	if( interrupt ) {
		interrupt->disable();
	}
	
	drainRing();
	
	if( wolEnabled )
		startTx();
	
	writeRegister(NvRegMacAddrA, origMac[0]);
	writeRegister(NvRegMacAddrB, origMac[1]);
	
	ifEnabled = false;
	
	return kIOReturnSuccess;
}

void com_triton_forcedeth::stop(IOService *provider)
{
    IOLog("forcedeth: Stopping\n");

	if( interface ) {
		detachInterface(interface);
		interface = NULL;
	}
	
	if( nicPollTimer ) {
		workLoop->removeEventSource(nicPollTimer);
		nicPollTimer->release();
	}

	if( interrupt ) {
		workLoop->removeEventSource(interrupt);
		interrupt->release();
	}
	
	if( rxMbufCursor )
		rxMbufCursor->release();
		
	if( txMbufCursor )
		txMbufCursor->release();
	
	if( rxRing )
		IOFreeContiguous(rxRing, sizeof(struct RingDesc) * (RX_RING + TX_RING));
	
	if( map ) {
		map->release();
		map = NULL;
	}
	
	if( device ) {
		device->close(this);
		device->release();
		device = NULL;
	}
	
	if( workLoop ) {
		workLoop->release();
	}
	
	if( dictionary )
		dictionary->release();

    super::stop(provider);
}

IOReturn com_triton_forcedeth::getHardwareAddress(IOEthernetAddress * addrP)
{
	if( map ) {
		*addrP = macAddr;
		return kIOReturnSuccess;
	} else {
		return kIOReturnError;
	}
}

IOReturn com_triton_forcedeth::setHardwareAddress(const IOEthernetAddress * addrP)
{	

	macAddr = *addrP;;
	if( ifEnabled ) {
		outputQueue->stop();
		//IOLockLock(lock);
		stopRx();
		
		copyMacToHW();
		
		startRx();
		//IOLockUnlock(lock);
		outputQueue->start();
		return kIOReturnSuccess;
	} else {
		copyMacToHW();
		return kIOReturnSuccess;
	}
}

IOReturn com_triton_forcedeth::setWakeOnMagicPacket(bool active)
{	
	//IOLockLock(lock);
	if( active ) {
		writeRegister(NvRegWakeUpFlags, NVREG_WAKEUPFLAGS_ENABLE);
		wolEnabled = true;
	} else {
		writeRegister(NvRegWakeUpFlags, 0);
		wolEnabled = false;
	}
	//IOLockUnlock(lock);
	return kIOReturnSuccess;
}

IOOutputQueue *com_triton_forcedeth::createOutputQueue() {
	outputQueue = IOGatedOutputQueue::withTarget(this, getWorkLoop(), TX_LIMIT_START);
	return outputQueue;
}

IOReturn com_triton_forcedeth::getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput) {
	if( checksumFamily != kChecksumFamilyTCPIP ) {
		IOLog("forcedeth: Operating system wants information for unknown checksum family.\n");
		return kIOReturnUnsupported;
	} else {
		if( !isOutput ) {
			if( rxCRC )
				*checksumMask = kChecksumIP | kChecksumTCP | kChecksumUDP;
			else
				*checksumMask = 0;
		} else {
			if( txCRC )
				*checksumMask = kChecksumIP | kChecksumTCP | kChecksumUDP;
			else
				*checksumMask = 0;
		}
		return kIOReturnSuccess;
	}
}

IOReturn com_triton_forcedeth::setMulticastMode(bool active) {
	if( active ) {
		// Well, if there's a multicast list, it'll already there, so I needn't do anything.
		// And if there isn't, then we're already in the default state, which is the same
		// thing as enabling multicasting.
	} else {
		multicastMode = false;
		
		if( promiscuousMode ) {
			setPromiscuousMode(true);
		} else {
			updateMulticast(NVREG_PFF_MYADDR, 0, 0, 0, 0);
		}
	}
	
	return kIOReturnSuccess;
}

IOReturn com_triton_forcedeth::setPromiscuousMode(bool active) {
	if( active ) {
		updateMulticast(NVREG_PFF_PROMISC, 0, 0, 0, 0);
	} else {
		promiscuousMode = false;
		
		if( multicastMode ) {
			setMulticastMode(true);
		} else {
			updateMulticast(NVREG_PFF_MYADDR, 0, 0, 0, 0);
		}
	}
	
	return kIOReturnSuccess;
}
	
IOReturn com_triton_forcedeth::setMulticastList(IOEthernetAddress *addrs, UInt32 count) {
	UInt32 a, b;
	UInt32 alwaysOff[2];
	UInt32 alwaysOn[2];

	alwaysOn[0] = alwaysOn[1] = alwaysOff[0] = alwaysOff[1] = 0xffffffff;
	
	for( int i = 0; i < count; i++ ) {
		a = (addrs[i].bytes[0] << 0) + (addrs[i].bytes[1] << 8) + (addrs[i].bytes[2] << 16) + (addrs[i].bytes[3] << 24);
		b = (addrs[i].bytes[4] << 0) + (addrs[i].bytes[5] << 8);
		
		alwaysOn[0] &= a;
		alwaysOff[0] &= ~a;
		alwaysOn[1] &= b;
		alwaysOff[1] &= ~b;
	}
	
	updateMulticast(NVREG_PFF_MYADDR, alwaysOn[0], alwaysOn[1], alwaysOn[0] | alwaysOff[0], alwaysOn[1] | alwaysOff[1]);
	
	return kIOReturnSuccess;
}
