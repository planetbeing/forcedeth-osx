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

#include <IOKit/IOService.h>
#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/network/IOEthernetController.h>
#include <IOKit/network/IOMbufMemoryCursor.h>
#include <IOKit/network/IOGatedOutputQueue.h>

/*
 * Hardware access:
 */

#define DEV_NEED_LINKTIMER      0x0002

enum {
	NvRegIrqStatus = 0x000,
#define NVREG_IRQSTAT_MIIEVENT  0x040
#define NVREG_IRQSTAT_MASK              0x1ff
	NvRegIrqMask = 0x004,
#define NVREG_IRQ_RX_ERROR              0x0001
#define NVREG_IRQ_RX                    0x0002
#define NVREG_IRQ_RX_NOBUF              0x0004
#define NVREG_IRQ_TX_ERR                0x0008
#define NVREG_IRQ_TX_OK                 0x0010
#define NVREG_IRQ_TIMER                 0x0020
#define NVREG_IRQ_LINK                  0x0040
#define NVREG_IRQ_TX_ERROR              0x0080
#define NVREG_IRQ_TX1                   0x0100
#define NVREG_IRQMASK_THROUGHPUT        0x00df
#define NVREG_IRQMASK_CPU               0x0040
	
#define NVREG_IRQ_UNKNOWN       (~(NVREG_IRQ_RX_ERROR|NVREG_IRQ_RX|NVREG_IRQ_RX_NOBUF|NVREG_IRQ_TX_ERR| \
								   NVREG_IRQ_TX_OK|NVREG_IRQ_TIMER|NVREG_IRQ_LINK|NVREG_IRQ_TX_ERROR| \
								   NVREG_IRQ_TX1))
	
	NvRegUnknownSetupReg6 = 0x008,
#define NVREG_UNKSETUP6_VAL             3
	
	/*
	 * NVREG_POLL_DEFAULT is the interval length of the timer source on the nic
	 * NVREG_POLL_DEFAULT=97 would result in an interval length of 1 ms
	 */
	NvRegPollingInterval = 0x00c,
#define NVREG_POLL_DEFAULT_THROUGHPUT   970
#define NVREG_POLL_DEFAULT_CPU  13
	NvRegMisc1 = 0x080,
#define NVREG_MISC1_HD          0x02
#define NVREG_MISC1_FORCE       0x3b0f3c
	
	NvRegTransmitterControl = 0x084,
#define NVREG_XMITCTL_START     0x01
	NvRegTransmitterStatus = 0x088,
#define NVREG_XMITSTAT_BUSY     0x01
	
	NvRegPacketFilterFlags = 0x8c,
#define NVREG_PFF_ALWAYS        0x7F0008
#define NVREG_PFF_PROMISC       0x80
#define NVREG_PFF_MYADDR        0x20
	
	NvRegOffloadConfig = 0x90,
#define NVREG_OFFLOAD_HOMEPHY   0x601
#define NVREG_OFFLOAD_NORMAL    RX_NIC_BUFSIZE
	NvRegReceiverControl = 0x094,
#define NVREG_RCVCTL_START      0x01
	NvRegReceiverStatus = 0x98,
#define NVREG_RCVSTAT_BUSY      0x01
	
	NvRegRandomSeed = 0x9c,
#define NVREG_RNDSEED_MASK      0x00ff
#define NVREG_RNDSEED_FORCE     0x7f00
#define NVREG_RNDSEED_FORCE2    0x2d00
#define NVREG_RNDSEED_FORCE3    0x7400
	
	NvRegUnknownSetupReg1 = 0xA0,
#define NVREG_UNKSETUP1_VAL     0x16070f
	NvRegUnknownSetupReg2 = 0xA4,
#define NVREG_UNKSETUP2_VAL     0x16
	NvRegMacAddrA = 0xA8,
	NvRegMacAddrB = 0xAC,
	NvRegMulticastAddrA = 0xB0,
#define NVREG_MCASTADDRA_FORCE  0x01
	NvRegMulticastAddrB = 0xB4,
	NvRegMulticastMaskA = 0xB8,
	NvRegMulticastMaskB = 0xBC,
	
	NvRegPhyInterface = 0xC0,
#define PHY_RGMII               0x10000000
	
	NvRegTxRingPhysAddr = 0x100,
	NvRegRxRingPhysAddr = 0x104,
	NvRegRingSizes = 0x108,
#define NVREG_RINGSZ_TXSHIFT 0
#define NVREG_RINGSZ_RXSHIFT 16
	NvRegUnknownTransmitterReg = 0x10c,
	NvRegLinkSpeed = 0x110,
#define NVREG_LINKSPEED_FORCE 0x10000
#define NVREG_LINKSPEED_10      1000
#define NVREG_LINKSPEED_100     100
#define NVREG_LINKSPEED_1000    50
#define NVREG_LINKSPEED_MASK    (0xFFF)
	NvRegUnknownSetupReg5 = 0x130,
#define NVREG_UNKSETUP5_BIT31   (1<<31)
	NvRegUnknownSetupReg3 = 0x13c,
#define NVREG_UNKSETUP3_VAL1    0x200010
	NvRegTxRxControl = 0x144,
#define NVREG_TXRXCTL_KICK      0x0001
#define NVREG_TXRXCTL_BIT1      0x0002
#define NVREG_TXRXCTL_BIT2      0x0004
#define NVREG_TXRXCTL_IDLE      0x0008
#define NVREG_TXRXCTL_RESET     0x0010
#define NVREG_TXRXCTL_RXCHECK   0x0400
#define NVREG_TXRXCTL_DESC_1    0
#define NVREG_TXRXCTL_DESC_2    0x02100
#define NVREG_TXRXCTL_DESC_3    0x02200
	NvRegMIIStatus = 0x180,
#define NVREG_MIISTAT_ERROR             0x0001
#define NVREG_MIISTAT_LINKCHANGE        0x0008
#define NVREG_MIISTAT_MASK              0x000f
#define NVREG_MIISTAT_MASK2             0x000f
	NvRegUnknownSetupReg4 = 0x184,
#define NVREG_UNKSETUP4_VAL     8
	
	NvRegAdapterControl = 0x188,
#define NVREG_ADAPTCTL_START    0x02
#define NVREG_ADAPTCTL_LINKUP   0x04
#define NVREG_ADAPTCTL_PHYVALID 0x40000
#define NVREG_ADAPTCTL_RUNNING  0x100000
#define NVREG_ADAPTCTL_PHYSHIFT 24
	NvRegMIISpeed = 0x18c,
#define NVREG_MIISPEED_BIT8     (1<<8)
#define NVREG_MIIDELAY  5
	NvRegMIIControl = 0x190,
#define NVREG_MIICTL_INUSE      0x08000
#define NVREG_MIICTL_WRITE      0x00400
#define NVREG_MIICTL_ADDRSHIFT  5
	NvRegMIIData = 0x194,
	NvRegWakeUpFlags = 0x200,
#define NVREG_WAKEUPFLAGS_VAL           0x7770
#define NVREG_WAKEUPFLAGS_BUSYSHIFT     24
#define NVREG_WAKEUPFLAGS_ENABLESHIFT   16
#define NVREG_WAKEUPFLAGS_D3SHIFT       12
#define NVREG_WAKEUPFLAGS_D2SHIFT       8
#define NVREG_WAKEUPFLAGS_D1SHIFT       4
#define NVREG_WAKEUPFLAGS_D0SHIFT       0
#define NVREG_WAKEUPFLAGS_ACCEPT_MAGPAT         0x01
#define NVREG_WAKEUPFLAGS_ACCEPT_WAKEUPPAT      0x02
#define NVREG_WAKEUPFLAGS_ACCEPT_LINKCHANGE     0x04
#define NVREG_WAKEUPFLAGS_ENABLE        0x1111
	
	NvRegPatternCRC = 0x204,
	NvRegPatternMask = 0x208,
	NvRegPowerCap = 0x268,
#define NVREG_POWERCAP_D3SUPP   (1<<30)
#define NVREG_POWERCAP_D2SUPP   (1<<26)
#define NVREG_POWERCAP_D1SUPP   (1<<25)
	NvRegPowerState = 0x26c,
#define NVREG_POWERSTATE_POWEREDUP      0x8000
#define NVREG_POWERSTATE_VALID          0x0100
#define NVREG_POWERSTATE_MASK           0x0003
#define NVREG_POWERSTATE_D0             0x0000
#define NVREG_POWERSTATE_D1             0x0001
#define NVREG_POWERSTATE_D2             0x0002
#define NVREG_POWERSTATE_D3             0x0003
	NvRegPowerState2 = 0x600,
#define NVREG_POWERSTATE2_POWERUP_MASK         0x0F11
#define NVREG_POWERSTATE2_POWERUP_REV_A3       0x0001
};

#define FLAG_MASK_V2 0xffffc000
#define LEN_MASK_V2 (0xffffffff ^ FLAG_MASK_V2)

#define NV_TX_LASTPACKET        (1<<16)
#define NV_TX_RETRYERROR        (1<<19)
#define NV_TX_FORCED_INTERRUPT  (1<<24)
#define NV_TX_DEFERRED          (1<<26)
#define NV_TX_CARRIERLOST       (1<<27)
#define NV_TX_LATECOLLISION     (1<<28)
#define NV_TX_UNDERFLOW         (1<<29)
#define NV_TX_ERROR             (1<<30)
#define NV_TX_VALID             (1<<31)

#define NV_TX2_LASTPACKET       (1<<29)
#define NV_TX2_RETRYERROR       (1<<18)
#define NV_TX2_FORCED_INTERRUPT (1<<30)
#define NV_TX2_DEFERRED         (1<<25)
#define NV_TX2_CARRIERLOST      (1<<26)
#define NV_TX2_LATECOLLISION    (1<<27)
#define NV_TX2_UNDERFLOW        (1<<28)
/* error and valid are the same for both */
#define NV_TX2_ERROR            (1<<30)
#define NV_TX2_VALID            (1<<31)
#define NV_TX2_TSO              (1<<28)
#define NV_TX2_TSO_SHIFT        14
#define NV_TX2_TSO_MAX_SHIFT    14
#define NV_TX2_TSO_MAX_SIZE     (1<<NV_TX2_TSO_MAX_SHIFT)
#define NV_TX2_CHECKSUM_L3      (1<<27)
#define NV_TX2_CHECKSUM_L4      (1<<26)

#define NV_RX_DESCRIPTORVALID   (1<<16)
#define NV_RX_MISSEDFRAME       (1<<17)
#define NV_RX_SUBSTRACT1        (1<<18)
#define NV_RX_ERROR1            (1<<23)
#define NV_RX_ERROR2            (1<<24)
#define NV_RX_ERROR3            (1<<25)
#define NV_RX_ERROR4            (1<<26)
#define NV_RX_CRCERR            (1<<27)
#define NV_RX_OVERFLOW          (1<<28)
#define NV_RX_FRAMINGERR        (1<<29)
#define NV_RX_ERROR             (1<<30)
#define NV_RX_AVAIL             (1<<31)

#define NV_RX2_CHECKSUMMASK     (0x1C000000)
#define NV_RX2_CHECKSUMOK1      (0x10000000)
#define NV_RX2_CHECKSUMOK2      (0x14000000)
#define NV_RX2_CHECKSUMOK3      (0x18000000)
#define NV_RX2_DESCRIPTORVALID  (1<<29)
#define NV_RX2_SUBTRACT1       (1<<25)
#define NV_RX2_ERROR1           (1<<18)
#define NV_RX2_ERROR2           (1<<19)
#define NV_RX2_ERROR3           (1<<20)
#define NV_RX2_ERROR4           (1<<21)
#define NV_RX2_CRCERR           (1<<22)
#define NV_RX2_OVERFLOW         (1<<23)
#define NV_RX2_FRAMINGERR       (1<<24)
/* error and avail are the same for both */
#define NV_RX2_ERROR            (1<<30)
#define NV_RX2_AVAIL            (1<<31)

/* Miscelaneous hardware related defines: */
#define NV_PCI_REGSZ            0x270
#define NV_PCI_REGSZ_VER2              0x2d4
#define NV_PCI_REGSZ_VER3              0x604

/* various timeout delays: all in usec */
#define NV_TXRX_RESET_DELAY     4
#define NV_TXSTOP_DELAY1        10
#define NV_TXSTOP_DELAY1MAX     500000
#define NV_TXSTOP_DELAY2        100
#define NV_RXSTOP_DELAY1        10
#define NV_RXSTOP_DELAY1MAX     500000
#define NV_RXSTOP_DELAY2        100
#define NV_SETUP5_DELAY         5
#define NV_SETUP5_DELAYMAX      50000
#define NV_POWERUP_DELAY        5
#define NV_POWERUP_DELAYMAX     5000
#define NV_MIIBUSY_DELAY        50
#define NV_MIIPHY_DELAY 10
#define NV_MIIPHY_DELAYMAX      10000

#define NV_WAKEUPPATTERNS       5
#define NV_WAKEUPMASKENTRIES    4

/* General driver defaults */
#define NV_WATCHDOG_TIMEO       (5*HZ)

#define RX_RING         128
#define TX_RING         256
/* 
* If your nic mysteriously hangs then try to reduce the limits
 * to 1/0: It might be required to set NV_TX_LASTPACKET in the
 * last valid ring entry. But this would be impossible to
 * implement - probably a disassembly error.
 */
#define TX_LIMIT_STOP   255
#define TX_LIMIT_START  254

/* rx/tx mac addr + type + vlan + align + slack*/
#define NV_RX_HEADERS           (64)
/* even more slack. */
#define NV_RX_ALLOC_PAD         (64)

/* maximum mtu size */
#define NV_PKTLIMIT_1   kIOEthernetMaxPacketSize    /* hard limit not known */
#define NV_PKTLIMIT_2   9100    /* Actual limit according to NVidia: 9202 */

#define HZ 100
#define OOM_REFILL      (1+HZ/20)
#define POLL_WAIT       (1+HZ/100)
#define LINK_TIMEOUT    (3*HZ)

/* PHY defines */
#define PHY_OUI_MARVELL 0x5043
#define PHY_OUI_CICADA  0x03f1
#define PHYID1_OUI_MASK 0x03ff
#define PHYID1_OUI_SHFT 6
#define PHYID2_OUI_MASK 0xfc00
#define PHYID2_OUI_SHFT 10
#define PHY_INIT1       0x0f000
#define PHY_INIT2       0x0e00
#define PHY_INIT3       0x01000
#define PHY_INIT4       0x0200
#define PHY_INIT5       0x0004
#define PHY_INIT6       0x02000
#define PHY_GIGABIT     0x0100

#define PHY_TIMEOUT     0x1
#define PHY_ERROR       0x2

#define PHY_100 0x1
#define PHY_1000        0x2
#define PHY_HALF        0x100

/* FIXME: MII defines that should be added to <linux/mii.h> */
#define MII_SREVISION	0x16
#define MII_RESV1		0x17
#define MII_NCONFIG		0x1c
#define MII_BMCR        0x00
#define MII_BMSR		0x01
#define MII_PHYSID1		0x02
#define MII_PHYSID2		0x03
#define MII_ADVERTISE	0x04
#define BMCR_ANRESTART	0x0200
#define BMCR_ANENABLE	0x1000
#define BMCR_RESET		0x8000

// NEED THIS
#define BMSR_LSTATUS	0x0004
#define BMSR_ANEGCAPABLE   0x0008
#define BMSR_ANEGCOMPLETE   0x0020
#define BMSR_100FULL2	0x0200  /* Can do 100BASE-T2 HDX */
#define BMSR_100HALF2	0x0400  /* Can do 100BASE-T2 FDX */
#define BMSR_10HALF		0x0800  /* Can do 10mbps, half-duplex  */
#define BMSR_10FULL		0x1000  /* Can do 10mbps, full-duplex  */
#define BMSR_100HALF	0x2000  /* Can do 100mbps, half-duplex */
#define BMSR_100FULL	0x4000  /* Can do 100mbps, full-duplex */
#define BMSR_100BASE4	0x8000  /* Can do 100mbps, 4k packets  */
#define LPA_100HALF		0x0080
#define LPA_100FULL		0x0100
#define	LPA_10HALF		0x0020
#define LPA_10FULL		0x0040
#define MII_LPA			0x05

#define MII_1000BT_CR   0x09
#define MII_1000BT_SR   0x0a
#define ADVERTISE_10HALF	0x0020
#define ADVERTISE_10FULL	0x0040
#define	ADVERTISE_100HALF	0x0080
#define ADVERTISE_100FULL	0x0100
#define ADVERTISE_1000FULL	0x0200
#define ADVERTISE_1000HALF	0x0100
#define LPA_1000FULL    0x0800
#define LPA_1000HALF    0x0400

#define MII_READ        (-1)

typedef struct RingDesc {
	UInt32	PacketBuffer;
	UInt32	FlagLen;
};

static int maxInterruptWork = 5;

class com_triton_forcedeth : public IOEthernetController
{
	OSDeclareDefaultStructors(com_triton_forcedeth)
private:
	IOMemoryMap			*map;
	char				*baseAddress;
	IOEthernetAddress	macAddr;
	IOPCIDevice			*device;
	
	IONetworkInterface	*interface;
	IONetworkStats		*stats;
	IOOutputQueueStats	*outputStats;
	
	OSDictionary *dictionary;
	
	UInt16		vendorID;
	UInt16		deviceID;
	UInt16		subVendID;
	UInt16		subDevID;
	
	IOWorkLoop			*workLoop;
	IOTimerEventSource	*nicPollTimer;
	IOInterruptEventSource	*interrupt;
	
	IOGatedOutputQueue	*outputQueue;
	
	//IOLock			*lock;
	
	bool			ifEnabled;
	bool			promiscuousMode;
	bool			multicastMode;
	
	/* General data:
		* Locking: spin_lock(&np->lock); */
	int				inShutdown;
	UInt32			linkspeed;
	bool			duplex;
	bool			autoneg;
	int				fixedMode;
	int				phyAddr;
	bool			wolEnabled;
	unsigned int	phyOui;
	UInt16			gigabit;
	
	/* General data: RO fields */
	UInt32			ringAddr;
	UInt32			irqMask;
	UInt32			txrxCtlBits;
	UInt32			origMac[2];
	
	bool			txCRC;
	bool			rxCRC;
	bool			multSeg;
	bool			timerIRQ;
	bool			debugMode;
	
	bool locked;
	
	/* rx specific fields.
		* Locking: Within irq hander or disable_irq+spin_lock(&np->lock);
	*/
	RingDesc		*rxRing;		// ring descriptors
	UInt64			curRx;
	mbuf_t			rxBuf[RX_RING];
	UInt32			rxDma[RX_RING];	// I'm thinking this is a bunch of addresses, the coalesced packets
	unsigned int	rxBufSz;
	unsigned int	pktLimit;
	
	/*
	 * tx specific fields.
	 */
	RingDesc		*txRing;
	UInt64			nextTx;
	UInt64			nicTx;
	mbuf_t			txBuf[TX_RING];
	UInt32			txDma[TX_RING];
	unsigned int	txDmaLen[TX_RING];
	UInt32			txFlags;
	
	/* media detection workaround.
		* Locking: Within irq hander or disable_irq+spin_lock(&np->lock); */
	bool			needLinkTimer;
	unsigned long	linkTimeout;
	
	// Some memory crap. I'm learning way too much about this shit.
	IOMbufNaturalMemoryCursor *rxMbufCursor;
	IOMbufNaturalMemoryCursor *txMbufCursor;
	
	volatile UInt32 readRegister(UInt16 offset);
	void writeRegister(UInt16 offset, UInt32 data);
	bool regDelay(int offset, UInt32 mask, UInt32 target, int delay, int delayMax, const char *msg);
	int miiRW(int addr, int miireg, int value);
	
	bool phyReset();
	int phyInit();
	
	void copyMacToHW();

	bool initRx();
	void initTx();
	bool initRing();
	
	bool releaseTxPkt(int index);
	void drainTx();
	void drainRx();
	void drainRing();
	
	void setBufSize();
	
	void startRx();
	void startTx();
	void stopRx();
	void stopTx();
	void txrxReset();
	
	int getLength(mbuf_t buf, int len);
	void rxProcess();
	
	bool updateLinkSpeed();
	
	void linkChange();
	
	void linkIRQ();
	
	void nicIRQ();
	
	void txDone();
	
	void doNicPoll();
	
	void updateMulticast(UInt32 pff, UInt32 addrA, UInt32 addrB, UInt32 maskA, UInt32 maskB);
	
protected:
	virtual IOOutputQueue *createOutputQueue();
	/*virtual bool createWorkLoop();*/
		
public:
	/*virtual IOWorkLoop* getWorkLoop() const;*/

	virtual UInt32 outputPacket(mbuf_t buf, void *param);
    virtual bool init(OSDictionary *dictionary = 0);
    virtual void free(void);
    virtual IOService *probe(IOService *provider, SInt32 *score);
    virtual bool start(IOService *provider);
    virtual void stop(IOService *provider);
	virtual IOReturn enable(IOService *provider);
	virtual IOReturn disable(IOService *provider);
	
	virtual IOReturn getHardwareAddress(IOEthernetAddress * addrP);
	virtual IOReturn setHardwareAddress(const IOEthernetAddress * addrP);
	virtual IOReturn setWakeOnMagicPacket(bool active);
	virtual IOReturn getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput);
	virtual IOReturn setMulticastMode(bool active);
	virtual IOReturn setPromiscuousMode(bool active);
	virtual IOReturn setMulticastList(IOEthernetAddress *addrs, UInt32 count);
};