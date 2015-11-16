#ifndef LINUX_H
#define LINUX_H

// Provide enough linux structures to get a driver to compile with few changes


#include <stdint.h>

#define PROGRAM_START 0x80c0000
#define BIT_IS_SET(byte, bit) ((byte & (1 << bit)) ? 1 : 0)
#define BIT_IS_CLEAR(byte, bit) ((byte & (1 << bit)) ? 0 : 1)
#define SET_BIT_(byte, bit) byte |= (1 << bit);
#define CLEAR_BIT_(byte, bit) byte &= ~(1 << bit);
#define COPY_BIT(dst_byte, dst_bit, src_byte, src_bit) \
	if((src_byte & (1 << src_bit)) != 0) \
		dst_byte |= (1 << dst_bit); \
	else \
		dst_byte &= ~(1 << dst_bit);
#define CLAMP(x, y, z) ((x) = ((x) < (y) ? (y) : ((x) > (z) ? (z) : (x))))
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) < (y) ? (x) : (y))

#define SQR(x) ((x) * (x))
#define M_PI  3.14159f

#define BIT_TO_PIN(gpio, x, bit) \
{ \
	if(bit) \
		gpio->BSRRL = x; \
	else \
		gpio->BSRRH = x; \
}
#define SET_PIN(gpio, x) gpio->BSRRL = x;
#define CLEAR_PIN(gpio, x) gpio->BSRRH = x;
// only tests 1 pin
#define PIN_IS_SET(gpio, x) ((gpio->IDR & (x)) ? 1 : 0)
#define PIN_IS_CLEAR(gpio, x) ((gpio->IDR & (x)) ? 0 : 1)
#define TOGGLE_PIN(gpio, x) (gpio->ODR ^= (x))
#define SET_COMPARE(tim, number, value) { tim->number = value; }

extern uint64_t jiffies;

typedef int		__kernel_ssize_t;
typedef int		__kernel_ptrdiff_t;
// typedef int		pid_t;
typedef uint8_t* dma_addr_t;

typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

typedef uint64_t __u64;
typedef uint32_t __u32;
typedef uint16_t __u16;
typedef uint8_t __u8;

typedef int64_t s64;
typedef int32_t s32;
typedef int16_t s16;
typedef int8_t s8;

typedef int64_t __s64;
typedef int32_t __s32;
typedef int16_t __s16;
typedef int8_t __s8;

typedef uint64_t __le64;
typedef uint32_t __le32;
typedef uint16_t __le16;
typedef uint8_t __le8;

typedef uint64_t le64;
typedef uint32_t le32;
typedef uint16_t le16;
typedef uint8_t le8;

typedef uint32_t __be32;
typedef uint16_t __be16;
typedef uint8_t __be8;

typedef int size_t;
typedef int __kernel_size_t;

typedef char bool;
#define true 1
#define false 0


typedef __u32 __wsum;
typedef __u16 __sum16;
typedef unsigned gfp_t;

#define cpu_to_le16(x) (x)
#define le16_to_cpu(x) (x)
#define cpu_to_le32(x) (x)
#define le32_to_cpu(x) (x)
#define host_to_le16(x) (x)

#define __iomem
#define unlikely(x) (x)
#define likely(x) (x)
#define test_bit(x, addr) ((*(addr) & (x)) ? 1 : 0)


#define container_of(ptr, type, member) ({			\
	const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
	(type *)( (char *)__mptr - offsetof(type,member) );})

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define __LITTLE_ENDIAN_BITFIELD
#define uninitialized_var(x) x = x
#define WARN_ON(condition) (0)
#define WARN_ON_ONCE(condition) (0)
#define pr_err(fmt, ...)


#define IEEE80211_HDRLEN (sizeof(struct ieee80211_hdr))
#define IEEE80211_FC(type, stype) host_to_le16((type << 2) | (stype << 4))

// bits 2 & 3 of the frame control
#define WLAN_FC_TYPE_MGMT		(0 << 2)
#define WLAN_FC_TYPE_CTRL		(1 << 2)
#define WLAN_FC_TYPE_DATA		(2 << 2)

/* management */
#define WLAN_FC_STYPE_ASSOC_REQ		0
#define WLAN_FC_STYPE_ASSOC_RESP	1
#define WLAN_FC_STYPE_REASSOC_REQ	2
#define WLAN_FC_STYPE_REASSOC_RESP	3
#define WLAN_FC_STYPE_PROBE_REQ		4
#define WLAN_FC_STYPE_PROBE_RESP	5
#define WLAN_FC_STYPE_BEACON		8
#define WLAN_FC_STYPE_ATIM		    9
#define WLAN_FC_STYPE_DISASSOC		0xa
#define WLAN_FC_STYPE_AUTH		    0xb
#define WLAN_FC_STYPE_DEAUTH		0xc
#define WLAN_FC_STYPE_ACTION		0xd

#define BUILD_BUG_ON_NOT_POWER_OF_2(n)
#define BUILD_BUG_ON_ZERO(e) (0)
#define BUILD_BUG_ON_NULL(e) ((void*)0)
#define BUILD_BUG_ON(condition)
#define BUG_ON(condition)

#define LINUX_VERSION_CODE 132641
#define KERNEL_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))
#define	EPERM		 1	/* Operation not permitted */
#define	ENOENT		 2	/* No such file or directory */
#define	ESRCH		 3	/* No such process */
#define	EINTR		 4	/* Interrupted system call */
#define	EIO		 5	/* I/O error */
#define	ENXIO		 6	/* No such device or address */
#define	E2BIG		 7	/* Argument list too long */
#define	ENOEXEC		 8	/* Exec format error */
#define	EBADF		 9	/* Bad file number */
#define	ECHILD		10	/* No child processes */
#define	EAGAIN		11	/* Try again */
#define	ENOMEM		12	/* Out of memory */
#define	EACCES		13	/* Permission denied */
#define	EFAULT		14	/* Bad address */
#define	ENOTBLK		15	/* Block device required */
#define	EBUSY		16	/* Device or resource busy */
#define	EEXIST		17	/* File exists */
#define	EXDEV		18	/* Cross-device link */
#define	ENODEV		19	/* No such device */
#define	ENOTDIR		20	/* Not a directory */
#define	EISDIR		21	/* Is a directory */
#define	EINVAL		22	/* Invalid argument */
#define	ENFILE		23	/* File table overflow */
#define	EMFILE		24	/* Too many open files */
#define	ENOTTY		25	/* Not a typewriter */
#define	ETXTBSY		26	/* Text file busy */
#define	EFBIG		27	/* File too large */
#define	ENOSPC		28	/* No space left on device */
#define	ESPIPE		29	/* Illegal seek */
#define	EROFS		30	/* Read-only file system */
#define	EMLINK		31	/* Too many links */
#define	EPIPE		32	/* Broken pipe */
#define	EDOM		33	/* Math argument out of domain of func */
#define	ERANGE		34	/* Math result not representable */

#define	EDEADLK		35	/* Resource deadlock would occur */
#define	ENAMETOOLONG	36	/* File name too long */
#define	ENOLCK		37	/* No record locks available */
#define	ENOSYS		38	/* Function not implemented */
#define	ENOTEMPTY	39	/* Directory not empty */
#define	ELOOP		40	/* Too many symbolic links encountered */
#define	EWOULDBLOCK	EAGAIN	/* Operation would block */
#define	ENOMSG		42	/* No message of desired type */
#define	EIDRM		43	/* Identifier removed */
#define	ECHRNG		44	/* Channel number out of range */
#define	EL2NSYNC	45	/* Level 2 not synchronized */
#define	EL3HLT		46	/* Level 3 halted */
#define	EL3RST		47	/* Level 3 reset */
#define	ELNRNG		48	/* Link number out of range */
#define	EUNATCH		49	/* Protocol driver not attached */
#define	ENOCSI		50	/* No CSI structure available */
#define	EL2HLT		51	/* Level 2 halted */
#define	EBADE		52	/* Invalid exchange */
#define	EBADR		53	/* Invalid request descriptor */
#define	EXFULL		54	/* Exchange full */
#define	ENOANO		55	/* No anode */
#define	EBADRQC		56	/* Invalid request code */
#define	EBADSLT		57	/* Invalid slot */

#define	EDEADLOCK	EDEADLK

#define	EBFONT		59	/* Bad font file format */
#define	ENOSTR		60	/* Device not a stream */
#define	ENODATA		61	/* No data available */
#define	ETIME		62	/* Timer expired */
#define	ENOSR		63	/* Out of streams resources */
#define	ENONET		64	/* Machine is not on the network */
#define	ENOPKG		65	/* Package not installed */
#define	EREMOTE		66	/* Object is remote */
#define	ENOLINK		67	/* Link has been severed */
#define	EADV		68	/* Advertise error */
#define	ESRMNT		69	/* Srmount error */
#define	ECOMM		70	/* Communication error on send */
#define	EPROTO		71	/* Protocol error */
#define	EMULTIHOP	72	/* Multihop attempted */
#define	EDOTDOT		73	/* RFS specific error */
#define	EBADMSG		74	/* Not a data message */
#define	EOVERFLOW	75	/* Value too large for defined data type */
#define	ENOTUNIQ	76	/* Name not unique on network */
#define	EBADFD		77	/* File descriptor in bad state */
#define	EREMCHG		78	/* Remote address changed */
#define	ELIBACC		79	/* Can not access a needed shared library */
#define	ELIBBAD		80	/* Accessing a corrupted shared library */
#define	ELIBSCN		81	/* .lib section in a.out corrupted */
#define	ELIBMAX		82	/* Attempting to link in too many shared libraries */
#define	ELIBEXEC	83	/* Cannot exec a shared library directly */
#define	EILSEQ		84	/* Illegal byte sequence */
#define	ERESTART	85	/* Interrupted system call should be restarted */
#define	ESTRPIPE	86	/* Streams pipe error */
#define	EUSERS		87	/* Too many users */
#define	ENOTSOCK	88	/* Socket operation on non-socket */
#define	EDESTADDRREQ	89	/* Destination address required */
#define	EMSGSIZE	90	/* Message too long */
#define	EPROTOTYPE	91	/* Protocol wrong type for socket */
#define	ENOPROTOOPT	92	/* Protocol not available */
#define	EPROTONOSUPPORT	93	/* Protocol not supported */
#define	ESOCKTNOSUPPORT	94	/* Socket type not supported */
#define	EOPNOTSUPP	95	/* Operation not supported on transport endpoint */
#define	EPFNOSUPPORT	96	/* Protocol family not supported */
#define	EAFNOSUPPORT	97	/* Address family not supported by protocol */
#define	EADDRINUSE	98	/* Address already in use */
#define	EADDRNOTAVAIL	99	/* Cannot assign requested address */
#define	ENETDOWN	100	/* Network is down */
#define	ENETUNREACH	101	/* Network is unreachable */
#define	ENETRESET	102	/* Network dropped connection because of reset */
#define	ECONNABORTED	103	/* Software caused connection abort */
#define	ECONNRESET	104	/* Connection reset by peer */
#define	ENOBUFS		105	/* No buffer space available */
#define	EISCONN		106	/* Transport endpoint is already connected */
#define	ENOTCONN	107	/* Transport endpoint is not connected */
#define	ESHUTDOWN	108	/* Cannot send after transport endpoint shutdown */
#define	ETOOMANYREFS	109	/* Too many references: cannot splice */
#define	ETIMEDOUT	110	/* Connection timed out */
#define	ECONNREFUSED	111	/* Connection refused */
#define	EHOSTDOWN	112	/* Host is down */
#define	EHOSTUNREACH	113	/* No route to host */
#define	EALREADY	114	/* Operation already in progress */
#define	EINPROGRESS	115	/* Operation now in progress */
#define	ESTALE		116	/* Stale NFS file handle */
#define	EUCLEAN		117	/* Structure needs cleaning */
#define	ENOTNAM		118	/* Not a XENIX named type file */
#define	ENAVAIL		119	/* No XENIX semaphores available */
#define	EISNAM		120	/* Is a named type file */
#define	EREMOTEIO	121	/* Remote I/O error */
#define	EDQUOT		122	/* Quota exceeded */

#define	ENOMEDIUM	123	/* No medium found */
#define	EMEDIUMTYPE	124	/* Wrong medium type */
#define	ECANCELED	125	/* Operation Canceled */
#define	ENOKEY		126	/* Required key not available */
#define	EKEYEXPIRED	127	/* Key has expired */
#define	EKEYREVOKED	128	/* Key has been revoked */
#define	EKEYREJECTED	129	/* Key was rejected by service */

/* for robust mutexes */
#define	EOWNERDEAD	130	/* Owner died */
#define	ENOTRECOVERABLE	131	/* State not recoverable */

#define ERFKILL		132	/* Operation not possible due to RF-kill */

#define EHWPOISON	133	/* Memory page has hardware error */


#define BITS_PER_LONG 32

#define BIT(x) (1 << (x))
#define set_bit(bit, data) (*data) |= (1 << (bit));


/* __packed keyword used to decrease the data type alignment to 1-byte */
#if defined (__CC_ARM)         /* ARM Compiler */
  #define __packed    __packed
#elif defined (__ICCARM__)     /* IAR Compiler */
  #define __packed    __packed
#elif defined   ( __GNUC__ )   /* GNU Compiler */                        
  #define __packed    __attribute__ ((__packed__))
  #define STRUCT_PACKED    __attribute__ ((__packed__))
#elif defined   (__TASKING__)  /* TASKING Compiler */
  #define __packed    __unaligned
#endif /* __CC_ARM */


#define __devinit
#define MODULE_AUTHOR(x)
#define MODULE_LICENSE(x)
#define MODULE_DESCRIPTION(x)
#define MODULE_FIRMWARE(x)
#define module_param_named(a, b, c, d)
#define MODULE_PARM_DESC(a, b)


#define __aligned(x)
#define NULL 0

#define KERN_DEBUG
#define printk(...) 
#define wpa_printf(x, y, ...)

void* kmalloc(int size, int atomic);
void* vmalloc(int size);
void kfree(void* ptr);
void vfree(void* ptr);
void bzero(void *ptr, int size);
void memcpy(void *dst, const void *src, int size);
int strcpy(char *dst, const char *src);
int strncpy(char *dst, const char *src, int maxlen);
void memset(void *dst, uint8_t value, int size);
void udelay(int usec);
void mdelay(int msec);
int memcmp(uint8_t *dst, uint8_t *src, int size);
int strlen(const char *string);



void init_linux();




#define EXPORT_SYMBOL(x)
#define GFP_ATOMIC 0










/* Standard well-defined IP protocols.  */
enum {
  IPPROTO_IP = 0,		/* Dummy protocol for TCP		*/
  IPPROTO_ICMP = 1,		/* Internet Control Message Protocol	*/
  IPPROTO_IGMP = 2,		/* Internet Group Management Protocol	*/
  IPPROTO_IPIP = 4,		/* IPIP tunnels (older KA9Q tunnels use 94) */
  IPPROTO_TCP = 6,		/* Transmission Control Protocol	*/
  IPPROTO_EGP = 8,		/* Exterior Gateway Protocol		*/
  IPPROTO_PUP = 12,		/* PUP protocol				*/
  IPPROTO_UDP = 17,		/* User Datagram Protocol		*/
  IPPROTO_IDP = 22,		/* XNS IDP protocol			*/
  IPPROTO_DCCP = 33,		/* Datagram Congestion Control Protocol */
  IPPROTO_RSVP = 46,		/* RSVP protocol			*/
  IPPROTO_GRE = 47,		/* Cisco GRE tunnels (rfc 1701,1702)	*/

  IPPROTO_IPV6	 = 41,		/* IPv6-in-IPv4 tunnelling		*/

  IPPROTO_ESP = 50,            /* Encapsulation Security Payload protocol */
  IPPROTO_AH = 51,             /* Authentication Header protocol       */
  IPPROTO_BEETPH = 94,	       /* IP option pseudo header for BEET */
  IPPROTO_PIM    = 103,		/* Protocol Independent Multicast	*/

  IPPROTO_COMP   = 108,                /* Compression Header protocol */
  IPPROTO_SCTP   = 132,		/* Stream Control Transport Protocol	*/
  IPPROTO_UDPLITE = 136,	/* UDP-Lite (RFC 3828)			*/

  IPPROTO_RAW	 = 255,		/* Raw IP packets			*/
  IPPROTO_MAX
};


struct iphdr {
#if defined(__LITTLE_ENDIAN_BITFIELD)
	__u8	ihl:4,
		version:4;
#elif defined (__BIG_ENDIAN_BITFIELD)
	__u8	version:4,
  		ihl:4;
#else
#error	"Please fix <asm/byteorder.h>"
#endif
	__u8	tos;
	__be16	tot_len;
	__be16	id;
	__be16	frag_off;
	__u8	ttl;
	__u8	protocol;
	__sum16	check;
	__be32	saddr;
	__be32	daddr;
	/*The options start here. */
};








typedef struct {
	int counter;
} atomic_t;


union ktime {
	s64	tv64;
#if BITS_PER_LONG != 64 && !defined(CONFIG_KTIME_SCALAR)
	struct {
# ifdef __BIG_ENDIAN
	s32	sec, nsec;
# else
	s32	nsec, sec;
# endif
	} tv;
#endif
};

typedef union ktime ktime_t;		/* Kill this */





typedef struct spinlock 
{
} spinlock_t;

struct mutex
{
};

struct timer_list
{
};

struct tasklet_struct
{
};

struct delayed_work
{
};

struct urb
{
};

struct usb_anchor
{
};

typedef struct
{
} pm_message_t;

struct usb_device_id
{
};

struct pci_device_id
{
};

struct device
{
};


struct list_head {
	struct list_head *next, *prev;
};

#define spin_unlock(a)
#define spin_lock(a)
#define spin_unlock_irqrestore(a, b)
#define spin_lock_irqsave(a, b)
#define spin_unlock_irq(a)
#define spin_lock_irq(a)
#define spin_lock_init(a)
#define mutex_init(a)
#define mutex_lock(a)
#define mutex_unlock(a)
#define pr_info(x, ...)
#define rcu_read_lock()
#define rcu_read_unlock()

#define kmemcheck_bitfield_begin(x)
#define kmemcheck_bitfield_end(x)
#define hw_to_local(x) (x)
#define cancel_delayed_work(x)
#define queue_delayed_work(x, y, z)
#define dev_kfree_skb_any(x)
#define wiphy_rfkill_set_hw_state(x, y)
void print_heap();
void workaround1(int x);


static inline unsigned int jiffies_to_msecs(const unsigned long j)
{
	return (uint64_t)(j * 1000) / jiffies;
}


static inline unsigned int msecs_to_jiffies(const unsigned long m)
{
	return (uint64_t)m * jiffies / 1000;
}


#include "linux/ethtool.h"
#include "linux/skbuff.h"
#include "net/mac80211.h"

#endif



