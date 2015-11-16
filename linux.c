/*
 * STM32F4 Wifi flight controller
 * Copyright (C) 2012 Adam Williams <broadcast at earthling dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */

#include "linux.h"
#include "linux/skbuff.h"
#include "uart.h"



//#define HEAP_SIZE 8192
// big enough for network firmware & camera buffers
//#define HEAP_SIZE 65536
#define HEAP_SIZE 81920
uint8_t heap[HEAP_SIZE];
#define MAX_SEGMENTS 32

typedef struct
{
	uint8_t* ptr;
	int size;
} segment_t;

segment_t segments[MAX_SEGMENTS];
int total_segments;
uint64_t jiffies;


void workaround1(int x)
{
}

void print_heap()
{
	int i;
	TRACE
	print_text("segments=");
	print_number(total_segments);
	print_lf();
	
	for(i = 0; i < total_segments; i++)
	{
		print_text(" ptr=");
		print_hex((uint32_t)segments[i].ptr);
		print_text(" size=");
		print_number(segments[i].size);
		print_lf();
	}
	
	flush_uart();
}

void* kmalloc(int size, int atomic)
{
	uint8_t* result = 0;
	int heap_used = 0;
// 32 bit alignment
	if(size % 4) size += 4 - (size % 4);
	
	if(total_segments)
	{
		heap_used = segments[total_segments - 1].ptr +
			segments[total_segments - 1].size -
			heap;
		result = heap + heap_used;
	}
	else
	{
		result = heap;
	}

	if(heap_used + size > HEAP_SIZE ||
		total_segments >= MAX_SEGMENTS)
	{
		TRACE
		print_text("out of memory ");
		print_text(" size=");
		print_number(size);
		print_text("used=");
		print_number(heap_used + size);
		print_text(" total_segments=");
		print_number(total_segments);
		print_lf();
		flush_uart();
		return 0;
	}
	else
	{
		TRACE
		print_text("result=0x");
		print_hex((uint32_t)result);
		print_text(" size=");
		print_number(size);
		print_text(" used=");
		print_number(heap_used + size);
		print_lf();
		flush_uart();
	}

	segments[total_segments].ptr = result;
	segments[total_segments].size = size;
	total_segments++;

	bzero(result, size);
	return result;
}


void* vmalloc(int size)
{
	return kmalloc(size, 0);
}



void kfree(void* ptr)
{
	int i;
	int got_it = 0;
	for(i = 0; i < total_segments; i++)
	{
		if(segments[i].ptr == ptr)
		{
			got_it = 1;

// shift all later segments forward
			while(i < total_segments - 1)
			{
				segments[i] = segments[i + 1];
				i++;
			}

			total_segments--;
			break;
		}
	}

	if(!got_it)
	{
		TRACE
		print_text("ptr=");
		print_hex((uint32_t)ptr);
		print_text(" not allocated");
		print_lf();
	}
	else
	{
		TRACE
		print_text("ptr=");
		print_hex((u32)ptr);
		print_text(" used=");
		int heap_used = 0;
		if(total_segments)
			heap_used = segments[total_segments - 1].ptr +
				segments[total_segments - 1].size -
				heap;
		print_number(heap_used);
	}
}


void vfree(void* ptr)
{
	kfree(ptr);
}


void bzero(void *ptr, int size)
{
	int i;
	if((size % 4) || (((int)ptr) % 4))
	{
		unsigned char *ptr2 = ptr;
		for(i = 0; i < size; i++)
			ptr2[i] = 0;
	}
	else
	{
		int size4 = size / 4;
		uint32_t *ptr2 = ptr;
		for(i = 0; i < size4; i++)
			ptr2[i] = 0;
	}
}

int strcpy(char *dst, const char *src)
{
	int i = 0;
	while(*src != 0)
	{
		*dst++ = *src++;
		i++;
	}
	*dst = 0;
	return i;
}

int strncpy(char *dst, const char *src, int maxlen)
{
	int i = 0;
	while(*src != 0 && i < maxlen - 1)
	{
		*dst++ = *src++;
		i++;
	}
	*dst = 0;
	return i;
}

void memcpy(void *dst, const void *src, int size)
{
	int i;

	if((size % 4) || (((int)dst) % 4) || (((int)src) % 4))
	{
/*
 * TRACE
 * print_text("Not a multiple of 4 size=");
 * print_number(size);
 */
		unsigned char *ptr1 = dst;
		const unsigned char *ptr2 = src;
		for(i = 0; i < size; i++)
			*ptr1++ = *ptr2++;
	}
	else
	{
		int size4 = size / 4;
		uint32_t *ptr1 = dst;
		const uint32_t *ptr2 = src;
		for(i = 0; i < size4; i++)
			*ptr1++ = *ptr2++;
	}
}

void memset(void *dst, uint8_t value, int size)
{
	int i;
	unsigned char *ptr = dst;
	for(i = 0; i < size; i++)
		ptr[i] = value;
}

int memcmp(uint8_t *dst, uint8_t *src, int size)
{
	int i;
	for(i = 0; i < size; i++)
	{
		if(dst[i] != src[i]) return 1;
	}
	return 0;
}

int strlen(const char *string)
{
	int i = 0;
	while(string[i] != 0) i++;
	return i;
}



void udelay(int usec)
{
	USB_OTG_BSP_uDelay (usec);
}

void mdelay(int msec)
{
	USB_OTG_BSP_mDelay (msec);
}


unsigned int ieee80211_hdrlen(__le16 fc)
{
	unsigned int hdrlen = 24;

	if (ieee80211_is_data(fc)) {
		if (ieee80211_has_a4(fc))
			hdrlen = 30;
		if (ieee80211_is_data_qos(fc)) {
			hdrlen += IEEE80211_QOS_CTL_LEN;
			if (ieee80211_has_order(fc))
				hdrlen += IEEE80211_HT_CTL_LEN;
		}
		goto out;
	}

	if (ieee80211_is_ctl(fc)) {
		/*
		 * ACK and CTS are 10 bytes, all others 16. To see how
		 * to get this condition consider
		 *   subtype mask:   0b0000000011110000 (0x00F0)
		 *   ACK subtype:    0b0000000011010000 (0x00D0)
		 *   CTS subtype:    0b0000000011000000 (0x00C0)
		 *   bits that matter:         ^^^      (0x00E0)
		 *   value of those: 0b0000000011000000 (0x00C0)
		 */
		if ((fc & cpu_to_le16(0x00E0)) == cpu_to_le16(0x00C0))
			hdrlen = 10;
		else
			hdrlen = 16;
	}
out:
	return hdrlen;
}



unsigned int ieee80211_get_hdrlen_from_skb(const struct sk_buff *skb)
{
	const struct ieee80211_hdr *hdr =
			(const struct ieee80211_hdr *)skb->data;
	unsigned int hdrlen;

	if (unlikely(skb->len < 10))
		return 0;
	hdrlen = ieee80211_hdrlen(hdr->frame_control);
	if (unlikely(hdrlen > skb->len))
		return 0;
	return hdrlen;
}

void ieee80211_rx_irqsafe(struct ieee80211_hw *hw, struct sk_buff *skb)
{
TRACE
/*
 * 	struct ieee80211_local *local = hw_to_local(hw);
 * 
 * 	BUILD_BUG_ON(sizeof(struct ieee80211_rx_status) > sizeof(skb->cb));
 * 
 * 	skb->pkt_type = IEEE80211_RX_MSG;
 * 	skb_queue_tail(&local->skb_queue, skb);
 * 	tasklet_schedule(&local->tasklet);
 */
}

static void skb_under_panic(struct sk_buff *skb, int sz, void *here)
{
	TRACE
}

unsigned char *skb_push(struct sk_buff *skb, unsigned int len)
{
	skb->data -= len;
	skb->len  += len;
	if (unlikely(skb->data<skb->head))
		skb_under_panic(skb, len, __builtin_return_address(0));
	return skb->data;
}

struct ieee80211_sta *ieee80211_find_sta(struct ieee80211_vif *vif,
					 const u8 *addr)
{
//TRACE
#if 0
	struct sta_info *sta;

	if (!vif)
		return NULL;

	sta = sta_info_get_bss(vif_to_sdata(vif), addr);
	if (!sta)
		return NULL;

	if (!sta->uploaded)
		return NULL;

	return &sta->sta;
#endif // 0

	return 0;
}

struct ieee80211_sta *ieee80211_find_sta_by_ifaddr(struct ieee80211_hw *hw,
					       const u8 *addr,
					       const u8 *localaddr)
{
TRACE

#if 0
	struct sta_info *sta, *nxt;

	/*
	 * Just return a random station if localaddr is NULL
	 * ... first in list.
	 */
	for_each_sta_info(hw_to_local(hw), addr, sta, nxt) {
		if (localaddr &&
		    compare_ether_addr(sta->sdata->vif.addr, localaddr) != 0)
			continue;
		if (!sta->uploaded)
			return NULL;
		return &sta->sta;
	}
#endif // 0



	return NULL;
}

/**
 *	__alloc_skb	-	allocate a network buffer
 *	@size: size to allocate
 *	@gfp_mask: allocation mask
 *	@fclone: allocate from fclone cache instead of head cache
 *		and allocate a cloned (child) skb
 *	@node: numa node to allocate memory on
 *
 *	Allocate a new &sk_buff. The returned buffer has no headroom and a
 *	tail room of size bytes. The object has a reference count of one.
 *	The return is the buffer. On a failure the return is %NULL.
 *
 *	Buffers may only be allocated from interrupts using a @gfp_mask of
 *	%GFP_ATOMIC.
 */
struct sk_buff *__alloc_skb(unsigned int size, gfp_t gfp_mask,
			    int fclone, int node)
{
TRACE


#if 0

	struct kmem_cache *cache;
	struct skb_shared_info *shinfo;
	struct sk_buff *skb;
	u8 *data;

	cache = fclone ? skbuff_fclone_cache : skbuff_head_cache;

	/* Get the HEAD */
	skb = kmem_cache_alloc_node(cache, gfp_mask & ~__GFP_DMA, node);
	if (!skb)
		goto out;
	prefetchw(skb);

	/* We do our best to align skb_shared_info on a separate cache
	 * line. It usually works because kmalloc(X > SMP_CACHE_BYTES) gives
	 * aligned memory blocks, unless SLUB/SLAB debug is enabled.
	 * Both skb->head and skb_shared_info are cache line aligned.
	 */
	size = SKB_DATA_ALIGN(size);
	size += SKB_DATA_ALIGN(sizeof(struct skb_shared_info));
	data = kmalloc_node_track_caller(size, gfp_mask, node);
	if (!data)
		goto nodata;
	/* kmalloc(size) might give us more room than requested.
	 * Put skb_shared_info exactly at the end of allocated zone,
	 * to allow max possible filling before reallocation.
	 */
	size = SKB_WITH_OVERHEAD(ksize(data));
	prefetchw(data + size);

	/*
	 * Only clear those fields we need to clear, not those that we will
	 * actually initialise below. Hence, don't put any more fields after
	 * the tail pointer in struct sk_buff!
	 */
	memset(skb, 0, offsetof(struct sk_buff, tail));
	/* Account for allocated memory : skb + skb->head */
	skb->truesize = SKB_TRUESIZE(size);
	atomic_set(&skb->users, 1);
	skb->head = data;
	skb->data = data;
	skb_reset_tail_pointer(skb);
	skb->end = skb->tail + size;
#ifdef NET_SKBUFF_DATA_USES_OFFSET
	skb->mac_header = ~0U;
#endif

	/* make sure we initialize shinfo sequentially */
	shinfo = skb_shinfo(skb);
	memset(shinfo, 0, offsetof(struct skb_shared_info, dataref));
	atomic_set(&shinfo->dataref, 1);
	kmemcheck_annotate_variable(shinfo->destructor_arg);

	if (fclone) {
		struct sk_buff *child = skb + 1;
		atomic_t *fclone_ref = (atomic_t *) (child + 1);

		kmemcheck_annotate_bitfield(child, flags1);
		kmemcheck_annotate_bitfield(child, flags2);
		skb->fclone = SKB_FCLONE_ORIG;
		atomic_set(fclone_ref, 1);

		child->fclone = SKB_FCLONE_UNAVAILABLE;
	}
out:
	return skb;
nodata:
	kmem_cache_free(cache, skb);
	skb = NULL;
	goto out;
#endif // 0


}


void __kfree_skb(struct sk_buff *skb)
{
TRACE
//	skb_release_all(skb);
//	kfree_skbmem(skb);
}

/**
 *	kfree_skb - free an sk_buff
 *	@skb: buffer to free
 *
 *	Drop a reference to the buffer and free it if the usage count has
 *	hit zero.
 */
void kfree_skb(struct sk_buff *skb)
{
//	if (unlikely(!skb))
//		return;
//	if (likely(atomic_read(&skb->users) == 1))
//		smp_rmb();
//	else if (likely(!atomic_dec_and_test(&skb->users)))
//		return;
//	trace_kfree_skb(skb, __builtin_return_address(0));
	__kfree_skb(skb);
}

// signal handler
int raise(int sig)
{
	return 0;
}

void init_linux()
{
	total_segments = 0;

	jiffies = 168000000;
// Point heap to the CCM SRAM
//	heap = (unsigned char*)0x10000000;
}









