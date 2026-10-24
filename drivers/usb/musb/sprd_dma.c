// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Otto Pflüger
 *
 * Based on Spreadtrum downstream code,
 *  Copyright (c) 2018 Spreadtrum Co., Ltd.
 * and on musb_dma.c,
 *  Copyright 2005 Mentor Graphics Corporation
 *  Copyright (C) 2005-2007 by Texas Instruments
 */
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "musb_core.h"
#include "musb_gadget.h"
#include "sprd_dma.h"

#define MAX_NODES_PER_CHANNEL 2048

struct sprd_musb_dma_channel {
	struct dma_channel		channel;
	struct sprd_musb_dma_controller	*controller;
	u32				len;
	u8				idx;
	u8				epnum;
	u8				transmit;
	/* Peripheral RX: queued requests chained into the running list. */
	unsigned int			nreqs;
	/* Where the next list starts in this channel's node array. */
	unsigned int			next_node;
};

struct sprd_musb_dma_controller {
	struct dma_controller		controller;
	struct sprd_musb_dma_channel	channel[MUSB_DMA_CHANNELS+1];
	void				*private_data;
	void __iomem			*base;
	u32				used_channels;
	void				*linklist;
	dma_addr_t 			linklist_dma_addr;
};

static void dma_channel_release(struct dma_channel *channel);

static void dma_controller_stop(struct sprd_musb_dma_controller *controller)
{
	struct musb *musb = controller->private_data;
	struct dma_channel *channel;
	u8 bit;

	if (controller->used_channels != 0) {
		dev_err(musb->controller,
			"Stopping DMA controller while channel active\n");

		for (bit = 1; bit <= MUSB_DMA_CHANNELS; bit++) {
			if (controller->used_channels & (1 << bit)) {
				channel = &controller->channel[bit].channel;
				dma_channel_release(channel);

				if (!controller->used_channels)
					break;
			}
		}
	}
}

static struct dma_channel *dma_channel_allocate(struct dma_controller *c,
				struct musb_hw_ep *hw_ep, u8 transmit)
{
	struct sprd_musb_dma_controller *controller = container_of(c,
			struct sprd_musb_dma_controller, controller);
	struct sprd_musb_dma_channel *musb_channel = NULL;
	struct dma_channel *channel = NULL;
	u8 bit = hw_ep->epnum + (transmit ? 0 : 15);

	if (controller->used_channels & (1 << bit))
		return NULL;

	controller->used_channels |= (1 << bit);
	musb_channel = &(controller->channel[bit]);
	musb_channel->controller = controller;
	musb_channel->idx = bit;
	musb_channel->epnum = hw_ep->epnum;
	musb_channel->transmit = transmit;
	channel = &(musb_channel->channel);
	channel->private_data = musb_channel;
	channel->status = MUSB_DMA_STATUS_FREE;
	channel->max_len = 0xfffc * MAX_NODES_PER_CHANNEL + 3;
	/* Tx => mode 1; Rx => mode 0 */
	channel->desired_mode = transmit;
	channel->actual_len = 0;

	return channel;
}

static void dma_channel_release(struct dma_channel *channel)
{
	struct sprd_musb_dma_channel *musb_channel = channel->private_data;

	channel->actual_len = 0;

	musb_channel->controller->used_channels &= ~(1 << musb_channel->idx);

	channel->status = MUSB_DMA_STATUS_UNKNOWN;
}

/*
 * The channel's node array, and where the next list should start in it.
 * Lists rotate through the array rather than always starting at node 0:
 * handed the address of the list it had just finished, the engine
 * completed the new one at once with nothing transferred (seen right after
 * a short packet). The vendor driver walks its array the same way.
 */
static struct sprd_musb_linklist_node *
channel_linklist(struct sprd_musb_dma_channel *musb_channel, u64 *list_addr,
		 int *start)
{
	struct sprd_musb_dma_controller *controller = musb_channel->controller;
	unsigned long ll_offset = sizeof(struct sprd_musb_linklist_node) *
				  MAX_NODES_PER_CHANNEL * (musb_channel->idx - 1);

	if (musb_channel->next_node > MAX_NODES_PER_CHANNEL / 2)
		musb_channel->next_node = 0;
	*start = musb_channel->next_node;
	ll_offset += sizeof(struct sprd_musb_linklist_node) * *start;
	*list_addr = (u64)controller->linklist_dma_addr + ll_offset;
	return controller->linklist + ll_offset;
}

/*
 * Append nodes covering [dma_addr, dma_addr + len) at index idx, none of
 * them marked as the end of the list. Returns the number of nodes added,
 * 0 when the list is full.
 */
static int add_nodes(struct sprd_musb_linklist_node *linklist, int idx,
		     int limit, dma_addr_t dma_addr, u32 len)
{
	dma_addr_t next_boundary;
	u32 tmp_len;
	int n = 0;

	while (len > 0 && idx + n < limit) {
		struct sprd_musb_linklist_node *node = &linklist[idx + n];

		if (len < 0x10000)
			tmp_len = len;
		else
			tmp_len = 0xfffc;

		next_boundary = (dma_addr | 0xfffffff) + 1;
		if ((dma_addr + tmp_len) > next_boundary)
			tmp_len = (u32)(next_boundary - dma_addr);

		node->addr = (u32)dma_addr;
		node->data_addr = (dma_addr >> 32) & 0xf;
		node->frag_len = 32;
		node->blk_len = tmp_len;
		node->list_end = 0;
		node->sp = 0;
		node->ioc = 0;

		dma_addr += tmp_len;
		len -= tmp_len;
		n++;
	}
	return n;
}

/* Hand a prepared list to the engine and enable the channel. */
static void start_channel(struct sprd_musb_dma_channel *musb_channel,
			  u64 list_addr)
{
	void __iomem *mbase = musb_channel->controller->base;
	u8 bchannel = musb_channel->idx;
	u32 csr;

	wmb();

	/*
	 * Have the engine wait 9 more cycles before acting on the USB core's
	 * request length. Without it, the first IN transfer of more than one
	 * packet stalled (mass storage; ACM never sends more than a packet per
	 * request). The vendor driver writes it at every channel allocation;
	 * writing it at every start also survives a controller reset.
	 */
	musb_writel(mbase, MUSB_DMA_FRAG_WAIT, 0x8);

	/* set linklist pointer */
	musb_writel(mbase, MUSB_DMA_CHN_LLIST_PTR(bchannel), (u32)list_addr);
	musb_writel(mbase, MUSB_DMA_CHN_ADDR_H(bchannel),
		    ((list_addr >> 32) & 0xf) << 4);

	if (musb_channel->transmit) {
		/* enable linklist end interrupt */
		csr = musb_readl(mbase, MUSB_DMA_CHN_INTR(bchannel));
		csr |= CHN_LLIST_INT_EN | CHN_CLEAR_INT_EN;
		musb_writel(mbase, MUSB_DMA_CHN_INTR(bchannel), csr);
	} else {
		/* enable linklist end and rx last interrupt */
		csr = musb_readl(mbase, MUSB_DMA_CHN_INTR(bchannel));
		csr |= CHN_USBRX_INT_EN | CHN_LLIST_INT_EN | CHN_CLEAR_INT_EN;
		musb_writel(mbase, MUSB_DMA_CHN_INTR(bchannel), csr);
	}

	/* enable channel and trigger the transfer */
	csr = musb_readl(mbase, MUSB_DMA_CHN_PAUSE(bchannel));
	if (csr & CHN_CLR)
		musb_writel(mbase, MUSB_DMA_CHN_PAUSE(bchannel), 0);
	csr = musb_readl(mbase, MUSB_DMA_CHN_CFG(bchannel));
	csr |= CHN_EN;
	musb_writel(mbase, MUSB_DMA_CHN_CFG(bchannel), csr);
}

/* One transfer: a single buffer, as the core asked for. */
static void configure_channel(struct dma_channel *channel,
				dma_addr_t dma_addr, u32 len)
{
	struct sprd_musb_dma_channel *musb_channel = channel->private_data;
	struct musb *musb = musb_channel->controller->private_data;
	struct sprd_musb_linklist_node *linklist;
	u64 list_addr;
	int start, n;

	dev_dbg(musb->controller, "%cx len=%u on channel %d\n",
		musb_channel->transmit ? 'T' : 'R', len, musb_channel->idx);

	linklist = channel_linklist(musb_channel, &list_addr, &start);
	n = add_nodes(linklist, 0, MAX_NODES_PER_CHANNEL - start, dma_addr, len);
	linklist[n - 1].list_end = 1;
	linklist[n - 1].ioc = 1;
	musb_channel->nreqs = 0;
	musb_channel->next_node = start + n;
	start_channel(musb_channel, list_addr);
}

static bool request_chainable(struct musb_request *req)
{
	return req->map_state != UN_MAPPED &&
	       req->request.dma != DMA_ADDR_INVALID &&
	       req->request.length > req->request.actual;
}

/*
 * Peripheral RX: chain every queued request of the endpoint into one list,
 * so the engine runs from one buffer straight into the next. Between a
 * list's end and the next CHN_EN the engine takes no DMA request, and at
 * full OUT rate packets land in that gap on every transfer; with the next
 * request already in the list there is no gap. The vendor driver works
 * this way. Returns false when the head of the queue is not the request
 * the core asked for, in which case the caller programs just that one.
 */
static bool configure_rx_chain(struct dma_channel *channel,
			       struct musb_ep *musb_ep,
			       dma_addr_t dma_addr, u32 len)
{
	struct sprd_musb_dma_channel *musb_channel = channel->private_data;
	struct musb *musb = musb_channel->controller->private_data;
	struct sprd_musb_linklist_node *linklist;
	struct musb_request *req;
	u64 list_addr;
	int start, idx = 0, n;
	unsigned int nreqs = 0;

	req = next_request(musb_ep);
	if (!req || !request_chainable(req) ||
	    req->request.dma + req->request.actual != dma_addr ||
	    req->request.length - req->request.actual != len)
		return false;

	linklist = channel_linklist(musb_channel, &list_addr, &start);
	list_for_each_entry(req, &musb_ep->req_list, list) {
		if (!request_chainable(req))
			break;
		n = add_nodes(linklist, idx, MAX_NODES_PER_CHANNEL - start,
			      req->request.dma + req->request.actual,
			      req->request.length - req->request.actual);
		if (!n)
			break;
		idx += n;
		nreqs++;
	}
	linklist[idx - 1].list_end = 1;
	linklist[idx - 1].ioc = 1;
	musb_channel->nreqs = nreqs;
	musb_channel->next_node = start + idx;

	dev_dbg(musb->controller, "Rx len=%u (+%u queued) on channel %d\n",
		len, nreqs - 1, musb_channel->idx);
	start_channel(musb_channel, list_addr);
	return true;
}

static int dma_channel_program(struct dma_channel *channel,
				u16 packet_sz, u8 mode,
				dma_addr_t dma_addr, u32 len)
{
	struct sprd_musb_dma_channel *musb_channel = channel->private_data;
	struct musb *musb = musb_channel->controller->private_data;

	/* Refuse rather than panic, as the vendor driver does. */
	if (channel->status == MUSB_DMA_STATUS_UNKNOWN ||
	    channel->status == MUSB_DMA_STATUS_BUSY) {
		dev_warn_ratelimited(musb->controller,
				     "channel %d programmed while %s\n",
				     musb_channel->idx,
				     channel->status == MUSB_DMA_STATUS_BUSY ?
				     "busy" : "unallocated");
		return false;
	}

	channel->actual_len = 0;
	musb_channel->len = len;
	channel->status = MUSB_DMA_STATUS_BUSY;

	if (!musb_channel->transmit && !is_host_active(musb)) {
		struct musb_hw_ep *hw_ep = &musb->endpoints[musb_channel->epnum];
		struct musb_ep *musb_ep = hw_ep->is_shared_fifo ?
					  &hw_ep->ep_in : &hw_ep->ep_out;

		if (configure_rx_chain(channel, musb_ep, dma_addr, len))
			return true;
	}
	configure_channel(channel, dma_addr, len);

	return true;
}

static int dma_channel_abort(struct dma_channel *channel)
{
	struct sprd_musb_dma_channel *musb_channel = channel->private_data;
	void __iomem *mbase = musb_channel->controller->base;
	struct musb *musb = musb_channel->controller->private_data;

	u8 bchannel = musb_channel->idx;
	int offset;
	u16 csr;
	u32 pause;

	/*
	 * Ask the engine to clear the channel. It answers with
	 * CHN_CLEAR_INT_MASK_STATUS, and sprd_musb_dma_interrupt() disables
	 * the channel from there; configure_channel() clears a CHN_CLR left
	 * over from this before starting the channel again. Both of those
	 * halves are already here, but nothing asked for the clear, so the
	 * interrupt never came, the channel stayed enabled on a descriptor
	 * that had just been zeroed, and configure_channel() only ORs CHN_EN
	 * back into a register where it is already set -- no edge, no
	 * transfer. Anything submitted afterwards never completes.
	 *
	 * The core also sets MUSB_DMA_STATUS_CORE_ABORT before aborting on
	 * its error paths, which is every abort the host side does, so the
	 * clear has to cover that status too.
	 */
	if (channel->status == MUSB_DMA_STATUS_BUSY ||
	    channel->status == MUSB_DMA_STATUS_CORE_ABORT) {
		pause = musb_readl(mbase, MUSB_DMA_CHN_PAUSE(bchannel));
		pause |= CHN_CLR;
		musb_writel(mbase, MUSB_DMA_CHN_PAUSE(bchannel), pause);
	}

	if (channel->status == MUSB_DMA_STATUS_BUSY) {
		if (musb_channel->transmit) {
			offset = musb->io.ep_offset(musb_channel->epnum,
						MUSB_TXCSR);

			/*
			 * The programming guide says that we must clear
			 * the DMAENAB bit before the DMAMODE bit...
			 */
			csr = musb_readw(mbase, offset);
			csr &= ~(MUSB_TXCSR_AUTOSET | MUSB_TXCSR_DMAENAB);
			musb_writew(mbase, offset, csr);
			csr &= ~MUSB_TXCSR_DMAMODE;
			musb_writew(mbase, offset, csr);
		} else {
			offset = musb->io.ep_offset(musb_channel->epnum,
						MUSB_RXCSR);

			csr = musb_readw(mbase, offset);
			csr &= ~(MUSB_RXCSR_AUTOCLEAR |
				 MUSB_RXCSR_DMAENAB |
				 MUSB_RXCSR_DMAMODE);
			musb_writew(mbase, offset, csr);
		}

		musb_writel(mbase, MUSB_DMA_CHN_LLIST_PTR(bchannel), 0);
		musb_writel(mbase, MUSB_DMA_CHN_ADDR(bchannel), 0);
		channel->status = MUSB_DMA_STATUS_FREE;
	}

	return 0;
}

/*
 * A chained RX list ended: either every request in it filled up, or a short
 * packet ended it part way through one of them. Give back what is done,
 * then, if more requests are queued and a packet is already waiting, start
 * the next list; otherwise leave DMAENAB off and let the next packet's
 * endpoint interrupt arm it through musb_g_rx() (see rxstate()).
 */
static void sprd_rx_chain_complete(struct musb *musb,
				   struct sprd_musb_dma_channel *musb_channel)
{
	struct dma_channel *channel = &musb_channel->channel;
	struct musb_hw_ep *hw_ep = &musb->endpoints[musb_channel->epnum];
	struct musb_ep *musb_ep = hw_ep->is_shared_fifo ? &hw_ep->ep_in :
							  &hw_ep->ep_out;
	void __iomem *mbase = musb->mregs;
	void __iomem *epio = hw_ep->regs;
	unsigned int nreqs = musb_channel->nreqs, i, last;
	u32 remaining, addr, done = 0;
	struct musb_request *req;
	u16 csr;

	remaining = musb_readl(mbase, MUSB_DMA_CHN_LEN(musb_channel->idx)) >> 16;
	addr = musb_readl(mbase, MUSB_DMA_CHN_ADDR(musb_channel->idx));
	musb_channel->nreqs = 0;
	channel->status = MUSB_DMA_STATUS_FREE;
	musb_ep_select(mbase, musb_channel->epnum);
	/*
	 * Nothing is armed now: drop DMAENAB first, so that a packet landing
	 * from here on raises the endpoint interrupt instead of a DMA
	 * request nobody takes. Any packet already there is caught below.
	 */
	csr = musb_readw(epio, MUSB_RXCSR);
	csr &= ~(MUSB_RXCSR_AUTOCLEAR | MUSB_RXCSR_DMAENAB |
		 MUSB_RXCSR_DMAMODE);
	musb_writew(epio, MUSB_RXCSR, MUSB_RXCSR_P_WZC_BITS | csr);
	dev_dbg(musb->controller, "rx done ch%d remaining %u addr %08x\n",
		musb_channel->idx, remaining, addr);

	/*
	 * Which request did the list end in? With nothing remaining it ran
	 * to the end. Otherwise a short packet stopped it in the node whose
	 * end, less what remains, is where the engine's address points.
	 */
	last = nreqs - 1;
	if (remaining) {
		bool found = false;

		i = 0;
		list_for_each_entry(req, &musb_ep->req_list, list) {
			u32 start = (u32)(req->request.dma + req->request.actual);
			u32 len = req->request.length - req->request.actual;

			if (i >= nreqs)
				break;
			if (remaining <= len && start + len - remaining == addr) {
				last = i;
				done = len - remaining;
				found = true;
				break;
			}
			i++;
		}
		if (!found) {
			/* No match: assume the first request, as before. */
			req = next_request(musb_ep);
			last = 0;
			done = req->request.length - req->request.actual;
			if (remaining <= done)
				done -= remaining;
			dev_warn_ratelimited(musb->controller,
				"rx chain: addr %08x matches no node (remaining %u), taking request 0\n",
				addr, remaining);
		}
	}

	musb_ep_select(mbase, musb_channel->epnum);
	for (i = 0; i <= last; i++) {
		struct usb_request *request;

		req = next_request(musb_ep);
		if (!req)
			break;
		request = &req->request;
		if (i == last && remaining)
			request->actual += done;
		else
			request->actual = request->length;
		dev_dbg(musb->controller, "Rx actual_len=%u on channel %d\n",
			request->actual, musb_channel->idx);
		/* Drops and retakes the lock; the queue may grow meanwhile. */
		musb_g_giveback(musb_ep, request, 0);
		musb_ep_select(mbase, musb_channel->epnum);
	}

	req = musb_ep->desc ? next_request(musb_ep) : NULL;
	/*
	 * Only arm the next list with a packet already waiting (see
	 * rxstate()); otherwise its arrival interrupts and musb_g_rx() arms.
	 */
	if (!req || !request_chainable(req))
		return;
	csr = musb_readw(epio, MUSB_RXCSR);
	if (!(csr & MUSB_RXCSR_RXPKTRDY)) {
		dev_dbg(musb->controller, "rx chain wait ch%d rxcsr %04x\n",
			musb_channel->idx, csr);
		return;
	}
	if (dma_channel_program(channel, musb_ep->packet_sz, 0,
				req->request.dma + req->request.actual,
				req->request.length - req->request.actual)) {
		csr |= MUSB_RXCSR_AUTOCLEAR | MUSB_RXCSR_DMAMODE |
		       MUSB_RXCSR_DMAENAB;
		musb_writew(epio, MUSB_RXCSR, csr);
	}
}

static void sprd_musb_dma_complete(struct musb *musb,
				   struct sprd_musb_dma_channel *musb_channel)
{
	struct dma_channel *channel = &musb_channel->channel;
	void __iomem *mbase = musb->mregs;
	u32 remaining;

	if (musb_channel->nreqs) {
		sprd_rx_chain_complete(musb, musb_channel);
		return;
	}

	/* Incomplete transfers are not possible when transmitting */
	if (musb_channel->transmit) {
		channel->actual_len = musb_channel->len;
	} else {
		/*
		 * A short packet ends the receive early. The vendor driver
		 * takes what is left of the block from the high half of
		 * CHN_LEN; the channel's address register, used here before,
		 * sometimes reads unrelated values, and re-arming the channel
		 * on those threw received data away.
		 */
		remaining = musb_readl(mbase,
				MUSB_DMA_CHN_LEN(musb_channel->idx)) >> 16;
		if (remaining > musb_channel->len) {
			dev_warn_ratelimited(musb->controller,
				"rx remaining %u > len %u on channel %d\n",
				remaining, musb_channel->len,
				musb_channel->idx);
			remaining = 0;
		}
		channel->actual_len = musb_channel->len - remaining;
	}

	channel->status = MUSB_DMA_STATUS_FREE;

	dev_dbg(musb->controller, "%cx actual_len=%lu on channel %d\n",
		musb_channel->transmit ? 'T' : 'R', channel->actual_len,
		musb_channel->idx);
	musb_dma_completion(musb, musb_channel->epnum,
			    musb_channel->transmit);
}

irqreturn_t sprd_musb_dma_interrupt(struct musb *musb, u32 int_hsdma)
{
	struct sprd_musb_dma_controller *controller = container_of(
			musb->dma_controller,
			struct sprd_musb_dma_controller,
			controller);
	struct sprd_musb_dma_channel *musb_channel = NULL;
	void __iomem *mbase = musb->mregs;
	u8 bchannel;
	u32 intr;

	for (bchannel = 1; bchannel <= MUSB_DMA_CHANNELS; bchannel++) {
		if (int_hsdma & (1 << (bchannel - 1))) {
			musb_channel = (struct sprd_musb_dma_channel *)
					&(controller->channel[bchannel]);

			intr = musb_readl(mbase, MUSB_DMA_CHN_INTR(bchannel));

			if (intr & CHN_START_INT_MASK_STATUS) {
				intr |= CHN_START_INT_CLR;
				musb_writel(mbase, MUSB_DMA_CHN_INTR(bchannel), intr);
			}

			if (intr & CHN_CLEAR_INT_MASK_STATUS) {
				musb_writel(mbase, MUSB_DMA_CHN_PAUSE(bchannel), 0);
				musb_writel(mbase, MUSB_DMA_CHN_CFG(bchannel), 0);
			}

			if (intr & CHN_LLIST_INT_MASK_STATUS) {
				intr |= CHN_LLIST_INT_CLR | CHN_START_INT_CLR |
					CHN_FRAG_INT_CLR | CHN_BLK_INT_CLR |
					CHN_USBRX_LAST_INT_CLR;
				musb_writel(mbase, MUSB_DMA_CHN_INTR(bchannel), intr);

				sprd_musb_dma_complete(musb, musb_channel);
			}
		}
	}

	return IRQ_HANDLED;
}
EXPORT_SYMBOL_GPL(sprd_musb_dma_interrupt);

void sprd_musb_dma_controller_destroy(struct dma_controller *c)
{
	struct sprd_musb_dma_controller *controller = container_of(c,
			struct sprd_musb_dma_controller, controller);
	struct musb *musb = controller->private_data;

	if (controller->linklist) {
		dma_free_coherent(musb->controller,
				  sizeof(struct sprd_musb_linklist_node) *
				  MAX_NODES_PER_CHANNEL * MUSB_DMA_CHANNELS,
				  controller->linklist,
				  controller->linklist_dma_addr);
	}

	dma_controller_stop(controller);

	kfree(controller);
}
EXPORT_SYMBOL_GPL(sprd_musb_dma_controller_destroy);

static struct sprd_musb_dma_controller *
dma_controller_alloc(struct musb *musb, void __iomem *base)
{
	struct sprd_musb_dma_controller *controller;

	controller = kzalloc(sizeof(*controller), GFP_KERNEL);
	if (!controller)
		return NULL;

	controller->private_data = musb;
	controller->base = base;

	controller->controller.channel_alloc = dma_channel_allocate;
	controller->controller.channel_release = dma_channel_release;
	controller->controller.channel_program = dma_channel_program;
	controller->controller.channel_abort = dma_channel_abort;

	controller->linklist = dma_alloc_coherent(musb->controller,
			sizeof(struct sprd_musb_linklist_node) *
			MAX_NODES_PER_CHANNEL * MUSB_DMA_CHANNELS,
			&controller->linklist_dma_addr, GFP_KERNEL);
	if (!controller->linklist) {
		dev_err(musb->controller, "Failed to allocate DMA list\n");
		sprd_musb_dma_controller_destroy(&controller->controller);
		return NULL;
	}

	return controller;
}

struct dma_controller *
sprd_musb_dma_controller_create(struct musb *musb, void __iomem *base)
{
	struct sprd_musb_dma_controller *controller;
	int i;

	controller = dma_controller_alloc(musb, base);
	if (!controller)
		return NULL;

	/* reset DMA to avoid unexpected problems if USB was already enabled */
	for (i = 1; i <= MUSB_DMA_CHANNELS; i++) {
		musb_writel(base, MUSB_DMA_CHN_PAUSE(i), CHN_CLR);
		musb_writel(base, MUSB_DMA_CHN_CFG(i), 0);
		musb_writel(base, MUSB_DMA_CHN_INTR(i), CHN_CLEAR_INT_EN |
			CHN_LLIST_INT_CLR | CHN_START_INT_CLR |
			CHN_FRAG_INT_CLR | CHN_BLK_INT_CLR |
			CHN_USBRX_LAST_INT_CLR);
		musb_writel(base, MUSB_DMA_CHN_ADDR(i), 0);
		musb_writel(base, MUSB_DMA_CHN_LLIST_PTR(i), 0);
		musb_writel(base, MUSB_DMA_CHN_ADDR_H(i), 0);
	}

	return &controller->controller;
}
EXPORT_SYMBOL_GPL(sprd_musb_dma_controller_create);
