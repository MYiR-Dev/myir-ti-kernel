/*
 * TI Keystone Common Platform Time Sync
 * Based on cpts.c by Richard Cochran <richardcochran@gmail.com>
 *
 * Copyright (C) 2014 Texas Instruments Incorporated
 * Authors:	Wingman Kwok <w-kwok2@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/err.h>
#include <linux/if.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/net_tstamp.h>
#include <linux/ptp_classify.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>

#include "netcp_cpts.h"

#ifdef CONFIG_TI_KEYSTONE_NETCP_CPTS
struct cpts_tc_ts_converter {
	u32 freq;
	u32 mult;
	u32 shift;
	u32 div;
};

struct cpts_tc_ts_converter cpts_tc_ts_default_converters[] = {
	{400000000, 2560, 10,  1},
	{425000000, 5120,  7, 17},
	{500000000, 2048, 10,  1},
	{600000000, 5120, 10,  3},
	{614400000, 5000, 10,  3},
	{625000000, 4096,  9,  5},
	{675000000, 5120,  7, 27},
	{700000000, 5120,  9,  7},
	{750000000, 4096, 10,  3},
};

#define cpts_read32(c, r)	__raw_readl(&c->reg->r)
#define cpts_write32(c, v, r)	__raw_writel(v, &c->reg->r)

#define CPTS_COUNTER_BITS	32
#define TS_COMP_LAST_LO(cpts)	(cpts->ts_comp_last & cpts->cc.mask)

#define VLAN_IPV4_HLEN(data) \
	(((struct iphdr *)(data + OFF_IHL + VLAN_HLEN))->ihl << 2)

static u64 cpts_ppb_correct_ns(struct cpts *cpts, u64 ns)
{
	s32 ppb = cpts->ppb;
	int neg_adj = 0;
	u64 temp_ns;

	if (ppb == 0)
		return ns;

	if (ppb < 0) {
		neg_adj = 1;
		ppb = -ppb;
	}

	temp_ns = ns * ppb;
	do_div(temp_ns, 1000000000);
	return ns + (neg_adj ? -temp_ns : temp_ns);
}

/* Assumes ppb < 1000000000 */
static u64 cpts_ppb_correct_cyc(struct cpts *cpts, u64 cyc)
{
	u64 temp_cyc;

	if (cpts->ppb == 0)
		return cyc;

	temp_cyc = cyc * 1000000000;
	do_div(temp_cyc, 1000000000 + cpts->ppb);
	return temp_cyc;
}

static inline u64 cpts_cyc2ns(cycle_t cycles, u32 mult, u32 shift, u32 div)
{
	u64 ns;

	ns = ((cycles * mult) >> shift);
	do_div(ns, div);
	return ns;
}

static u64 cpts_cc_cyc2ns(struct cpts *cpts, cycle_t cycles)
{
	const struct cyclecounter *cc = &cpts->cc;
	u64 nsec = 0;
	u64 corrected_ns;

	/* to prevent overflow in cpts_cyc2ns */
	while (cycles >= cpts->max_cycles) {
		nsec += cpts->max_nsec;
		cycles -= cpts->max_cycles;
	}

	nsec += cpts_cyc2ns(cycles, cc->mult, cc->shift, cpts->cc_div);

	corrected_ns = cpts_ppb_correct_ns(cpts, nsec);

	return nsec;
}

static cycle_t cpts_cc_read_delta(struct cpts *cpts)
{
	struct timecounter *tc = &cpts->tc;
	cycle_t cycle_now, cycle_delta;

	/* read cycle counter: */
	cycle_now = tc->cc->read(tc->cc);

	/* calculate the cc delta since the last cpts_cc_read_delta(): */
	cycle_delta = (cycle_now - tc->cycle_last) & tc->cc->mask;

	/* update time stamp of cpts_cc_read_delta() call: */
	tc->cycle_last = cycle_now;

	return cycle_delta;
}

/* With cc's mult, calculate the max cycles that can be taken in
 * cyc2ns without overflow. Also calculates the corresponding ns
 */
void cpts_cyc2ns_set_max_cap(struct cpts *cpts)
{
	cpts->max_cycles = 0xffffffffffffffff;
	do_div(cpts->max_cycles, cpts->cc.mult);
	cpts->max_nsec = cpts_cyc2ns(cpts->max_cycles, cpts->cc.mult,
					cpts->cc.shift, cpts->cc_div);
	cpts->max_nsec = cpts_ppb_correct_ns(cpts, cpts->max_nsec);
}

/* timecounter_read - return nanoseconds elapsed since timecounter_init()
 *                    plus the initial time stamp
 */
static u64 cpts_timecounter_read(struct cpts *cpts)
{
	cpts->cc_total += cpts_cc_read_delta(cpts);
	if (cpts->cc_total >= cpts->max_cycles) {
		cpts->cc_total -= cpts->max_cycles;
		cpts->tc.nsec = cpts->max_nsec +
				cpts_cc_cyc2ns(cpts, cpts->cc_total);
	} else {
		cpts->tc.nsec = cpts_cc_cyc2ns(cpts, cpts->cc_total);
	}

	return cpts->tc_base + cpts->tc.nsec;
}

u64 cpts_tstamp_cyc2time(struct cpts *cpts, cycle_t cycle_tstamp)
{
	struct timecounter *tc = &cpts->tc;
	u64 cycle_delta = (cycle_tstamp - tc->cycle_last) & tc->cc->mask;
	u64 nsec;

	/* Instead of always treating cycle_tstamp as more recent
	 * than tc->cycle_last, detect when it is too far in the
	 * future and treat it as old time stamp instead.
	 */
	if (cycle_delta > tc->cc->mask / 2) {
		cycle_delta = (tc->cycle_last - cycle_tstamp) & tc->cc->mask;
		nsec = tc->nsec - cpts_cc_cyc2ns(cpts, cycle_delta);
	} else {
		nsec = cpts_cc_cyc2ns(cpts, cycle_delta) + tc->nsec;
	}

	return cpts->tc_base + nsec;
}

void cpts_timecounter_init(struct cpts *cpts, u64 start_tstamp)
{
	timecounter_init(&cpts->tc, &cpts->cc, 0);
	cpts->cc_total = 0;
	cpts->tc_base = start_tstamp;
}

static int event_expired(struct cpts_event *event)
{
	return time_after(jiffies, event->tmo);
}

static int event_type(struct cpts_event *event)
{
	return (event->high >> EVENT_TYPE_SHIFT) & EVENT_TYPE_MASK;
}

static int event_port(struct cpts_event *event)
{
	return (event->high >> PORT_NUMBER_SHIFT) & PORT_NUMBER_MASK;
}

static int cpts_fifo_pop(struct cpts *cpts, u32 *high, u32 *low)
{
	u32 r = cpts_read32(cpts, intstat_raw);

	if (r & TS_PEND_RAW) {
		*high = cpts_read32(cpts, event_high);
		*low  = cpts_read32(cpts, event_low);
		cpts_write32(cpts, EVENT_POP, event_pop);
		return 0;
	}
	return -1;
}

static int cpts_event_list_clean_up(struct cpts *cpts, int ev_type)
{
	struct list_head *this, *next;
	struct cpts_event *event;
	int removed = 0;

	list_for_each_safe(this, next, &cpts->events) {
		event = list_entry(this, struct cpts_event, list);
		if (event_expired(event) || (ev_type == event_type(event))) {
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			++removed;
		}
	}
	return removed;
}

static int cpts_match(struct sk_buff *skb, unsigned int ptp_class,
		      u16 ts_seqid, u8 ts_msgtype)
{
	u16 *seqid;
	unsigned int offset = 0;
	u8 *msgtype, *data = skb->data;

	if (ptp_class & PTP_CLASS_VLAN)
		offset += VLAN_HLEN;

	switch (ptp_class & PTP_CLASS_PMASK) {
	case PTP_CLASS_IPV4:
		offset += ETH_HLEN + IPV4_HLEN(data + offset) + UDP_HLEN;
		break;
	case PTP_CLASS_IPV6:
		offset += ETH_HLEN + IP6_HLEN + UDP_HLEN;
		break;
	case PTP_CLASS_L2:
		offset += ETH_HLEN;
		break;
	default:
		return 0;
	}

	if (skb->len + ETH_HLEN < offset + OFF_PTP_SEQUENCE_ID + sizeof(*seqid))
		return 0;

	if (unlikely(ptp_class & PTP_CLASS_V1))
		msgtype = data + offset + OFF_PTP_CONTROL;
	else
		msgtype = data + offset;

	seqid = (u16 *)(data + offset + OFF_PTP_SEQUENCE_ID);

	return (ts_msgtype == (*msgtype & 0xf) && ts_seqid == ntohs(*seqid));
}

static bool cpts_match_tx_ts(struct cpts *cpts, struct cpts_event *event)
{
	struct sk_buff *skb, *tmp;
	u16 seqid;
	u8 mtype;
	unsigned long flags;
	ktime_t timeout;
	bool found = false;

	mtype = (event->high >> MESSAGE_TYPE_SHIFT) & MESSAGE_TYPE_MASK;
	seqid = (event->high >> SEQUENCE_ID_SHIFT) & SEQUENCE_ID_MASK;
	timeout = ktime_sub_ns(ktime_get_real(), NSEC_PER_SEC); /* 1s timeout */

	spin_lock_irqsave(&cpts->tx_ts_queue.lock, flags);
	skb_queue_walk_safe(&cpts->tx_ts_queue, skb, tmp) {
		struct skb_shared_hwtstamps ssh;
		unsigned int class = ptp_classify_raw(skb);

		if (cpts_match(skb, class, seqid, mtype)) {
			u64 ns = cpts_tstamp_cyc2time(cpts, event->low);

			memset(&ssh, 0, sizeof(ssh));
			ssh.hwtstamp = ns_to_ktime(ns);
			skb_tstamp_tx(skb, &ssh);
			found = true;
			__skb_unlink(skb, &cpts->tx_ts_queue);
			consume_skb(skb);
		} else if (ktime_compare(timeout, skb_get_ktime(skb)) >= 0) {
			/* timeout any expired skbs over 1s */
			dev_err(cpts->dev,
				"expiring tx timestamp mtype %u seqid %04x\n",
				mtype, seqid);
			__skb_unlink(skb, &cpts->tx_ts_queue);
			kfree_skb(skb);
		}
	}
	spin_unlock_irqrestore(&cpts->tx_ts_queue.lock, flags);

	return found;
}

/* Returns zero if matching event type was found. */
static int cpts_fifo_read(struct cpts *cpts, int match)
{
	int i, type = -1, removed;
	u32 hi, lo;
	struct cpts_event *event;
	bool found = false;

	for (i = 0; i < CPTS_FIFO_DEPTH; i++) {
		if (cpts_fifo_pop(cpts, &hi, &lo))
			break;
		if (list_empty(&cpts->pool)) {
			removed = cpts_event_list_clean_up(cpts, -1);
			if (!removed) {
				pr_err("cpts: event pool is empty\n");
				return -1;
			}
			pr_debug("cpts: list cleaned up %d\n", removed);
		}
		event = list_first_entry(&cpts->pool, struct cpts_event, list);
		event->tmo = jiffies + CPTS_TMO;
		event->high = hi;
		event->low = lo;
		type = event_type(event);
		switch (type) {
		case CPTS_EV_HW:
		case CPTS_EV_COMP:
			event->tmo += (CPTS_COMP_TMO - CPTS_TMO);
		case CPTS_EV_TX:
			if (cpts_match_tx_ts(cpts, event)) {
				/* if the new event matches an existing skb,
				 * then don't queue it
				 */
				break;
			}
		case CPTS_EV_PUSH:
		case CPTS_EV_RX:
			list_del_init(&event->list);
			list_add_tail(&event->list, &cpts->events);
			break;
		case CPTS_EV_ROLL:
		case CPTS_EV_HALF:
			break;
		default:
			pr_err("cpts: unknown event type\n");
			break;
		}
		if (type == match)
			found = true;
	}
	return found ? 0 : -1;
}

static cycle_t cpts_systim_read(const struct cyclecounter *cc)
{
	u64 val = 0;
	struct cpts_event *event;
	struct list_head *this, *next;
	struct cpts *cpts = container_of(cc, struct cpts, cc);

	cpts_write32(cpts, TS_PUSH, ts_push);
	if (cpts_fifo_read(cpts, CPTS_EV_PUSH))
		pr_err("cpts: unable to obtain a time stamp\n");

	list_for_each_safe(this, next, &cpts->events) {
		event = list_entry(this, struct cpts_event, list);
		if (event_type(event) == CPTS_EV_PUSH) {
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			val = event->low;
			break;
		}
	}

	return val;
}

static u64 cpts_cc_ns2cyc(struct cpts *cpts, u64 nsec)
{
	u64 max_ns, max_cyc, cyc = 0;
	u64 dividend;
	u64 corrected_cyc;

	max_cyc = (1ULL << CPTS_COUNTER_BITS) - 1;
	max_ns = cpts_cc_cyc2ns(cpts, max_cyc);

	while (nsec >= max_ns) {
		nsec -= max_ns;
		cyc += max_cyc;
	}

	if (nsec) {
		dividend = (nsec * cpts->cc_div) << cpts->cc.shift;
		cyc += div_u64(dividend, cpts->cc.mult);
	}

	corrected_cyc = cpts_ppb_correct_cyc(cpts, cyc);
	return corrected_cyc;
}

static inline void cpts_ts_comp_add_ns(struct cpts *cpts, s64 add_ns)
{
	u64 temp, cyc;
	int neg_adj = 0;

	if (add_ns < 0) {
		neg_adj = 1;
		add_ns = -add_ns;
	}

	if (add_ns == NSEC_PER_SEC) {
		/* avoid calculation */
		cpts->ts_comp_last += cpts->pps_one_sec;
		return;
	}

	temp = add_ns;
	cyc = cpts_cc_ns2cyc(cpts, temp);
	cpts->ts_comp_last += (neg_adj ? -cyc : cyc);
}

static inline void cpts_disable_ts_comp(struct cpts *cpts)
{
	cpts_write32(cpts, 0, ts_comp_length);
}

static inline void cpts_enable_ts_comp(struct cpts *cpts)
{
	cpts_write32(cpts, TS_COMP_LAST_LO(cpts), ts_comp_val);
	cpts_write32(cpts, cpts->ts_comp_length, ts_comp_length);
}

static int cpts_ts_comp_add_reload(struct cpts *cpts, s64 add_ns, int enable)
{
	struct list_head *this, *next;
	struct ptp_clock_event pevent;
	struct cpts_event *event;
	int reported = 0;
	u64 ns;

	list_for_each_safe(this, next, &cpts->events) {
		event = list_entry(this, struct cpts_event, list);
		if (event_type(event) == CPTS_EV_COMP) {
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			if (TS_COMP_LAST_LO(cpts) != event->low) {
				pr_err("cpts ts_comp mismatch: %llx %08x\n",
				       cpts->ts_comp_last, event->low);
				break;
			}
			pr_debug("cpts comp ev tstamp: %u\n",
				 event->low);

			/* report the event */
			ns = cpts_tstamp_cyc2time(cpts, event->low);
			pevent.type = PTP_CLOCK_PPSUSR;
			pevent.pps_times.ts_real = ns_to_timespec(ns);
			ptp_clock_event(cpts->clock, &pevent);
			reported = 1;

			/* reload: add ns to ts_comp */
			cpts_ts_comp_add_ns(cpts, add_ns);
			if (enable) {
				/* enable ts_comp pulse with new val */
				cpts_disable_ts_comp(cpts);
				cpts_enable_ts_comp(cpts);
			}
			break;
		}
	}
	return reported;
}

static int cpts_ts_comp_hw_ts_ev_report_restart(struct cpts *cpts)
{
	struct list_head *this, *next;
	struct ptp_clock_event pevent;
	struct cpts_event *event;
	int reported = 0, ev;
	u64 ns;

	list_for_each_safe(this, next, &cpts->events) {
		event = list_entry(this, struct cpts_event, list);
		ev = event_type(event);
		if (ev == CPTS_EV_COMP) {
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			if (TS_COMP_LAST_LO(cpts) != event->low) {
				pr_err("cpts ts_comp mismatch: %llx %08x\n",
				       cpts->ts_comp_last, event->low);
				continue;
			}
			pr_debug("cpts comp ev tstamp: %u\n",
				 event->low);

			/* report the event */
			ns = cpts_tstamp_cyc2time(cpts, event->low);
			pevent.type = PTP_CLOCK_PPSUSR;
			pevent.pps_times.ts_real = ns_to_timespec(ns);
			ptp_clock_event(cpts->clock, &pevent);
			++reported;

			/* reload: add ns to ts_comp */
			cpts_ts_comp_add_ns(cpts, NSEC_PER_SEC);
			/* enable ts_comp pulse with new val */
			cpts_disable_ts_comp(cpts);
			cpts_enable_ts_comp(cpts);
			continue;
		} else if (ev == CPTS_EV_HW) {
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			/* report the event */
			pevent.timestamp =
				cpts_tstamp_cyc2time(cpts, event->low);
			pevent.type = PTP_CLOCK_EXTTS;
			pevent.index = event_port(event) - 1;
			ptp_clock_event(cpts->clock, &pevent);
			++reported;
			continue;
		}
	}
	return reported;
}

/* PTP clock operations */

static void cpts_tc_settime(struct cpts *cpts, u64 now)
{
	cycle_t cycles_to_pulse;
	u64 ns_to_pulse, pulse_time;
	u32 remainder;

	if (cpts->pps_enable) {
		cpts_disable_ts_comp(cpts);
		cpts_fifo_read(cpts, CPTS_EV_COMP);
		/* before adj, report existing pulse, if any,
		 * and add 1 sec to local ts_comp counter,
		 * but don't start the pps yet
		 */
		cpts_ts_comp_add_reload(cpts, NSEC_PER_SEC, 0);
	}

	/* set the ptp clock time */
	cpts->tc.nsec = 0;
	cpts->cc_total = 0;
	cpts->tc_base = now;

	if (cpts->pps_enable) {
		/* adjust ts_comp and start it */
		cycles_to_pulse =
			(TS_COMP_LAST_LO(cpts) - cpts->tc.cycle_last) &
			cpts->cc.mask;
		ns_to_pulse = cpts_cc_cyc2ns(cpts, cycles_to_pulse);
		pulse_time = cpts->tc_base + ns_to_pulse;
		/* align pulse time to next sec boundary */
		div_u64_rem(pulse_time, 1000000000, &remainder);
		cpts_ts_comp_add_ns(cpts, NSEC_PER_SEC - remainder);
		/* enable ts_comp pulse */
		cpts_enable_ts_comp(cpts);
	}
}

static int cpts_ptp_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	struct cpts *cpts = container_of(ptp, struct cpts, info);
	u64 ns_to_pulse = 0, pulse_time, now;
	cycle_t cycles_to_pulse;
	unsigned long flags;
	u32 remainder;

	if (cpts->ignore_adjfreq) {
		pr_debug("%s: ignored ppb = %d\n", __func__, ppb);
		return 0;
	}

	spin_lock_irqsave(&cpts->lock, flags);
	now = cpts_timecounter_read(cpts);

	if (cpts->pps_enable) {
		cpts_disable_ts_comp(cpts);
		cpts_fifo_read(cpts, CPTS_EV_COMP);
		/* before adj, if any, report existing pulse
		 * and add 1 sec to local ts_comp counter,
		 * but don't start the pps yet
		 */
		cpts_ts_comp_add_reload(cpts, NSEC_PER_SEC, 0);
		cycles_to_pulse =
			(TS_COMP_LAST_LO(cpts) - cpts->tc.cycle_last) &
			cpts->cc.mask;
		ns_to_pulse = cpts_cc_cyc2ns(cpts, cycles_to_pulse);
	}

	/* set the ptp clock time & new freq */
	cpts->tc.nsec = 0;
	cpts->cc_total = 0;
	cpts->tc_base = now;
	cpts->ppb = ppb;
	cpts_cyc2ns_set_max_cap(cpts);
	cpts->pps_one_sec = cpts_cc_ns2cyc(cpts, NSEC_PER_SEC);

	if (cpts->pps_enable) {
		/* adjust ts_comp based on new freq and start it */
		cycles_to_pulse = cpts_cc_ns2cyc(cpts, ns_to_pulse);
		cpts->ts_comp_last = (cycles_to_pulse + cpts->tc.cycle_last) &
					cpts->cc.mask;
		pulse_time = cpts->tc_base + ns_to_pulse;
		/* align pulse time to next sec boundary */
		div_u64_rem(pulse_time, 1000000000, &remainder);
		cpts_ts_comp_add_ns(cpts, NSEC_PER_SEC - remainder);
		/* enable ts_comp pulse */
		cpts_enable_ts_comp(cpts);
	}
	spin_unlock_irqrestore(&cpts->lock, flags);

	return 0;
}

static int cpts_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	s64 now;
	unsigned long flags;
	struct cpts *cpts = container_of(ptp, struct cpts, info);

	spin_lock_irqsave(&cpts->lock, flags);
	now = cpts_timecounter_read(cpts) + delta;
	cpts_tc_settime(cpts, now);
	spin_unlock_irqrestore(&cpts->lock, flags);
	return 0;
}

static int cpts_ptp_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	u64 ns;
	u32 remainder;
	unsigned long flags;
	struct cpts *cpts = container_of(ptp, struct cpts, info);

	spin_lock_irqsave(&cpts->lock, flags);
	ns = cpts_timecounter_read(cpts);
	spin_unlock_irqrestore(&cpts->lock, flags);

	ts->tv_sec = div_u64_rem(ns, 1000000000, &remainder);
	ts->tv_nsec = remainder;

	return 0;
}

static int cpts_ptp_settime(struct ptp_clock_info *ptp,
			    const struct timespec64 *ts)
{
	u64 ns;
	unsigned long flags;
	struct cpts *cpts = container_of(ptp, struct cpts, info);

	ns = timespec64_to_ns(ts);

	spin_lock_irqsave(&cpts->lock, flags);
	cpts_timecounter_read(cpts);
	cpts_tc_settime(cpts, ns);
	spin_unlock_irqrestore(&cpts->lock, flags);
	return 0;
}

/* PPS */
static int cpts_pps_enable(struct cpts *cpts, int on)
{
	unsigned long flags;
	struct timespec64 ts;
	u32 v;

	if (cpts->pps_enable == on)
		return 0;

	cpts->pps_enable = on;

	if (!on)
		return 0;

	/* get current counter value */
	v = cpts_read32(cpts, control) | CPTS_EN;
	cpts_write32(cpts, v, control);
	cpts_write32(cpts, TS_PEND_EN, int_enable);
	cpts_ptp_gettime(&cpts->info, &ts);
	cpts->ts_comp_last = cpts->tc.cycle_last;
	/* align to next sec boundary and add one sec */
	cpts_ts_comp_add_ns(cpts, 2 * NSEC_PER_SEC - ts.tv_nsec);
	/* remove stale TS_COMP events */
	spin_lock_irqsave(&cpts->lock, flags);
	cpts_event_list_clean_up(cpts, CPTS_EV_COMP);
	/* enable ts_comp pulse */
	cpts_disable_ts_comp(cpts);
	cpts_enable_ts_comp(cpts);
	spin_unlock_irqrestore(&cpts->lock, flags);
	return 0;
}

static int cpts_pps_init(struct cpts *cpts)
{
	cpts->pps_one_sec = cpts_cc_ns2cyc(cpts, NSEC_PER_SEC);
	return 0;
}

/* HW TS */
static int cpts_extts_enable(struct cpts *cpts, u32 index, int on)
{
	u32 v;

	if (index >= cpts->info.n_ext_ts)
		return -ENXIO;

	if (((cpts->hw_ts_enable & BIT(index)) >> index) == on)
		return 0;

	v = cpts_read32(cpts, control);
	if (on) {
		v |= BIT(8 + index);
		cpts->hw_ts_enable |= BIT(index);
	} else {
		v &= ~BIT(8 + index);
		cpts->hw_ts_enable &= ~BIT(index);
	}
	cpts_write32(cpts, v, control);

	return 0;
}

static int cpts_ptp_enable(struct ptp_clock_info *ptp,
			   struct ptp_clock_request *rq, int on)
{
	struct cpts *cpts = container_of(ptp, struct cpts, info);

	switch (rq->type) {
	case PTP_CLK_REQ_PPS:
		return cpts_pps_enable(cpts, on);
	case PTP_CLK_REQ_EXTTS:
		return cpts_extts_enable(cpts, rq->extts.index, on ? 1 : 0);
	default:
		break;
	}
	return -EOPNOTSUPP;
}

static struct ptp_clock_info cpts_info = {
	.owner		= THIS_MODULE,
	.name		= "CPTS timer",
	.max_adj	= 1000000,
	.n_ext_ts	= 8,
	.pps		= 1,
	.adjfreq	= cpts_ptp_adjfreq,
	.adjtime	= cpts_ptp_adjtime,
	.gettime64	= cpts_ptp_gettime,
	.settime64	= cpts_ptp_settime,
	.enable		= cpts_ptp_enable,
};

static int cpts_check_ts_comp_hw_ts_ev(struct cpts *cpts)
{
	unsigned long flags;

	spin_lock_irqsave(&cpts->lock, flags);
	cpts_ts_comp_hw_ts_ev_report_restart(cpts);
	spin_unlock_irqrestore(&cpts->lock, flags);
	return 0;
}

static void cpts_overflow_check(struct work_struct *work)
{
	struct cpts *cpts = container_of(work, struct cpts, overflow_work.work);
	struct timespec64 ts;
	u32 v;

	v = cpts_read32(cpts, control) | CPTS_EN;
	cpts_write32(cpts, v, control);
	cpts_write32(cpts, TS_PEND_EN, int_enable);
	cpts_ptp_gettime(&cpts->info, &ts);
	pr_debug("cpts overflow check at %lld.%09lu\n", ts.tv_sec, ts.tv_nsec);
	if (!cpts->pps_enable)
		cpts_disable_ts_comp(cpts);
	if (cpts->pps_enable || cpts->hw_ts_enable)
		cpts_check_ts_comp_hw_ts_ev(cpts);
	schedule_delayed_work(&cpts->overflow_work, CPTS_OVERFLOW_PERIOD);
}

static int cpts_get_rftclk_default_converters(struct cpts *cpts)
{
	struct cpts_tc_ts_converter *c = &cpts_tc_ts_default_converters[0];
	struct cpts_tc_ts_converter *d_min_conv = NULL;
	u32 a_size = ARRAY_SIZE(cpts_tc_ts_default_converters);
	u32 f = cpts->rftclk_freq;
	u32 d_min = 0xffffffff;
	u32 i, d;

	for (i = 0, c = &cpts_tc_ts_default_converters[0];
					i < a_size; i++, c++) {
		d = abs(f - c->freq);
		if (d < d_min) {
			d_min = d;
			d_min_conv = c;
		}
	}

	/* if ref clk freq differs from freq in the defaults
	 * for more than 1 MHz, do not return any multi/div/shif.
	 * 1MHz is arbitrary but reasonable
	 */
	if (d_min >= 1000000)
		return -1;

	cpts->cc.mult	= d_min_conv->mult;
	cpts->cc.shift	= d_min_conv->shift;
	cpts->cc_div	= d_min_conv->div;

	return 0;
}

static void cpts_clk_init(struct device *dev, struct cpts *cpts)
{
	unsigned long rate;
	u64 max_sec;

	cpts->refclk = devm_clk_get(cpts->dev, "cpts");
	if (IS_ERR(cpts->refclk)) {
		dev_info(dev, "No refclk defined.  Assumes external refclk.\n");
		cpts->refclk = NULL;
	} else {
		cpts->rftclk_freq = clk_get_rate(cpts->refclk);
	}

	if (!cpts->cc.mult && !cpts->cc.shift) {
		if (cpts_get_rftclk_default_converters(cpts)) {
			/* calculate the multiplier/shift to
			 * convert CPTS counter ticks to ns.
			 */
			rate = cpts->rftclk_freq;
			max_sec = ((1ULL << CPTS_COUNTER_BITS) - 1) +
								(rate - 1);
			do_div(max_sec, rate);

			clocks_calc_mult_shift(&cpts->cc.mult, &cpts->cc.shift,
					       rate, NSEC_PER_SEC, max_sec);
			cpts->cc_div = 1;
		}
	}

	pr_info("cpts rftclk: sel(%d),rate(%u HZ),mult(%u),shift(%u),div(%u)\n",
		cpts->rftclk_sel, cpts->rftclk_freq, cpts->cc.mult,
		cpts->cc.shift, cpts->cc_div);

	if (cpts->refclk)
		clk_prepare_enable(cpts->refclk);

	cpts_write32(cpts, cpts->rftclk_sel & 0x1f, rftclk_sel);
}

static void cpts_clk_release(struct cpts *cpts)
{
	if (!cpts->refclk)
		return;

	clk_disable(cpts->refclk);
	clk_put(cpts->refclk);
}

static u64 cpts_find_ts(struct cpts *cpts, struct sk_buff *skb, int ev_type)
{
	u64 ns = 0;
	struct cpts_event *event;
	struct list_head *this, *next;
	unsigned int class = ptp_classify_raw(skb);
	unsigned long flags;
	u16 seqid;
	u8 mtype;

	if (class == PTP_CLASS_NONE)
		return 0;

	spin_lock_irqsave(&cpts->lock, flags);
	cpts_fifo_read(cpts, CPTS_EV_PUSH);
	list_for_each_safe(this, next, &cpts->events) {
		event = list_entry(this, struct cpts_event, list);
		if (event_expired(event)) {
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			continue;
		}
		mtype = (event->high >> MESSAGE_TYPE_SHIFT) & MESSAGE_TYPE_MASK;
		seqid = (event->high >> SEQUENCE_ID_SHIFT) & SEQUENCE_ID_MASK;
		if (ev_type == event_type(event) &&
		    cpts_match(skb, class, seqid, mtype)) {
			ns = cpts_tstamp_cyc2time(cpts, event->low);
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			break;
		}
	}

	if (ev_type == CPTS_EV_TX && !ns) {
		/* Not found, add frame to queue for processing later.
		 * The periodic FIFO check will handle this.
		 */
		skb_get(skb);
		/* get the timestamp for timeouts */
		__net_timestamp(skb);
		skb_queue_tail(&cpts->tx_ts_queue, skb);
	}

	spin_unlock_irqrestore(&cpts->lock, flags);

	return ns;
}

int netcp_cpts_rx_timestamp(struct cpts *cpts, struct sk_buff *skb)
{
	u64 ns;
	struct skb_shared_hwtstamps *ssh;

	if (!cpts->rx_enable)
		return -EPERM;
	ns = cpts_find_ts(cpts, skb, CPTS_EV_RX);
	if (!ns)
		return -ENOENT;
	ssh = skb_hwtstamps(skb);
	memset(ssh, 0, sizeof(*ssh));
	ssh->hwtstamp = ns_to_ktime(ns);
	return 0;
}

int netcp_cpts_tx_timestamp(struct cpts *cpts, struct sk_buff *skb)
{
	u64 ns;
	struct skb_shared_hwtstamps ssh;

	if (!(skb_shinfo(skb)->tx_flags & SKBTX_IN_PROGRESS))
		return -EPERM;
	ns = cpts_find_ts(cpts, skb, CPTS_EV_TX);
	if (!ns)
		return -ENOENT;
	memset(&ssh, 0, sizeof(ssh));
	ssh.hwtstamp = ns_to_ktime(ns);
	skb_tstamp_tx(skb, &ssh);
	return 0;
}
#endif /* CONFIG_TI_KEYSTONE_NETCP_CPTS */

/* If both mult and shift are passed in as 0, they will be
 * calculated based on the cpts rfclk frequency
 */
int netcp_cpts_register(struct device *dev, struct cpts *cpts,
			u32 mult, u32 shift, u32 div)
{
#ifdef CONFIG_TI_KEYSTONE_NETCP_CPTS
	int err, i;
	unsigned long flags;
	u32 v;

	cpts->info = cpts_info;
	cpts->clock = ptp_clock_register(&cpts->info, dev);
	if (IS_ERR(cpts->clock)) {
		err = PTR_ERR(cpts->clock);
		cpts->clock = NULL;
		return err;
	}
	spin_lock_init(&cpts->lock);
	skb_queue_head_init(&cpts->tx_ts_queue);

	cpts->dev = dev;
	cpts->cc.read = cpts_systim_read;
	cpts->cc.mask = CLOCKSOURCE_MASK(32);
	cpts->cc.mult = mult;
	cpts->cc.shift = shift;

	cpts->cc_div = div;
	cpts->cc_mult = mult;

	INIT_LIST_HEAD(&cpts->events);
	INIT_LIST_HEAD(&cpts->pool);
	for (i = 0; i < CPTS_MAX_EVENTS; i++)
		list_add(&cpts->pool_data[i].list, &cpts->pool);

	cpts_clk_init(dev, cpts);
	/* mult may be updated during clk init */
	cpts->cc_mult = cpts->cc.mult;

	v = cpts_read32(cpts, control) | CPTS_EN;
	if (cpts->ts_comp_polarity)
		v |= TS_COMP_POLARITY;
	else
		v &= ~TS_COMP_POLARITY;
	cpts_write32(cpts, v, control);
	cpts_write32(cpts, TS_PEND_EN, int_enable);

	cpts_cyc2ns_set_max_cap(cpts);

	if (cpts->info.pps)
		cpts_pps_init(cpts);

	spin_lock_irqsave(&cpts->lock, flags);
	cpts_timecounter_init(cpts, ktime_to_ns(ktime_get_real()));
	spin_unlock_irqrestore(&cpts->lock, flags);

	cpts->cc_total = cpts->tc.cycle_last;

	INIT_DELAYED_WORK(&cpts->overflow_work, cpts_overflow_check);
	schedule_delayed_work(&cpts->overflow_work, CPTS_OVERFLOW_PERIOD);

	cpts->phc_index = ptp_clock_index(cpts->clock);
#endif
	return 0;
}

void netcp_cpts_unregister(struct cpts *cpts)
{
#ifdef CONFIG_TI_KEYSTONE_NETCP_CPTS
	if (cpts->clock) {
		ptp_clock_unregister(cpts->clock);
		cancel_delayed_work_sync(&cpts->overflow_work);
	}
	if (cpts->refclk)
		cpts_clk_release(cpts);
#endif
}
