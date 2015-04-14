/*
 *	ktree.h - Some generic list helpers, extending struct list_head a bit.
 *
 *	Implementations are found in lib/ktree.c
 *
 *
 *	Copyright (C) 2005 Patrick Mochel
 *
 *	This file is rleased under the GPL v2.
 */

#ifndef _LINUX_KTREE_H
#define _LINUX_KTREE_H

#include <linux/spinlock.h>
#include <linux/kref.h>
#include <linux/list.h>

struct ktree_node;

struct ktree {
	spinlock_t		lock; /* protects shared access to tree */
	struct ktree_node	*root;
	void			(*get)(struct ktree_node *);
	void			(*put)(struct ktree_node *);
};

#define KTREE_INIT(_name, _get, _put)					\
	{ .lock	= __SPIN_LOCK_UNLOCKED(_name.lock),			\
	  .get		= _get,						\
	  .put		= _put, }

#define DEFINE_KTREE(_name, _get, _put)					\
	struct ktree _name = KTREE_INIT(_name, _get, _put)

void ktree_init(struct ktree *ktree, void (*get)(struct ktree_node *),
		void (*put)(struct ktree_node *));

struct ktree_node {
	struct ktree		*ktree;
	bool			deleted;
	struct ktree_node	*parent;
	struct list_head	siblings;
	struct list_head	children;
	struct kref		refcount;
};

void ktree_set_root(struct ktree *ktree, struct ktree_node *root);

static inline struct ktree_node *ktree_get_node(struct ktree_node *node)
{
	if (node)
		kref_get(&node->refcount);
	return node;
}

static inline struct ktree_node *ktree_get_root(struct ktree *ktree)
{
	return ktree_get_node(ktree->root);
}

static inline struct ktree_node *ktree_get_parent(struct ktree_node *node)
{
	return ktree_get_node(node->parent);
}

void ktree_put_node(struct ktree_node *node);

void ktree_add_child_after(struct ktree_node *parent, struct ktree_node *child,
			   struct ktree_node *pos);
void ktree_add_child_before(struct ktree_node *parent, struct ktree_node *child,
			    struct ktree_node *pos);

static inline void ktree_add_child_first(struct ktree_node *parent,
					 struct ktree_node *child)
{
	ktree_add_child_after(parent, child, NULL);
}

static inline void ktree_add_child_last(struct ktree_node *parent,
					struct ktree_node *child)
{
	ktree_add_child_before(parent, child, NULL);
}

static inline void ktree_add_sibling_after(struct ktree_node *child,
					   struct ktree_node *pos)
{
	ktree_add_child_after(pos->parent, child, pos);
}

static inline void ktree_add_sibling_before(struct ktree_node *child,
					    struct ktree_node *pos)
{
	ktree_add_child_before(pos->parent, child, pos);
}

void ktree_del_node(struct ktree_node *node);
void ktree_remove_node(struct ktree_node *node);

static inline void ktree_del_tree(struct ktree *ktree)
{
	if (ktree->root)
		ktree_del_node(ktree->root);
}

static inline void ktree_remove_tree(struct ktree *ktree)
{
	if (ktree->root)
		ktree_remove_node(ktree->root);
}

static inline bool ktree_is_leaf(struct ktree_node *node)
{
	return list_empty(&node->children);
}

int ktree_for_each_child(struct ktree_node *parent,
			 int (*visitor)(struct ktree_node *child, void *arg),
			 void *arg);

void ktree_sort_children(struct ktree_node *parent,
			 int (*cmp)(struct ktree_node *a, struct ktree_node *b,
				    void *arg),
			 void *arg);

#endif
