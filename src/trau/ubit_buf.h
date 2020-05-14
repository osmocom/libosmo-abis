#pragma once
#include <osmocom/core/bits.h>

/* Small helper inspired by msgb */

struct ubit_buf {
	const ubit_t *buf;	/*!< start of underlying buffer */
	const ubit_t *data;	/*!< next to be consumed bit */
	size_t n_bits;		/*!< number of total bits iin buffer */
};

/*! length of [remainig, to be processed] data in ubit_buf */
static inline size_t ubb_length(struct ubit_buf *ubb)
{
	return ubb->n_bits - (ubb->data - ubb->buf);
}

/*! retrieve + remove a single ubit_t from start of ubit_buf */
static inline ubit_t ubb_pull_ubit(struct ubit_buf *ubb)
{
	OSMO_ASSERT(ubb->data < ubb->buf + ubb->n_bits);
	return *ubb->data++;
}

static inline void ubb_pull(struct ubit_buf *ubb, size_t count)
{
	OSMO_ASSERT(ubb_length(ubb) >= count);
	ubb->data += count;
}

/*! get pointer to next to be consumed bit */
static inline const ubit_t *ubb_data(struct ubit_buf *ubb)
{
	return ubb->data;
}

static inline void ubb_init(struct ubit_buf *ubb, const ubit_t *bits, size_t n_bits)
{
	ubb->buf = bits;
	ubb->data = ubb->buf;
	ubb->n_bits = n_bits;
}
