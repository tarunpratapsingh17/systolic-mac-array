"""
Python reference model and packing helpers shared by all testbenches.

Packing convention (matches the RTL): element i of a row lives in bits
[i*width +: width] of the bus.
"""
import random


def wrap_signed(value, width):
    """Two's-complement wrap of a Python int to `width` bits."""
    mask = (1 << width) - 1
    value &= mask
    return value - (1 << width) if value >> (width - 1) else value


def pack(values, width):
    word = 0
    for i, v in enumerate(values):
        word |= (v & ((1 << width) - 1)) << (i * width)
    return word


def unpack(word, width, count):
    return [wrap_signed(word >> (i * width), width) for i in range(count)]


def matmul_row(a_row, weights, acc=32):
    """One output row: C[j] = sum_k a[k] * W[k][j], wrapped to the accumulator width."""
    n = len(weights)
    return [wrap_signed(sum(a_row[k] * weights[k][j] for k in range(n)), acc) for j in range(n)]


def rand_int8_row(n, rng=random):
    # bias towards the extremes now and then to hit the corners of the multiplier
    choices = [-128, 127, -1, 0, 1]
    return [rng.choice(choices) if rng.random() < 0.15 else rng.randint(-128, 127) for _ in range(n)]


def rand_int8_matrix(n, rng=random):
    return [rand_int8_row(n, rng) for _ in range(n)]
