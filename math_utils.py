from typing import Iterable

def mean(values: Iterable[float]) -> float:
    """Return the arithmetic mean of `values`.

    Parameters
    ----------
    values: Iterable[float]
        Numbers to average. Raises `ValueError` if empty.
    """
    values = list(values)
    if not values:
        raise ValueError("mean() arg is an empty sequence")
    return sum(values) / len(values)
