# Error Suppression Demo

This repository demonstrates two simple models of numerical error growth:

1. **Vector error accumulation**, where error is treated as a vector and its norm
   can decrease due to cancellation.
2. **Scalar error accumulation**, where error is treated as a non-canceling
   magnitude budget.

A simple control mechanism removes a fixed amount of error per iteration
to illustrate **error suppression**, not chaos suppression.

The underlying dynamics are not modified.

License: MIT