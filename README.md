# Single Window Separable Convolution Accelerator

## Contributors

TBD

## Background

RTL implementation of a pipelined single window separable convolution.
Computes a 2D convolution by passing a 1x7 kernel across the input image
row-wise and column wise.
The dot-product used to compute each kernel step is optimized using
distributed arithmetic and pipelining to minimize the critical path.

Based on [this paper](https://ieeexplore.ieee.org/document/8325332)

## Running

Running the formal and simulation test suites requires installing the [Yosys OSS Cad Suite](https://github.com/YosysHQ/oss-cad-suite-build).
After installation, you can run tests with the following commands:

```bash
# runs all tests in the formal suite
make formal

# runs a specific formal verification instance
make formal/\<fv_name\>

# runs all tests in the simulation suite with Verilator (very fast runtime, slow build, less accurate)
make tests

# run a specific sim with Verilator
make tests/\<tb_name\>

# runs all tests in the simulation suite with Icarus (slow runtime, fast build, most accurate)
make itests

# run a specfic sim with Icarus
make itests/\<tb_name\>
```
