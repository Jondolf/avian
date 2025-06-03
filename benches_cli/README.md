# `benches_cli`

A command-line interface for benchmarking Avian.

## Usage

Below are the available option.

```text
Options:
  -n, --name <NAME>                The name or number of the benchmark to run. Leave empty to run all benchmarks
  -t, --threads <THREADS>          A range for which thread counts to run the benchmarks with. Can be specified as `start..end` (exclusive), `start..=end` (inclusive), or `start` [default: 1..13]
  -s, --steps <STEPS>              The number of steps to run for each benchmark [default: 300]
  -r, --repeat <REPEAT>            The number of times to repeat each benchmark. The results will be averaged over these repetitions [default: 5]
  -l, --list                       List all available benchmarks in a numbered list
  -o, --output <OUTPUT>            The output directory where results are written in CSV format. Leave empty to disable output
  -h, --help                       Print help
  -V, --version                    Print version
```

An example of running the "Large Pyramid 2D" benchmark with 1-6 threads, 500 steps, repeated 5 times:

```shell
# Running in release mode is heavily encouraged!
cargo run --release --bin benches2d -- --name "Large Pyramid 2D" --threads 1..=6 --steps 500 --repeat 5
```

The output might look something like this:

```text
Running benchmark 'Large Pyramid 2D' with threads ranging from 1 to 6:
| threads | avg time / step | min time / step |
| ------- | --------------- | --------------- |
|       1 |      4.89532 ms |      4.33950 ms |
|       2 |      4.04826 ms |      3.53192 ms |
|       3 |      3.68756 ms |      3.19322 ms |
|       4 |      3.53765 ms |      3.03870 ms |
|       5 |      3.42883 ms |      2.90144 ms |
|       6 |      3.39412 ms |      2.87670 ms |
```
