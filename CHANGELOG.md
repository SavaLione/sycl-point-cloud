# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
This project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]
### Added
- Added changelog
- Added benchmarks for real dataset testing
- Added new sections in the readme
- Added a tool for converting datasets
- Added new conversions to the converter
- Added documentation
- Added help information about the application (`-h`, `--help` flags)
- Added usage examples to the readme
- Added a benchmark for the spatial density probe algorithm across varying grid resolutions and sample sizes
- Added common arguments for benchmarks
- Added new metrics to some benchmarks
- Added new metrics and documentation the compact spatial hashing (CPU) benchmark
- Added new metrics and documentation the SYCL compact spatial hashing benchmark

### Fixed
- Fixed some minor typos
- Fixed the project description in the CMake configuration file
- Fixed a typo in `convert/sycl-point-cloud-convert.cpp`
- Fixed entropy values mentioned in the dataset generator

### Changed
- Suppressed warnings from AdaptiveCpp/Clang (`warning: argument unused during compilation: '-c'`)
- Reimplemented the probe density evaluation algorithm
- Changed the name of the probe density evaluation algorithm function
- Changed probe density default sample size from 256 to 4096
- Slightly changed behavior of some benchmarks
