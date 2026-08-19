# sycl-point-cloud
This project is a part of a scientific paper I've written.

The software implements an $O(1)$ dynamic task routing mechanism for 3D point cloud radius search operations.
By evaluating structural resolution and normalized spatial entropy, the system selects optimal execution hardware to minimize thread divergence, synchronization delays, and data transfer overhead across non-uniform datasets.

The source code in this repository should work.
I'm currently polishing and documenting everything.

## Building
### Dependencies and environmental variables
To build and run this project, ensure the following dependencies are installed and properly configured:
* CMake (>= 3.28)
* Ninja (recommended build system)
* SYCL compiler
    * AdaptiveCpp is used in this project
    * AdaptiveCpp requires LLVM / Clang (version 21 recommended, version 22 is not fully supported yet by the AdaptiveCpp compiler version 21)

Depending on your LLVM and SYCL compiler installation paths, you may need to export the following environmental variables before building:
```bash
export CC=/usr/bin/clang-21
export CXX=/usr/bin/clang++-21
export LDFLAGS="-fuse-ld=lld"
export AR=/usr/bin/llvm-ar-21
export RANLIB=/usr/bin/llvm-ranlib-21
export CXXFLAGS="-stdlib=libstdc++"
```

### Building
1. Clone the repository: `git clone https://github.com/SavaLione/sycl-point-cloud`
2. Navigate to the project directory: `cd sycl-point-cloud`
3. Create and enter a build directory: `mkdir build && cd build`
4. Configure the project with CMake: `cmake -GNinja -DCMAKE_INSTALL_PREFIX=/root/adaptivecpp ..`
    * Where:
        * `/root/adaptivecpp` is the AdaptiveCpp compiler location on your system
        * `-GNinja` use the Ninja build system (you can omit it in order to use the default one)
5. Compile the project: `ninja` or `make`

## Usage
Run the compiled executable:
```sh
./sycl-point-cloud
```

Enumerate and display all available SYCL platforms and devices:
```sh
./sycl-point-cloud --device-list
```

Run the application with the search radius for neighborhood queries of 1.5 and the spatial hashing cell dimension of 0.75:
```sh
./sycl-point-cloud -c dataset.bin -r 1.5 -s 0.75
```

Run the application with search queries `10.0,5.0,-2.5` and `0.0,0.0,0.0`, and number of query points processed per batch of 1000:
```sh
./sycl-point-cloud -c dataset.bin -q 10.0,5.0,-2.5,0.0,0.0,0.0 -b 1000
```

Perform spatial analysis and output the execution target prediction without executing the radius search:
```sh
./sycl-point-cloud -c dataset.bin --dispatcher-prediction
```

Run all benchmarks (it may take some time):
```sh
./sycl-point-cloud-benchmarks
```

Run a specific benchmark:
```sh
./sycl-point-cloud-benchmarks --benchmark_filter=bm_adaptive_dispatcher
```

Run two specific benchmarks:
```sh
./sycl-point-cloud-benchmarks --benchmark_filter="bm_adaptive_dispatcher|bm_cpu_compact_spatial_hashing"
```

Specify point cloud and run specific benchmark:
```sh
BENCHMARK_POINT_CLOUD=sg27_station3_intensity_rgb.bin ./sycl-point-cloud-benchmarks --benchmark_filter=bm_adaptive_dispatcher_file
```

Convert an xyz intensity rgb type point cloud to the binary representation:
```sh
./sycl-point-cloud-convert --file-source in.txt --file-destination out.bin --source-type xyz_intensity_rgb --destination-type binary
```

## Licenses and Acknowledgements
This project is licensed under [The GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.en.html).
See the [LICENSE](LICENSE) file for the full license text.

Copyright (C) 2026 Savelii Pototskii (savalione.com)

### Third-Party Libraries
This project incorporates code from several third-party libraries.
I am grateful to their developers and maintainers.
The full license texts for these libraries can be found in the `/licenses` directory.
* xgetopt - [zlib License](licenses/LICENSE-xgetopt.txt)
