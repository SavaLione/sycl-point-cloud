# sycl-point-cloud
This project is a part of a scientific paper I've written.

The software implements an $O(1)$ dynamic task routing mechanism for 3D point cloud radius search operations.
By evaluating structural resolution and normalized spatial entropy, the system selects optimal execution hardware to minimize thread divergence, synchronization delays, and data transfer overhead across non-uniform datasets.

The source code in this repository should work.
I'm currently polishing and documenting everything.

## Licenses and Acknowledgements
This project is licensed under [The GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.en.html).
See the [LICENSE](LICENSE) file for the full license text.

Copyright (C) 2026 Savelii Pototskii (savalione.com)

### Third-Party Libraries
This project incorporates code from several third-party libraries.
I am grateful to their developers and maintainers.
The full license texts for these libraries can be found in the `/licenses` directory.
* xgetopt - [zlib License](licenses/LICENSE-xgetopt.txt)
