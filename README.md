<div align="center">
    <h1>KISS-Matcher</h1>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
    <a href="https://github.com/MIT-SPARK/KISS-Matcher"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://arxiv.org/abs/2409.15615"><img src="https://img.shields.io/badge/arXiv-b33737?logo=arXiv" /></a>
    <br />
    <br />
  <br />
  <br />
  <p align="center"><img src="https://github.com/user-attachments/assets/763bafef-c11a-4412-a9f7-f138fc12ff9f" alt="KISS Matcher" width="95%"/></p>
  <p><strong><em>Keep it simple, make it scalable.</em></strong></p>
</div>

______________________________________________________________________

## :package: Installation

> All installations are set up automatically in an out-of-the-box manner.

Run the command below to install basic dependency packages:

```commandline
git clone https://github.com/MIT-SPARK/KISS-Matcher.git
cd KISS-Matcher
make deps
```

`make deps` auto-detects the host OS:

- **Linux (Ubuntu/Debian):** installs the toolchain via `apt-get`.
- **macOS (Apple Silicon / Intel):** installs the toolchain via [Homebrew](https://brew.sh).
  AppleClang does not ship with OpenMP, so we pull in `llvm` + `libomp`
  from Homebrew and the `Makefile` automatically routes the C++ build through
  `$(brew --prefix llvm)/bin/clang++`. If you have not installed Homebrew yet,
  install it first from https://brew.sh.

After that, follow the C++ or Python installation instructions below.

---

### C++

Run the command below. That's it.

```commandline
git clone https://github.com/MIT-SPARK/KISS-Matcher.git
cd KISS-Matcher
make cppinstall
```

> \[!WARNING\]
> If you encounter the following error:
>
> ```
> CMake Error: Not a file: ${your_directory}/KISS-Matcher/cpp/kiss_matcher/build/_deps/robin-build/cmake_install.cmake
> CMake Error: Error processing file: ${your_directory}/KISS-Matcher/cpp/kiss_matcher/build/_deps/robin-build/cmake_install.cmake
> make: *** [Makefile:42: cppinstall] Error 1
> ```
>
> it means that the [ROBIN](https://github.com/MIT-SPARK/ROBIN) package is already installed in your environment.
> In that case, simply run the following command:
>
> ```
> make cppinstall_matcher_only
> ```

<details>
  <summary><strong>Q. How doest it work?</a></strong></summary>

The `cppinstall` command is encapsulated in the [Makefile](https://github.com/MIT-SPARK/KISS-Matcher/blob/main/Makefile). and `cppinstall` calls `deps` to automatically install the dependencies.
In addition, KISS-Matcher requires [ROBIN](https://github.com/MIT-SPARK/ROBIN).
But it's also automatically linked via [robin.cmake](https://github.com/MIT-SPARK/KISS-Matcher/blob/main/cpp/kiss_matcher/3rdparty/robin/robin.cmake)

After installation, `kiss_matcher` and `robin` are placed in the installation directory using the following commands in the `Makefile`, respectively:
`	@$(SUDO) cmake --install cpp/kiss_matcher/build 	@$(SUDO) cmake --install cpp/kiss_matcher/build/_deps/robin-build    `

</details>

<details>
  <summary><strong>Q. Building from a non-Makefile flow on macOS?</strong></summary>

If you invoke `cmake` directly on macOS instead of going through `make
cppinstall`, point CMake at the Homebrew LLVM toolchain so OpenMP is available:

```bash
LLVM_PREFIX=$(brew --prefix llvm)
cmake -S cpp/kiss_matcher -B build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_C_COMPILER="$LLVM_PREFIX/bin/clang" \
      -DCMAKE_CXX_COMPILER="$LLVM_PREFIX/bin/clang++" \
      -DUSE_SYSTEM_ROBIN=OFF
cmake --build build -j$(sysctl -n hw.ncpu)
```

`USE_SYSTEM_ROBIN=OFF` forces a fresh ROBIN via `FetchContent` and avoids
picking up an older / incompatible system install. On Linux this flag is not
required, but it is harmless.

</details>

#### Example codes

We provide plentiful scalable registration examples. Please visit our [**cpp/example**](https://github.com/MIT-SPARK/KISS-Matcher/tree/main/cpp/examples) directory and follow the instructions.

#### How To Use in Other Packages in C++?

See [CMakeLists.txt](https://github.com/MIT-SPARK/KISS-Matcher/blob/main/cpp/examples/CMakeLists.txt) in `cpp/examples`.

______________________________________________________________________

### Python

The prerequisites for Pybind11 are just the minimum requirements as follows:"

```
pip3 install --upgrade pip setuptools wheel scikit-build-core ninja cmake build
```

And then, run the following command:

```
pip3 install -e python/
```

We also provide out-of-the-box python registration examples. Go to [**python**](https://github.com/MIT-SPARK/KISS-Matcher/tree/main/python) directory and follow the instructions.

______________________________________________________________________

## Citation

This study is the culmination of my continuous research since my graduate school years.
If you use this library for any academic work, please cite our original [paper](https://arxiv.org/abs/2409.15615), and refer to other papers that are highly relevant to KISS-Matcher.

<details>
  <summary><strong>See bibtex lists</a></strong></summary>

```bibtex
 @inproceedings{lim2025icra-KISSMatcher,
   title={{KISS-Matcher: Fast and Robust Point Cloud Registration Revisited}},
   author={Lim, Hyungtae and Kim, Daebeom and Shin, Gunhee and Shi, Jingnan and Vizzo, Ignacio and Myung, Hyun and Park, Jaesik and Carlone, Luca},
   booktitle={Proc. IEEE Int. Conf. Robot. Automat.},
   year={2025},
   codeurl   = {https://github.com/MIT-SPARK/KISS-Matcher},
   note   = {Accepted. To appear}
 }
```

```bibtex
 @article{Lim24ijrr-Quatropp,
   title={{Quatro++: R}obust global registration exploiting ground segmentation for loop closing in {LiDAR SLAM}},
   author={Lim, Hyungtae and Kim, Beomsoo and Kim, Daebeom and Mason Lee, Eungchang and Myung, Hyun},
   journal={Int. J. Robot. Res.},
   pages={685--715},
   year={2024},
   doi={10.1177/02783649231207654}
 }
```

```bibtex
@inproceedings{Lim22icra-Quatro,
    title={A single correspondence is enough: Robust global registration to avoid degeneracy in urban environments},
    author={Lim, Hyungtae and Yeon, Suyong and Ryu, Soohyun and Lee, Yonghan and Kim, Youngji and Yun, Jaeseong and Jung, Euigon and Lee, Donghwan and Myung, Hyun},
    booktitle={Proc. IEEE Int. Conf. Robot. Automat.},
    pages={8010--8017},
    year={2022}
}
```

```bibtex
@InProceedings{Shi21icra-robin,
    title={{ROBIN:} a Graph-Theoretic Approach to Reject Outliers in Robust Estimation using Invariants},
    author={J. Shi and H. Yang and L. Carlone},
    booktitle={Proc. IEEE Int. Conf. Robot. Automat.},
    note = {arXiv preprint: 2011.03659, \linkToPdf{https://arxiv.org/pdf/2011.03659.pdf}},
    pdf="https://arxiv.org/pdf/2011.03659.pdf",
    year={2021}
}
```

```bibtex
@article{Yang20tro-teaser,
    title={{TEASER: Fast and Certifiable Point Cloud Registration}},
    author={H. Yang and J. Shi and L. Carlone},
    journal={IEEE Trans. Robot.},
    volume = 37,
    number = 2,
    pages = {314--333},
    note = {extended arXiv version 2001.07715 \linkToPdf{https://arxiv.org/pdf/2001.07715.pdf}},
    pdf={https://arxiv.org/pdf/2001.07715.pdf},
    Year = {2020}
}
```

```bibtex
@inproceedings{Zhou16eccv-FGR,
    title={Fast global registration},
    fullauthor={Zhou, Qian-Yi and Park, Jaesik and Koltun, Vladlen},
    author={Q.Y. Zhou and J. Park and V. Koltun},
    booktitle={Proc. Eur. Conf. Comput. Vis.},
    pages={766--782},
    year={2016}
}
```

</details>

## Contributing

Like [KISS-ICP](https://github.com/PRBonn/kiss-icp),
we envision KISS-Matcher as a community-driven project, we love to see how the project is growing thanks to the contributions from the community. We would love to see your face in the list below, just open a Pull Request!

<a href="https://github.com/MIT-SPARK/KISS-Matcher/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=MIT-SPARK/KISS-Matcher" />
</a>

## Acknowledgements

This project was supported by the NRF grant funded by the government of South Korea, Ministry of Science and ICT (MSIT) (No. RS-2024-00461409).

We sincerely express our gratitude to [Prof. Cyrill Stachniss’ group](https://www.ipb.uni-bonn.de/index.html) at the University of Bonn for their generosity in allowing us to use the term *KISS*.
In particular, I (Hyungtae Lim) personally deeply appreciate [Tizziano Guadagnino](https://scholar.google.com/citations?user=5m73YFQAAAAJ&hl=it), [Benedikt Mersch](https://scholar.google.com/citations?user=XwuAB1sAAAAJ&hl=en), [Louis Wiesmann](https://scholar.google.de/citations?user=EEyCOpIAAAAJ&hl=de), and [Jens Behley](https://jbehley.github.io/) for their kind support and collaboration during my time at the University of Bonn.
Without them, KISS-Matcher would not exist today. A special thanks goes to my co-author, [Nacho](https://github.com/nachovizzo), for taking the time to thoroughly teach me modern C++ and modern CMake.
We would also like to express our gratitude to [Kenji Koide](https://staff.aist.go.jp/k.koide/) for his exceptional effort in open-sourcing the wonderful open source, [small_gicp](https://github.com/koide3/small_gicp), which plays a crucial role in enhancing the efficiency of our pipeline.
His dedication to the research community continues to inspire and enable advancements in the field.

Again, their generosity, expertise, and contributions have greatly enriched our work, and we are truly grateful for their support.
