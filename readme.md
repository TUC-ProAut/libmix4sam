# Mixtures for Smoothing and Mapping Library (libMix4SAM)

<img align="right" width="300px" src="doc/img/promo.gif">
<img align="right" height="300px" src="doc/img/promo_padding.png">

libMix4SAM is extending the functionality of GTSAM [1].
It provides factors and noise models to implement robust state estimation using Gaussian mixture distributions.

Further information can be found in the corresponding paper, see section "Citation", and our [project's landing page](https://mytuc.org/mix).

## Contact information
- [Sven Lange](https://www.tu-chemnitz.de/etit/proaut/sven_lange)
- [Tim Pfeifer](https://www.tu-chemnitz.de/etit/proaut/tim_pfeifer)
<br clear="right"/>

## License information
libmix4sam is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

libmix4sam is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this software.  If not, see <http://www.gnu.org/licenses/>.

## Citation

If you use this library for academic work, please either cite the library or a corresponding paper:
<details>
<summary>BibTeX of libmix4sam</summary>

```tex
  @Misc{libmix4sam,
   author       = {Sven Lange and Others},
   title        = {libmix4sam},
   howpublished = {\url{https://github.com/TUC-ProAut/libmix4sam}}
  }
```

</details>

<details>
<summary>Accompanying Paper</summary>

* Pfeifer, T., Lange, S. and Protzel, P. (2021) [Advancing Mixture Models for Least Squares Optimization](https://doi.org/10.1109/LRA.2021.3067307), IEEE Robotics and Automation Letters, 6(2), pp. 3941--3948.
* [arXiv-Version](http://arxiv.org/abs/2103.02472)


</details>

## How to compile
The following two subsections describe how to build libmix4sam in combination with a local build of GTSAM. As there are some specifics to the GTSAM build, we will start with this. *(If you want to use the Python and/or Matlab wrapper, I recommend using a Conda environment, see further below for details.)*

<details>
<summary>Compile GTSAM locally for libmix4sam</summary>

First, download a compatible release of GTSAM, e.g. 4.2a9:
```bash
❯ git clone --single-branch --branch 4.2a9 https://github.com/borglab/gtsam.git gtsam-42a9-src
```
Then configure and compile:
```bash
❯ mkdir gtsam-42a9-build && cd gtsam-42a9-build
❯ cmake -C <libmix4sam-src>/gtsamConfig.cmake -S ../gtsam-42a9-src 
❯ make -j2
```
> **Note:** We need some specific options for compiling GTSAM. They are invoked through using the `gtsamConfig.cmake` as pre-load option for cmake. Further explanation can be found directly within the `*.cmake` file.

> **Note2:** For easy use later on, we recommend to stick to the naming convention used above for source and build folder of GTSAM.
</details>

<details open>
<summary>Compile libmix4sam</summary>
Change to the source folder of libmix4sam.

```bash
❯ mkdir build && cd build
❯ cmake .. -DGTSAM_BUILD_DIR=<path-to-gtsam-42a9-build-directory>
❯ make -j2
```

</details>

## How to compile using Conda
Especially, when using the Python wrapper, we may want to develop using a different environment (e.g. with the newest machine learning library, which is not supported by the system's Python version anymore), so we can use Conda to set up a new environment.

### Create Conda environment
```bash
conda create --name libmix4sam python=3.10 jupyter cmake matplotlib
conda activate libmix4sam
pip install numpy pyparsing pytest
```

### Build and Install GtSAM

Go into the workspace folder of your choice to execute the following steps. You may want to additionally change the installation directory (CMAKE_INSTALL_PREFIX) of GTSAM during cmake configuration to be alongside the `gtsam-<version>-src` and `gtsam-<version>-build` folders.

```bash
❯ git clone --single-branch --branch 4.2a9 https://github.com/borglab/gtsam.git gtsam-42a9-src
❯ mkdir gtsam-42a9-build && cd gtsam-42a9-build
❯ cmake -C <libmix4sam-src>/gtsamConfig.cmake -DCMAKE_INSTALL_PREFIX=$(pwd)/../gtsam-42a9-install -S ../gtsam-42a9-src 
❯ make -j$(getconf _NPROCESSORS_ONLN) install
❯ make python-install
```

While building GTSAM and also libmix4sam, attention should be lied on the Python version. This should be the one from the Conda environment! 


### Build and Install libmix4sam

Now, we can **compile libmix4sam** using the proper GTSAM path configuration (GTSAM_BUILD_DIR and GTSAM_INSTALL_DIR) and Python environment. Here is an example assuming the libmix4sam and GTSAM source folders have a common parent directory.

```bash
❯ mkdir build && cd build
❯ cmake -DGTSAM_BUILD_DIR=$(pwd)/../../gtsam-42a9-build -DGTSAM_INSTALL_DIR=$(pwd)/../../gtsam-42a9-install
❯ make -j$(getconf _NPROCESSORS_ONLN)
❯ make python-install
```


## Example
The file `examples/testPsr2DFactor.cpp` implements a very simple 2D registration example using a GMM for the fixed point set.

The same example is also implemented in MATLAB, see the folder `matlab/libmix4sam/tests`.
For more advanced examples, we refer to our [CREME Project](https://mytuc.org/creme) (Credible Radar Ego-Motion Estimation).

Likewise, you can find the example implemented in Python and Jupyter notebook using the Python wrapper, see `python/examples/testPsr2DFactor.{py,ipynb}`.

## Matlab 
To use the library in MATLAB, add it to the search path:
```Matlab
❯ path( path, fullfile(<Path-to-lib-src>, 'matlab') );
❯ libmix4sam.configurePath(<Path-to-lib-src>);
```
To test the functionality, there is the same example as above implemented in MATLAB, see `matlab/tests/testPsr2DFactor.m`.

Some functionalities need additional toolboxes or functions. These are usually located in `matlab/External` and can be downloaded by running the included `getExternals.sh` script (for manual download, see contents of the script). 

## Additional notes

> **LM-Algorithm:** 
> In between the GTSAM versions 4.0 and 4.0.3, the stopping criteria calculation of the Levenberg-Marquardt algorithm changed (this regards the file `gtsam/nonlinear/LevenbergMarquardtOptimizer.cpp`). To get the same results as in our published experiments, the mentioned file has to be replaced by the version from gtsam release 4.0.

> **Matlab-Wrapper:** If MATLAB is complaining about 'Missing symbol's while loading the wrapper, it usually helps starting Matlab with changed `LD_PRELOAD` environment variable as follows:
>```bash
>❯ LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libstdc++.so.6" matlab
>```

> **Error-Modeling:** Be careful in modelling the mixture components. E.g. if you model an ambiguous measurement with three possible modes all into the mean for the Gaussian components and use zero for the factor, it may lead to an indeterminate system. If you instead use one mean as measurement and shift the remaining means accordingly, the system may be solvable again.  

## Known bugs
* The new GMM noise model won't work till now with GTSAM's BearingRangeFactor (and RangeFactor / BearingFactor), which are based on the ExpressionFactor. If you need it, you could use the older versions of these factors from GTSAM 3.2.3.

## References

[1] GTSAM 4.0 https://gtsam.org


