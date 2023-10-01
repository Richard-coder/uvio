# UVIO: UWB-Aided Visual-Inertial Odometry Framework

This is the code of the [academic paper] An UWB-Aided Visual-Inertial Odometry Framework with
Bias-Compensated Anchors Initialization, a multi-sensor framework that leverages 
Ultra Wideband (UWB) technology and Visual-Inertial Odometry (VIO) to provide robust 
and low-drift localization. The whole project is built on top of OpenVINS and enhances it 
with UWB capabilities. 

## Usage

To use UVIO, you can refer to the OpenVINS documentation for everything except the UWB-related features. It requires
UWB range measurements to be available to the system (for the paper we used these [sensors](https://www.qorvo.com/products/p/MDEK1001)) 
and additional configuration files that are located inside /config/iros_2023_uvio folder.

The framework requires the position of UWB anchors to operate. You have two options for providing this information:

1. UWB Anchors Config File (uwb_anchors.yaml): you can specify the positions of UWB anchors in the global reference frame in the 
uwb_anchors.yaml configuration file. This is a straightforward way to configure anchor positions.

2. Initialization as explained in the [academic paper]: alternatively, you can initialize unknown UWB anchors with the 
following [initialization library](https://github.com/aau-cns/uwb_init).

## Credit / Licensing

The UVIO module as an extension of OpenVins code was written by the [Control of Networked System (CNS)](https://www.aau.at/en/smart-systems-technologies/control-of-networked-systems/) at the University of Klagenfurt (Austria). If you use this software in an academic research setting, please cite the corresponding [academic paper].

```latex
@inproceedings{Delama2023,
   author       = {Delama, Giulio and Shamsfakhr, Farhad and Weiss, Stephan and Fontanelli, Daniele and Fornasier, Alessandro},
   booktitle    = {2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
   title        = {UVIO: An UWB-Aided Visual-Inertial Odometry Framework with Bias-Compensated Anchors Initialization},
   year         = {2023},
  organization  = {IEEE}
}
```

The codebase and documentation is licensed under the [GNU General Public License v3 (GPL-3)](https://www.gnu.org/licenses/gpl-3.0.txt).
You must preserve the copyright and license notices in your derivative work and make available the complete source code with modifications under the same license ([see this](https://choosealicense.com/licenses/gpl-3.0/); this is not legal advice).

## Reporting Issues

In case of issues, feature requests, or other questions please open a [New Issue](https://gitlab.aau.at/aau-cns/uvio/issues/new?issue) or contact the authors via email.

## Authors

* Giulio Delama ([email](mailto:giulio.delama@aau.at?subject=[UWB%20Init]))
* Alessandro Fornasier ([email](mailto:alessandro.fornasier@ieee.org?subject=[UWB%20Init]))

<!-- LINKS: -->
[academic paper]: https://arxiv.org/abs/2308.00513

