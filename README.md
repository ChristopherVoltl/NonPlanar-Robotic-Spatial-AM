# NonPlanar Robotic Spatial AM

Rhino 8 and Grasshopper plugin for robotic, nonplanar, and spatial additive manufacturing research in C#.

This project focuses on slicing a `Brep` into curved or topology-driven layers for robotic 3D printing workflows, then preparing those paths for downstream machine logic such as SMT. It currently contains two experimental slicing pipelines:

- a topology-driven nonplanar slicer for section-based layer generation
- an S3-inspired slicer based on an inner orientation-field solve and an outer deformation/scalar-field solve

## Table of Contents

- [Why This Project Exists](#why-this-project-exists)
- [Current Features](#current-features)
- [Project Status](#project-status)
- [Tech Stack](#tech-stack)
- [Repository Structure](#repository-structure)
- [Requirements](#requirements)
- [Installation and Build](#installation-and-build)
- [Grasshopper Components](#grasshopper-components)
- [Workflow Overview](#workflow-overview)
- [Research Background](#research-background)
- [Known Limitations](#known-limitations)
- [Roadmap](#roadmap)
- [Contributing](#contributing)
- [License](#license)

## Why This Project Exists

Conventional planar slicing is limiting for robotic extrusion on complex geometry. This plugin is aimed at workflows where:

- the source model is a `Brep`
- slice behavior should react to topology, curvature, or a derived field
- print layers need to be analyzed before machine export
- slicing and machine translation should stay modular

The goal is not only to produce curves, but to produce curves that are meaningful for robotic fabrication, visual debugging, and future toolpath planning.

## Current Features

- Brep-to-mesh analysis workflow inside Rhino/Grasshopper
- topology-driven nonplanar slicing with scalar-field-based extraction
- field diagnostics including mesh previews, field vectors, and histograms
- curve smoothing with surface-follow tolerance
- output resampling by point count or minimum point distance
- feature-aware point placement that prioritizes corners and sharp changes
- representative slice-plane generation for each extracted curve
- SMT export component for translating sliced curves into SMT point data
- separate experimental S3-inspired slicing pipeline

## Project Status

This is an active research and development project, not a finished production slicer.

Some parts of the plugin are stable enough for iterative testing in Grasshopper, while others are still experimental. In particular:

- the topology-driven slicer is the more practical day-to-day workflow right now
- the S3-inspired slicer is an approximation of the paper structure, not a full reproduction of the original optimizer
- SMT export depends on local SMT DLL availability and the expected SMT environment

## Tech Stack

- C#
- Rhino 8
- Grasshopper
- .NET `net8.0-windows`
- .NET Framework `net48`
- RhinoCommon / Grasshopper SDK
- optional SMT integration for export

## Repository Structure

```text
.
|-- Core/
|   |-- geometry primitives
|   |-- slicer option models
|   |-- topology-driven slicer
|   `-- S3-inspired slicer
|-- Integration/
|   |-- Grasshopper components
|   |-- Rhino Brep/mesh interop
|   |-- SMT export bridge
|   `-- SMT path mapping utilities
|-- Properties/
|-- NonPlanar Robotic Spatial AM.csproj
|-- NonPlanar Robotic Spatial AM.sln
`-- README.md
```

## Requirements

Before building, make sure you have:

- Windows
- Visual Studio 2022 or newer
- .NET SDK 8.x
- Rhino 8 installed
- Grasshopper available through Rhino 8

Optional:

- SMT DLLs if you want to enable the SMT export path

## Installation and Build

### 1. Clone the repository

```powershell
git clone <your-repo-url>
cd "NonPlanar Robotic Spatial AM"
```

### 2. Open the solution

Open:

```text
NonPlanar Robotic Spatial AM.sln
```

### 3. Restore and build

```powershell
dotnet build ".\NonPlanar Robotic Spatial AM.csproj"
```

Build outputs are generated as `.gha` assemblies under:

```text
bin\Debug\net48\
bin\Debug\net8.0-windows\
```

### 4. Install into Grasshopper

Copy the desired `.gha` output into your Grasshopper Libraries folder, then restart Rhino/Grasshopper.

If Rhino is open during build, the output `.gha` can be locked and prevent copying. In that case, close Rhino before rebuilding.

### 5. Optional SMT setup

For SMT export, make sure the SMT assemblies are available locally. The project is set up to compile the SMT path when the required DLLs are present.

Expected local setup:

- SMT references available under `libs\SMT\` or your existing local SMT path
- Rhino 8 environment configured to load the SMT plugin

## Grasshopper Components

### `NonPlanar Slice`

Main Brep slicer for topology-driven nonplanar layers.

Inputs include:

- Brep
- field mode
- angle influence
- smoothing
- layer height
- adaptive blend
- saddle bias
- curve smoothing
- sampling mode
- point count or minimum point distance
- surface tolerance

Outputs include:

- sliced curves
- representative slice planes
- analysis text
- angle map mesh
- field vectors
- angle histogram

### `S3 Slice`

Experimental S3-inspired slicer based on:

- inner orientation-field smoothing
- outer deformation and scalar-field update

Inputs include:

- Brep
- layer height
- inner and outer iterations
- field smoothness
- deformation strength
- support, strength, and quality weights
- curve smoothing
- sampling controls
- surface tolerance

Outputs include:

- sliced curves
- representative planes
- field map mesh
- field vectors
- histogram
- analysis text

### `SMT Export`

Takes sliced curves and translates them into SMT point data.

Inputs include:

- path curves
- vertical, angled, and horizontal E5 values
- velocity ratio multiplier
- run toggle

Outputs include:

- generated SMT planes
- export log message

## Workflow Overview

Typical workflow in Grasshopper:

1. Reference a `Brep`.
2. Run either `NonPlanar Slice` or `S3 Slice`.
3. Inspect the output curves, planes, and field diagnostics.
4. Tune smoothing, sampling, and field controls.
5. Send validated curves into `SMT Export` when machine-side translation is needed.

This separation is intentional. It makes it easier to debug geometry before committing to machine output.

## Research Background

The current codebase is informed by multiple nonplanar and curved-layer slicing ideas.

The two most important research directions currently represented are:

- topology-aware nonplanar slicing for complex surfaces
- S3-style deformation-based slicing with orientation-field optimization

Relevant references:

- [SIGGRAPH Asia 2022 S3 paper](https://dl.acm.org/doi/10.1145/3550454.3555516)
- [S3_DeformFDM reference repository](https://github.com/zhangty019/S3_DeformFDM)

Important note:

The S3 implementation in this repository is inspired by the paper structure and public reference code, but it is still an approximation adapted for Rhino/Grasshopper and this research plugin. It should not be treated as a full reproduction of the original method.

## Known Limitations

- some slicer behaviors are still experimental and may require manual tuning
- the S3-inspired workflow is not yet a full research-faithful implementation
- Rhino/Grasshopper plugin file locking can interrupt local build-copy-test loops
- SMT export depends on local DLL setup and expected SMT process naming
- the license file currently contains placeholder author and year values and should be finalized before public release

## Roadmap

- strengthen the relationship between field diagnostics and actual layer deformation
- continue improving the S3-inspired solver so deformation follows the field more directly
- add richer debugging and visualization outputs in Grasshopper
- improve SMT export robustness and setup documentation
- add example Grasshopper definitions
- add screenshots and benchmark models to the repository

## Contributing

This repository is currently structured around research-driven iteration, so contributions are easiest when they are:

- focused
- reproducible
- tied to a specific slicing, export, or visualization issue

If you contribute, please include:

- a short explanation of the problem
- the expected behavior
- the actual behavior
- a sample Brep or Grasshopper definition when possible

## License

This repository includes an MIT license template in [LICENSE.txt](./LICENSE.txt).

Before publishing or distributing broadly, update the placeholder copyright fields with the correct year and owner.

## Acknowledgments

This project builds on ideas from nonplanar additive manufacturing research, Rhino/Grasshopper plugin development practices, and SMT-based robotic printing workflows.
