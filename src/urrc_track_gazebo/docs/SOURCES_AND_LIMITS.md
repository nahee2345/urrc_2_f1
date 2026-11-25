# Source provenance and remaining fidelity limits

## Plan geometry

- Dataset: https://github.com/bacinger/f1-circuits
- Pinned revision: `394d8fbe70ef2c0b0c8d23ff7bee61fa09606055`
- Individual immutable URLs and SHA-256 hashes: `tracks/raw/SOURCES.json`.
- Upstream license and original attribution: `docs/references/upstream_LICENSE.md` and `upstream_README.md`.
- The upstream project identifies its initial source as Google My Maps and explicitly states that its coordinates are unofficial.
- The generator keeps all original source vertices and their relative metric coordinates. Cubic Hermite interpolation joins them with a periodic tangent. This can deviate between sparse vertices; the measured source-polyline Hausdorff deviation is reported.
- There is no XY length-fitting, axis warping, layout simplification into ovals, or AI-image tracing.
- Six circuits have been selected for single nonintersecting layouts. Monza is mandatory; the other five are Spa, Monaco, COTA, Hungaroring and Baku.
- This is a static reference-layout package, not an automatically updating representation of future circuit resurfacing or corner changes.

## Official layout and lap-length references

The references are listed in `references/official_layout_urls.json`. Official artwork is not redistributed, embedded in worlds, or used as a road texture.

- Monza: https://www.formula1.com/en/racing/2026/italy — 5,793 m
- Spa: https://www.formula1.com/en/racing/2026/belgium — 7,004 m
- Monaco: https://www.formula1.com/en/racing/2026/monaco — 3,337 m
- COTA: https://www.formula1.com/en/racing/2026/united-states — 5,513 m
- Hungaroring: https://www.formula1.com/en/racing/2026/hungary — 4,381 m
- Baku: https://www.formula1.com/en/racing/2026/azerbaijan — 6,003 m

Visual review checks the recognizable loop and corner sequence. It is not a pixel-level or surveyed-boundary certification.
Official illustrated maps are stylized, rotated and often exaggerate road width, so their pixels cannot establish an absolute width error.

## Width profiles: ESTIMATED

The raw GeoJSON has no separate left/right road boundaries. Profiles in YAML are engineering estimates.
The following references support broad width ranges or particular features, not every interpolated width sample:

- Monza circuit operator brochure: https://www.monzanet.it/media/2023/05/2023_Brochure_Experience_Monza_compressed.pdf — 10–12 m.
- Spa event organizer: https://press.europeanlemansseries.com/assets/fileuploads/64/2a/642a9e394534c.pdf — 11–14 m.
- Hungaroring operator guidebook: https://hungaroring.hu/guidebook/ — 10–16 m.
- Baku Turn 8/9: https://www.formula1.com/en/latest/article/circuit-guide-everything-you-need-to-know-about-the-baku-city-circuit.320UGBNQu2ALdgAtax1gRn.320UGBNQu2ALdgAtax1gRn — 7.6 m at the narrow section.
- Monaco's narrower road and COTA's wider start/T1 area are approximations rather than extracted metric boundaries.

Road offset folds inside tight corners are resolved as planar polygon topology before mesh export. This avoids intersecting faces and walls.
Any such local trimming is reported and is another reason not to claim surveyed boundary equality.
The finished road is one polygon with exactly one hole. Walls are offset into the non-driving side of each boundary.
Wall footprint and road footprint do not overlap in area; a 1 cm vertical foot overlap prevents a collision seam at road level.

## Elevation: APPROXIMATE

- Provider: OpenTopoData, SRTM 30m terrain dataset: https://www.opentopodata.org/datasets/srtm/
- Cached raw JSON: `tracks/raw/*_dem.json`, 90 samples per circuit, including query locations.
- SRTM is a terrain dataset, not a road engineering survey.
- Broad profiles are circularly smoothed and calibrated to the published full-scale ranges below, then reduced by 0.2.
- Reference: https://www.formula1.com/en/latest/article/highs-and-lows-which-f1-track-has-the-most-elevation-changes-.7I9JEcBw3R2AqXbnJ6hyvc.7I9JEcBw3R2AqXbnJ6hyvc
- Monza 12.8 m; Spa 102.2 m; Monaco 42.0 m; COTA 30.9 m; Hungaroring 34.7 m; Baku 26.8 m.
- These published ranges are historical reference values, not a new 2026 survey. The precise elevation range after smooth interpolation is measured separately.
- No road camber/banking, tunnel-interior road measurement, or high-frequency surface bumps are claimed.

## Grid: APPROXIMATE PLACEMENT

Twenty positions, alternating left/right with 1.6 m successive longitudinal separation (3.2 m on the same side), are painted on the existing main straight.
The box is 1.2 x 0.5 m in the finished world; white line width is 0.025 m. These box dimensions and the exact pole-side/origin are configuration choices, not claimed as FIA survey values.
The structure is F1-style staggered placement, rather than two cars exactly alongside each other.
Grid paint is visual-only and has no collision surface. No vehicle is present.

## Gazebo compatibility target

- SDF 1.9 and OBJ meshes; model URI paths resolve through `GZ_SIM_RESOURCE_PATH`.
- Harmonic / Jazzy for Ubuntu 24.04; Jetty / Lyrical target for Ubuntu 26.04.
- No custom compiled plugins. Source launch scripts use only Python standard library.
- GUI configuration follows the top-level sibling `window`/`plugin` format consumed by Gazebo GUI Application.
- Runtime package availability on Ubuntu 26.04 must be checked in the user's configured official repositories.
- Both runtime environments remain **NOT_RUN** in this build environment. Static XML inspection is not a substitute for SDFormat parser or Gazebo execution.

## Acceptance still outstanding

1. Survey-grade left/right boundaries, grid locations and road elevation to meet a literal “identical to the real circuit” criterion.
2. SDF parser and server/GUI execution on both target operating systems.
3. Dynamic contact tests with the user's future vehicle and GPU LiDAR rendering tests.

These are recorded as unverified rather than marked PASS.
