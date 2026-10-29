# Route comparison and integration decisions

Compared on October 29, 2026 against `jonatl5/posepilot` main at `8407cb14ac1a5171274565fa4bac3c68301c7bde`. A final refresh found commit `50d18b8446a799b8f5ac984acf1f912b1a31f100`, which publishes the same canonical source plus an offline demo, HTML template, tests and CI. Those additions are retained. Original archive hashes remain in `source-provenance.json`; this integration is based on the newer commit.

| Area | Team route | Earlier local reference | Integrated decision |
|---|---|---|---|
| Demonstration | Stationary printed objects on a wall | ArUco/red card approach | Keep wall prints and YOLO as the product target; markers are bench fixtures only |
| Control | PID/PD baseline and PPO comparison | P baseline and a separate optional PPO environment | Import the team's PID, PPO tools and simulator; publish one active control contract |
| State | 16 features x 4 ticks, 64 inputs | 11 features with another normalization | Preserve `objectpilot-v1-16x4`; never load an 11-input policy |
| Center error | Full-frame center minus 0.5 | Divided by half the image dimensions | All active detectors output the team's normalized Detection record |
| Actions | Forward and yaw, 10 Hz | 20 Hz; optional vertical channel | Keep 10 Hz and zero lateral/vertical outputs |
| Distances | 0.65 m stand-off; 0.40 m minimum | 1.20 m stand-off; 0.65 m minimum | Keep the team simulation settings; qualify real clearances after delivery |
| Sensing | Forward ToF, sequence/validity/age extension | Stock range query and generic adapter | Stock replies are bench measurements and close-range vetoes; unknown sample age cannot authorize near approach |
| Evidence | 30 diagnostic scenarios; 15 original tests | 50 different scenarios; 52 reference tests | Preserve the existing team evidence verbatim; do not publish the reference scores as results of the integrated code |
| Hardware | No qualified flight dispatcher | Experimental operator-gated live adapter | Port watchdog behavior behind an injected sink; keep public camera commands ground-only until hardware qualification |

## Added to the repository

- The team's canonical Python package, simulator, training/evaluation tools, data workflow and firmware preparation sources, with provenance retained.
- Red/ArUco bench adapters, printable fixtures, camera calibration and grounded frame collection migrated from the earlier reference.
- Grounded Tello video/range capture, a shared camera/file approach pipeline, a measured arrival hold and explicit stock-range freshness handling.
- A command dispatcher/watchdog scaffold tested with fake I/O. It has no takeoff command and is not wired to a real flight CLI.
- Additional fault, schema, timestamp, arrival, detector and replay regression tests.
- An English data collection guide, architecture/control explanation, hardware gates, source download and synchronized website/repository PRD.

## Preserved boundaries

The historic PID/PPO CSVs and trajectory describe the original team's Windows run. They are not silently regenerated or relabeled as tests of this integration. The environment keeps its additional ground-truth velocity and wall-clearance success checks; the camera pipeline can only check measured image/range stability. Real stopping drift, optical offsets and radio-loss behavior remain hardware work.

The original 11-input reference code remains in the local working folder as prior experimental work. Its conflicting controller, simulator and live-flight CLI are not shipped as an alternative production route. Useful components were migrated into `src/objectpilot` instead.

Original team authorship is retained through archive provenance and the repository's existing license. Integration commits identify the integrating contributor; they do not claim sole authorship of imported team work.
