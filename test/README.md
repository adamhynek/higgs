# Portable contract tests

These tests compile this checkout's production service implementations directly from `../src` and the public headers in `../include`. They need no Skyrim, SKSE, Havok headers, game files, external fixture checkout, downloaded dependency or network connection. The top-level native plugin project is not built.

Prerequisites: CMake 3.26 or newer, a 64-bit C11/C++20 compiler and its standard library/thread support. CMake 3.26 is required for CTest's explicit empty-suite failure option. The convenience runner additionally needs Python 3.9 or newer and uses only its standard library. On Windows, install Visual Studio 2022 or newer with Desktop development with C++ and CMake tools; the runner discovers CMake bundled with Visual Studio if it is absent from PATH. A Linux compiler with C++20 support can use the same CMake project, but the recorded verification for this package is Windows x64 MSVC 19.51, Release.

From the repository root:

```text
python test/run.py --build-dir ../higgs-contract-build --report ../higgs-contract-result.json
```

Use a generated-output directory outside the checkout. `--cmake PATH` selects a specific CMake executable; `--generator "Visual Studio 17 2022"` or `--generator Ninja` selects an installed generator. Ninja on Windows requires a developer shell with the compiler environment already set. The default is Release; `--config Debug` selects Debug. Tests use explicit checks and fail independently of `NDEBUG`, including under Release. Each CTest case has a 60-second timeout.

CMake and CTest can also be used directly, without Python:

```text
cmake -S test -B ../higgs-contract-build
cmake --build ../higgs-contract-build --config Release
ctest --test-dir ../higgs-contract-build -C Release --output-on-failure
```

On a single-configuration generator such as Ninja or Unix Makefiles, add `-DCMAKE_BUILD_TYPE=Release` when configuring. On Windows the compiler target must be x64. The layout checks intentionally describe the 64-bit ABI.

The runner writes `configure.log`, `build.log`, `ctest.log` and a JSON result (by default `BUILD/result.json`). The result includes commands, return codes, source hashes and qualification scope. It returns nonzero on configuration, compilation, test failure or source changes during the run. Hashes cover the compiled `.cpp` source list, authored test package and a conservative superset of top-level headers. Some stress checks run variable numbers of loop iterations; the suite result is stable, while a total dynamic assertion count is not a useful cross-run metric.

| CTest case | Main contract assertions |
| --- | --- |
| `hand_broker` | Occupancy/unknown fields, priority maps and arbitration, both-hand grants, input ownership, typed operations, callback removal and concurrent callback drainage, owner/epoch/deadline validation, copied C table negotiation and extension-tail guards. |
| `hand_authority` | Exactly-once dispatch, owner/thread/scope fences, session renewal and resizing, child effect cleanup, races, lifecycle deferral, haptic priority/composition/replacement. |
| `shared_rounds` | Native/button/dwell order independence, exactly one selected effect, no replay, native possession, region arbitration, deterministic ties, reentrancy, legacy negotiation and extended-input lifecycle. |
| `native_regions` | Core detector registration/correlation, readiness/validation, callback reentrancy, self-removal and concurrent removal drainage. This case executes `src/main/native_regions.cpp`; its name does not imply a native adapter or game test. |
| `transactions` | Domain/native identities, idempotency, release before mutation, one-shot permits, held handover, commitment versus observed grip/equipment, compensation and unresolved recovery. |
| `equipment_claims` | Unsettled place/instance claims survive grant expiry and client loss; generation cannot rename an unresolved instance; independent instances remain usable; partial/unknown evidence retains claims until settled observation. |
| `inventory_service` | Queued selections, owner/buffer guards, current-selection invalidation, truthful backend capability, pending mutation/result handling, native pins retained after client loss and released at epoch change. Uses the production `InventoryService` with an explicit abstract `InventoryEngine` test backend. |
| `physical_policy` | Passive property resource ownership, revision checks, per-hand grab/finger policy, native cleanup acknowledgement, expiry/tracking/lifecycle fences, C layout/table. |
| `support` | Installed backend capability, pending/applied revisions, freshness, source possession and cleanup ownership across lifecycle changes. |
| `observations` | Copied contact/release/controller observations, numeric validation, buffering and public boundary guards. A narrow private-access seam forces counter exhaustion in the actual core without enormous allocations. |
| `own_contacts` | Copied endpoint identity, relative/closing speed, shared-motion rejection, ownership, manifold/cooldown deduplication, policy revision, freshness, invalidation ticket and expiry, C layout/table. Inputs are synthetic copied service records. |
| `input` | Controller frames, gesture arbitration, hold/edge lifecycle, menu contexts, capture/release and C layout. |
| `hand_parts` | Source/box geometry, invalid numeric inputs, revisions, freshness, ownership, native acknowledgement/cleanup and C layout. |
| `tracking`, `motion` | Production motion history, transformations, tracking validity, age boundaries, concurrent coherent reads and C table consumer. |
| `restrictions` | Overlapping owned leases, legacy isolation, collision precedence, expiry/renewal, capacity, lifecycle, concurrent owners and C table negotiation. |

Test backends provide outcomes at the production core's existing interfaces; they do not replace the tested service implementation. This package does not execute `hand_native.cpp`, `synergy_inventory_native.cpp`, other `*_native.cpp` adapters, the native plugin loader, Skyrim equipment/event processing, extra-data/UID resolution, Havok constraints/collision response, controller hardware, audio or physical haptics. It also does not probe exports from a complete native DLL. Separate extracted adapter fixtures and static project/export audits can cover those boundaries offline; they are distinct evidence and are not counted among these 16 core suites. Native build and in-game behavior require their own checks.

An optional implementation negative control adds a generation-equality condition to `SameInstance` in a new disposable copy of `src/hand_equipment.cpp`. It copies only the required source/header/test closure, requires a successful compilation, then requires CTest exit 8 and the exact unresolved same-item claim assertion. An observation revision must not turn the same instance into an independent resource. Run it with a fresh output directory:

```text
python test/negative-control.py --build-dir ../higgs-contract-negative
```

The original checkout is hashed before and after and is never mutated. `source/` and `build/` beneath the selected output must not already exist; use a new directory for another run. The script writes its mutation, commands and result to `negative-control-result.json`, with separate configure/build/CTest logs. Its own success exit code is 0 only when the intended compiled negative is detected. It accepts the same optional `--cmake` and `--generator` selections as the positive runner.
