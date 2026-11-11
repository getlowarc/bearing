# Benchmark results

From `bench/compare.html`: the same scenes through Bearing, Rapier, Jolt and cannon-es in one
page, taking turns in blocks of 10 steps, medians of 3 runs. 1 m boxes of 1,000 kg, friction 0.5,
restitution 0, steps of 1/60 s. Each engine's own solver settings; cannon-es given its
sweep-and-prune broadphase and sleeping, as its docs advise for many bodies. Lower is better for
ms per step; for a stack, less strayed and sank is better, and "stands" means every box ended
within 10 cm of where it started.

To run it: `node serve.js`, open <http://localhost:8080/bench/compare.html>, press Run, and paste
the result here. `?scene=stack,pyramid,pile` runs those only, `&auto=1` starts at once; in a
browser with less memory to spare, run a scene or two per page load.

## Laptop: Intel i7-1260P, Iris Xe

2026-11-11, on the charger, after the speed audit's small-world sort and jointed bodies freezing
(both after 1.5.0): Bearing at commit `b500ef3`, with workers ('auto') and on one thread
(`workers: 0`); Rapier 0.21.0, SIMD build; Jolt 1.1.0, single-threaded; cannon-es 0.20.0. Run in
the Claude desktop app's browser pane (Chrome 156), in view and cross-origin isolated, one page
load per scene or three; the rays alone, and the 10,000-box pile without cannon-es, as before. A
run on the battery the same morning read every engine 2 to 5 times slower and was thrown away.

### 10-box stack, 10 s

| Engine | ms per step | stands | strayed | sank |
|---|---|---|---|---|
| Bearing | 0.0123 | yes | 0.00 mm | 6.57 mm |
| Bearing (1 thread) | 0.0111 | yes | 0.00 mm | 6.57 mm |
| Rapier | 0.00922 | yes | 9.54 mm | 23.9 mm |
| Jolt | 0.00236 | yes | 20.0 mm | 13.1 mm |
| cannon-es | 0.0887 | **no** | 294.9 mm | 49.0 mm |

### 210-box pyramid, 10 s

| Engine | ms per step | stands | strayed | sank |
|---|---|---|---|---|
| Bearing | 0.433 | yes | 0.08 mm | 7.34 mm |
| Bearing (1 thread) | 0.412 | yes | 0.08 mm | 7.34 mm |
| Rapier | 0.0700 | yes | 11.1 mm | 27.3 mm |
| Jolt | 0.0726 | yes | 6.38 mm | 25.3 mm |
| cannon-es | 5.18 | **no** | 320.4 mm | 91.9 mm |

### 1,000-box pile, 10 s

| Engine | ms per step, 0–5 s | ms per step, 5–10 s | asleep at 10 s | fell through |
|---|---|---|---|---|
| Bearing | 6.04 | 4.62 | 101 of 1,000 | 0 |
| Bearing (1 thread) | 10.9 | 8.40 | 101 of 1,000 | 0 |
| Rapier | 1.88 | 1.20 | 1,000 of 1,000 | 0 |
| Jolt | 4.78 | 4.46 | 107 of 1,000 | 0 |
| cannon-es | 19.4 | 21.7 | 451 of 1,000 | 0 |

### 10,000 rays into 1,000 boxes

| Engine | ms for all 10,000 | rays that hit |
|---|---|---|
| Bearing | 33.8 | 10,000 |
| Bearing (1 thread) | 30.7 | 10,000 |
| Rapier | 17.1 | 10,000 |
| Jolt | 239 | 10,000 |
| cannon-es | 462 | 10,000 |

### 32 ragdolls, 8 s

| Engine | ms per step, 0–4 s | ms per step, 4–8 s | widest joint gap at the end | asleep at the end | fell through |
|---|---|---|---|---|---|
| Bearing | 4.80 | 3.18 | 0.00 mm | 66 of 352 | 0 |
| Bearing (1 thread) | 6.71 | 5.46 | 0.00 mm | 66 of 352 | 0 |
| Rapier | 1.13 | 0.952 | 7.08 mm | 11 of 352 | 0 |
| Jolt | 2.27 | 0.00158 | 0.10 mm | 352 of 352 | 0 |
| cannon-es | 5.81 | 4.97 | 5.25 mm | 149 of 352 | 0 |

### 1,000-hull pile, 10 s

| Engine | ms per step, 0–5 s | ms per step, 5–10 s | asleep at the end | fell through |
|---|---|---|---|---|
| Bearing | 11.4 | 1.56 | 1,000 of 1,000 | 0 |
| Bearing (1 thread) | 15.9 | 1.86 | 1,000 of 1,000 | 0 |
| Rapier | 2.97 | 0.230 | 1,000 of 1,000 | 0 |
| Jolt | 6.49 | 0.291 | 1,000 of 1,000 | 0 |
| cannon-es | 232 | 338 | 107 of 1,000 | 0 |

### 10,000-box pile, 1 s

| Engine | ms per step, 0–1 s | asleep at the end | fell through |
|---|---|---|---|
| Bearing | 30.9 | 0 of 10,000 | 0 |
| Bearing (1 thread) | 48.8 | 0 of 10,000 | 0 |
| Rapier | 13.8 | 0 of 10,000 | 0 |
| Jolt | 24.2 | 0 of 10,000 | 0 |

### Since 2026-11-02

Every engine read faster in this run, Rapier and Jolt by 5 to 35% on most scenes, so the page
can't tell this run's machine from what changed in Bearing. What changed is measured pinned in
Node, old against new, in one process: a box resting alone steps in half the time and the 10-box
stack 11 to 13% faster (the colouring's sort), both to the bit, and ragdolls fallen in heaps step
in 4.2 to 6.6 ms rather than 7 to 11 from 8 s on, once their trees freeze, which is after this
page's 8 s. Against Rapier, as a share, the one-thread stack moved from 1.08 to 1.20 times its
time and the ragdolls from 5.5 / 6.0 to 5.9 / 5.7: within the run's drift.

Two figures moved for reasons of their own. The ragdolls end with 66 parts of 352 asleep, not 88:
two ragdolls whose trees froze at about 7 s are frozen, not asleep, at 8 s, and are asleep by
10 s. With workers the stack is now 1.1 times its one-thread time, from 1.3.

Jolt's ragdolls all sleep by about 4 s because its solver loses energy: here the joints are
frictionless, and in Bearing a forearm left hanging swings on, as an ideal hinge does (one still
at 414 mm/s at 40 s), keeping its heap awake. Joint friction stays 0 by default, as in every
engine compared here (Nolan, 2026-11-11); `frictionForce` of 1 N·m on every joint puts all 352
parts to sleep by 8 s.

### Against the goals

- **Faster than cannon-es on every scene:** met with workers (the ragdolls 4.80 / 3.18 against
  5.81 / 4.97 ms). On one thread cannon-es is still 1.10 to 1.15 times faster on the ragdolls,
  holding its joints far looser (5.25 mm against 0.00).
- **100 active ragdolls in 8 ms:** not a scene in this page; measured in Node at about 6 ms with
  workers, 18 to 21 on one thread (milestone 19).
- **Stacks stand wherever they stand in Rapier and Jolt:** met, and held tightest of the four.

Rapier, for the record (no longer a goal): the stack 1.2× on one thread, the rays 1.8×, the
10,000-box pile 2.2× with workers (Jolt's 1.3×), the box pile 3.2 to 3.9×, the hulls 3.8 to 6.8×,
the ragdolls 3.3 to 4.2×, the pyramid 5.9×.

## Earlier run

2026-11-02, on the charger, after pose motors, joint forces and the joints on workers (milestones
17 to 19): Bearing at commit `82b6a59`, with workers ('auto') and on one thread (`workers: 0`);
Rapier 0.21.0, SIMD build; Jolt 1.1.0, single-threaded; cannon-es 0.20.0. Run in the Claude
desktop app's browser pane (Chrome 152), in view and cross-origin isolated, one page load per
scene or three; the rays alone, and the 10,000-box pile without cannon-es, as on 2026-10-29.

### 10-box stack, 10 s

| Engine | ms per step | stands | strayed | sank |
|---|---|---|---|---|
| Bearing | 0.0204 | yes | 0.00 mm | 6.57 mm |
| Bearing (1 thread) | 0.0154 | yes | 0.00 mm | 6.57 mm |
| Rapier | 0.0143 | yes | 9.54 mm | 23.9 mm |
| Jolt | 0.00305 | yes | 20.0 mm | 13.1 mm |
| cannon-es | 0.0976 | **no** | 294.9 mm | 49.0 mm |

### 210-box pyramid, 10 s

| Engine | ms per step | stands | strayed | sank |
|---|---|---|---|---|
| Bearing | 0.381 | yes | 0.08 mm | 7.34 mm |
| Bearing (1 thread) | 0.397 | yes | 0.08 mm | 7.34 mm |
| Rapier | 0.0638 | yes | 11.1 mm | 27.3 mm |
| Jolt | 0.0726 | yes | 6.38 mm | 25.3 mm |
| cannon-es | 4.79 | **no** | 320.4 mm | 91.9 mm |

### 1,000-box pile, 10 s

| Engine | ms per step, 0–5 s | ms per step, 5–10 s | asleep at 10 s | fell through |
|---|---|---|---|---|
| Bearing | 6.53 | 4.97 | 101 of 1,000 | 0 |
| Bearing (1 thread) | 12.7 | 9.33 | 101 of 1,000 | 0 |
| Rapier | 2.37 | 1.36 | 1,000 of 1,000 | 0 |
| Jolt | 5.78 | 4.87 | 107 of 1,000 | 0 |
| cannon-es | 23.1 | 24.1 | 451 of 1,000 | 0 |

### 10,000 rays into 1,000 boxes

From above into the 1,000-box pile as it starts, each the nearest hit within 100 m; each engine's
own nearest-hit query.

| Engine | ms for all 10,000 | rays that hit |
|---|---|---|
| Bearing | 28.9 | 10,000 |
| Bearing (1 thread) | 32.6 | 10,000 |
| Rapier | 18.6 | 10,000 |
| Jolt | 251 | 10,000 |
| cannon-es | 380 | 10,000 |

### 32 ragdolls, 8 s

| Engine | ms per step, 0–4 s | ms per step, 4–8 s | widest joint gap at the end | asleep at the end | fell through |
|---|---|---|---|---|---|
| Bearing | 4.71 | 3.14 | 0.00 mm | 88 of 352 | 0 |
| Bearing (1 thread) | 8.27 | 5.82 | 0.00 mm | 88 of 352 | 0 |
| Rapier | 1.50 | 0.971 | 7.08 mm | 11 of 352 | 0 |
| Jolt | 3.49 | 0.00108 | 0.10 mm | 352 of 352 | 0 |
| cannon-es | 7.95 | 5.06 | 5.25 mm | 149 of 352 | 0 |

### 1,000-hull pile, 10 s

| Engine | ms per step, 0–5 s | ms per step, 5–10 s | asleep at the end | fell through |
|---|---|---|---|---|
| Bearing | 14.1 | 1.72 | 1,000 of 1,000 | 0 |
| Bearing (1 thread) | 18.0 | 1.98 | 1,000 of 1,000 | 0 |
| Rapier | 3.21 | 0.240 | 1,000 of 1,000 | 0 |
| Jolt | 7.07 | 0.286 | 1,000 of 1,000 | 0 |
| cannon-es | 259 | 357 | 107 of 1,000 | 0 |

### 10,000-box pile, 1 s

| Engine | ms per step, 0–1 s | asleep at the end | fell through |
|---|---|---|---|
| Bearing | 37.3 | 0 of 10,000 | 0 |
| Bearing (1 thread) | 82.2 | 0 of 10,000 | 0 |
| Rapier | 21.8 | 0 of 10,000 | 0 |
| Jolt | 34.2 | 0 of 10,000 | 0 |

### Since 2026-10-29

The ragdolls are what moved: with workers 7.73 / 5.45 to 4.71 / 3.14 ms a step, now faster than
cannon-es's 7.95 / 5.06; on one thread about even (7.65 / 5.95 to 8.27 / 5.82), joint forces'
bookkeeping taking back what milestone 19's single-thread work gained. The box scenes, rays and
hulls are within a few percent, as milestones 17 to 19 didn't touch them. The 10,000-box pile with
workers read 37.3 ms (52.0 then), and Rapier and Jolt read 12 to 18% faster on it too: most of
that is the run, not the code. The other engines' ragdolls read 20 to 30% slower than on
2026-10-29, so the ragdoll figures sit high across the board.

### Against the goals

- **Faster than cannon-es on every scene:** met with workers, the ragdolls now included (3.14
  against 5.06 ms in the last 4 s). On one thread cannon-es is still 1.04 to 1.15 times faster on
  the ragdolls, holding its joints far looser (5.25 mm against 0.00).
- **100 active ragdolls in 8 ms:** not a scene in this page; measured in Node at about 6 ms with
  workers, 18 to 21 on one thread (milestone 19).
- **Stacks stand wherever they stand in Rapier and Jolt:** met, and held tightest of the four.

Rapier, for the record (no longer a goal): the stack 1.1× on one thread, the rays 1.6×, the
10,000-box pile 1.7× with workers (Jolt's 1.1×), the box pile 2.8 to 3.7×, the ragdolls 3.1 to
3.2×, the hulls 4.4 to 7.2×, the pyramid 6×.


## Desktop: Ryzen 5 5500

Not run yet.
