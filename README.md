# keel-book

The book of [keel](https://github.com/montmejat/keel) and
[drydock](https://github.com/montmejat/drydock): how each works, how they fit
together, with figures drawn from real runs.

- `book/`: the book, a static page. Open `book/index.html`; no server or
  build needed. `book/run.html` plays the recorded run back frame by frame.
- `book/runs/`: recorded runs, as the figures read them (written by the tool
  below).
- `book/models/`: robots' bodies for the 3D view, from drydock's robot file and
  its MuJoCo model (OpenArm's is Enactic's, Apache-2.0).
- `src/`: `keel-book`, which turns a run that keel and drydock recorded into
  a file in `book/runs/` (`keel-book run`), and a robot into one in
  `book/models/` (`keel-book model`).
- `dataflows/`: what was run to record them.

keel and drydock are expected beside this repository (`../keel`,
`../drydock`): `keel-book` reads recordings with keel's own crates.

## Recording a run

The figures in "One cycle, end to end" and "A motor dies, replayed" are
`book/runs/openarm-dead.js`; the playback page, `book/run.html`, also reads
`book/runs/openarm-dead-wire.js`, every frame at full rate. Both came from
this. Build keel and drydock
(`cargo build --release` in each), have `vcan0` up (drydock's README), then
from here:

```sh
mkdir -p runs/openarm-dead

# 1. The arm, its elbow dying 6 s in, every frame on the bus to a file.
../drydock/target/release/drydock ../drydock/examples/openarm/robot.yml \
    --scenario ../drydock/examples/openarm/elbow-dies.scenario.yml --trace runs/openarm-dead/bus.jsonl

# 2. In a second terminal: OpenArm's dataflow with a recorder.
../keel/target/release/keel run dataflows/openarm-recorded.yml | tee runs/openarm-dead/keel.log

# 3. A third, after ~13 s: the sampled traces, then stop both.
../keel/target/release/keel trace --export runs/openarm-dead/trace.json
../keel/target/release/keel stop     # and Ctrl-C drydock

# 4. Into the book.
cp "$(ls -t ~/.local/share/keel/recordings/*.keel | head -1)" runs/openarm-dead/recording.keel
cp dataflows/openarm-recorded.yml runs/openarm-dead/dataflow.yml
cargo run --release -- run runs/openarm-dead book/runs/openarm-dead.js
```

`runs/` (the raw recording, ~20 MB) isn't kept; `book/runs/` is.

keel's traces and drydock's bus trace are both on `CLOCK_MONOTONIC`, so a
frame and the message it carried line up to the microsecond. keel samples
one trace every 100 ms, so the cycle figure has about ten a second to pick
from; the replay has every state, drawn at 100 Hz.

## A robot's body

The playback page draws OpenArm in 3D, each joint where its motor says it
is. Its body comes from drydock's robot file and the model it names, which
drydock fetches on its first run:

```sh
cargo run --release -- model ../drydock/examples/openarm/robot.yml book/models/openarm.js
```

Bodies, joints and which motor turns which (through its transmission, for
the gripper), and the collision meshes, thinned to about 0.5 MB.
