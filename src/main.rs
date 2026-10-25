//! The data the book's figures draw from. Two commands:
//!
//!     cargo run --release -- run runs/openarm-dead book/runs/openarm-dead.js
//!     cargo run --release -- model ../drydock/examples/openarm/robot.yml book/models/openarm.js
//!
//! `model` writes a robot's body for the 3D views (see `model.rs`). `run`
//! turns a recorded run of keel driving drydock into what the figures and
//! the playback page show:
//!
//! It's a script, `KEEL_RUNS["openarm-dead"] = {…}`, not JSON: the book
//! loads it with a `<script>` tag, which works from disk as well as from a
//! server, where fetching JSON from `file://` doesn't.
//!
//! Beside it, `openarm-dead-wire.js` holds the whole run at full rate, for the
//! page that plays it back: every frame on the bus, when each state and
//! command was published, and when the joints' flags changed (about 2 MB).
//!
//! The run's directory holds what keel and drydock wrote while it ran (the
//! README says how to record one):
//!
//! - `recording.keel`: `keel-recorder`'s file, the arm's states and the
//!   controller's commands, every one.
//! - `trace.json`: `keel trace --export`, the sampled traces: when each
//!   message was published, taken and released, by whom.
//! - `bus.jsonl`: `drydock --trace`, every frame that crossed the bus.
//! - `keel.log`: what `keel run` printed.
//! - `dataflow.yml`: the dataflow that ran.
//!
//! keel's traces and drydock's frames are both on the machine's monotonic
//! clock, so they line up as they were.

use std::collections::{BTreeMap, HashMap};
use std::error::Error;
use std::fs;
use std::io::{BufRead, BufReader};
use std::path::Path;

use keel_control::{Command, State};
use keel_record::Reader;
use serde::Deserialize;
use serde_json::{json, Value};

/// The figures' replay is drawn at 100 Hz: every fifth state at 500 Hz.
const EVERY: usize = 5;
/// The bus's load, averaged over this.
const LOAD_WINDOW_NS: u64 = 20_000_000;

mod model;

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Deserialize)]
struct Frame {
    id: u32,
    from: String,
    data: String,
    ready_ns: u64,
    start_ns: u64,
    end_ns: u64,
}

/// A message's way to one receiver, from a sampled trace, in µs on the
/// monotonic clock.
#[derive(Default, Clone)]
struct Hop {
    node: String,
    input: String,
    published: f64,
    taken: f64,
    released: f64,
}

const USAGE: &str = "usage: keel-book run <run directory> <out.js>
       keel-book model <drydock robot.yml> <out.js>";

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match &args[..] {
        [cmd, dir, out] if cmd == "run" => run(Path::new(dir), out),
        [cmd, robot, out] if cmd == "model" => model::write(Path::new(robot), Path::new(out)),
        _ => Err(USAGE.into()),
    }
}

fn run(dir: &Path, out: &str) -> Result<()> {
    let log = fs::read_to_string(dir.join("keel.log"))?;
    let (states, commands) = recording(&dir.join("recording.keel"))?;
    let t0 = states.first().ok_or("no states in the recording")?.0;
    let motors = motors(&log);
    let s = |ns: u64| round((ns as f64 - t0 as f64) / 1e9, 4);

    // The replay: positions, targets, torques, statuses.
    let mut series: BTreeMap<&str, Vec<Value>> = BTreeMap::new();
    let mut c = 0;
    for (i, (t, _, state)) in states.iter().enumerate() {
        while c + 1 < commands.len() && commands[c + 1].0 <= *t {
            c += 1;
        }
        if i % EVERY != 0 {
            continue;
        }
        let told = (commands.first().is_some_and(|first| first.0 <= *t)).then(|| &commands[c].2);
        series.entry("t").or_default().push(json!(s(*t)));
        series.entry("position").or_default().push(json!(state.iter().map(|j| round(j.position, 3)).collect::<Vec<_>>()));
        series.entry("effort").or_default().push(json!(state.iter().map(|j| round(j.effort, 2)).collect::<Vec<_>>()));
        series.entry("flags").or_default().push(json!(state.iter().map(|j| j.status.flags).collect::<Vec<_>>()));
        series.entry("drive").or_default().push(json!(state.iter().map(|j| j.status.drive).collect::<Vec<_>>()));
        let target = told.map(|cs| cs.iter().map(|c| round(c.position, 3)).collect::<Vec<_>>());
        let asked = told.map(|cs| cs.iter().zip(state).map(|(c, j)| round(c.torque(*j), 2)).collect::<Vec<_>>());
        series.entry("target").or_default().push(json!(target));
        series.entry("asked").or_default().push(json!(asked));
    }

    // The bus, frame by frame, and how busy it was.
    // The trace's frames; its other lines say what drydock broke, and when.
    let mut frames: Vec<Frame> = Vec::new();
    for line in BufReader::new(fs::File::open(dir.join("bus.jsonl"))?).lines() {
        let line = line?;
        if line.contains("\"bus\":") {
            frames.push(serde_json::from_str(&line)?);
        }
    }
    let mut busy: BTreeMap<u64, u64> = BTreeMap::new();
    for f in frames.iter().filter(|f| f.start_ns >= t0) {
        *busy.entry((f.start_ns - t0) / LOAD_WINDOW_NS).or_default() += f.end_ns - f.start_ns;
    }
    let last = states.last().map_or(0, |s| (s.0 - t0) / LOAD_WINDOW_NS);
    let load: Vec<f64> = (0..=last).map(|w| round(*busy.get(&w).unwrap_or(&0) as f64 / LOAD_WINDOW_NS as f64, 3)).collect();

    // The sampled cycles: a state's way to the controller, the command's way
    // back, the frames around it, and what the state and the command said.
    let by_span: HashMap<u64, usize> = states.iter().enumerate().map(|(i, s)| (s.1, i)).collect();
    let command_by_span: HashMap<u64, usize> = commands.iter().enumerate().map(|(i, c)| (c.1, i)).collect();
    let mut cycles = Vec::new();
    for (state_span, command_span, hops) in traces(&dir.join("trace.json"))? {
        let (Some(&si), Some(&ci)) = (by_span.get(&state_span), command_by_span.get(&command_span)) else { continue };
        let published = hops[0].published;
        let at = |us: f64| round(us - published, 2);
        let pub_ns = (published * 1e3) as u64;
        let around: Vec<Value> = frames
            .iter()
            .filter(|f| f.ready_ns + 2_000_000 >= pub_ns && f.ready_ns <= pub_ns + 2_000_000)
            .map(|f| {
                let us = |ns: u64| round((ns as f64 - pub_ns as f64) / 1e3, 2);
                json!([f.id, f.from == "motor", us(f.ready_ns), us(f.start_ns), us(f.end_ns), f.data])
            })
            .collect();
        cycles.push(json!({
            "t": s(states[si].0),
            "seq": si,
            "hops": hops.iter().map(|h| json!({
                "node": h.node, "input": h.input,
                "published": at(h.published), "taken": at(h.taken), "released": at(h.released),
            })).collect::<Vec<_>>(),
            "state": states[si].2.iter().map(|j| json!([
                round(j.position, 4), round(j.velocity, 4), round(j.effort, 3), j.status.flags, j.status.drive
            ])).collect::<Vec<_>>(),
            "command": commands[ci].2.iter().map(|c| json!([
                round(c.position, 4), round(c.velocity, 4), round(c.kp, 2), round(c.kd, 3), round(c.effort, 3)
            ])).collect::<Vec<_>>(),
            "frames": around,
        }));
    }

    let first_command = commands.first().map_or(t0, |c| c.0);
    let data = json!({
        "hz": 500,
        "joints": names(&fs::read_to_string(dir.join("dataflow.yml"))?),
        "motors": motors,
        "series": series,
        "load": { "window_s": LOAD_WINDOW_NS as f64 / 1e9, "busy": load },
        "logs": logs(&log, s(first_command)),
        "cycles": cycles,
    });
    let name = Path::new(out).file_stem().and_then(|s| s.to_str()).ok_or("the out file needs a name")?;
    let script = format!(
        "// Written by keel-book from a recorded run; see the README.\n\
         (window.KEEL_RUNS = window.KEEL_RUNS || {{}})[{}] = {};\n",
        serde_json::to_string(name)?,
        serde_json::to_string(&data)?
    );
    fs::write(out, script)?;

    // The whole run, for playing it back. Times in tenths of a µs from the
    // first state, each the difference from the one before (they grow, and
    // small numbers are short); frames' data as indexes into the distinct
    // payloads, which repeat a lot.
    let tenth = |ns: u64| ((ns as i64 - t0 as i64) as f64 / 100.0).round() as i64;
    let deltas = |xs: &mut dyn Iterator<Item = i64>| {
        let mut last = 0;
        xs.map(|x| {
            let d = x - last;
            last = x;
            d
        })
        .collect::<Vec<_>>()
    };
    let on_wire: Vec<&Frame> = frames.iter().filter(|f| f.id != 0x7FF && f.ready_ns + 10_000_000 >= t0).collect();
    let mut payloads: Vec<&str> = Vec::new();
    let mut index: HashMap<&str, usize> = HashMap::new();
    let data_index: Vec<usize> = on_wire
        .iter()
        .map(|f| {
            *index.entry(f.data.as_str()).or_insert_with(|| {
                payloads.push(f.data.as_str());
                payloads.len() - 1
            })
        })
        .collect();
    let mut flag_changes = Vec::new();
    let mut was: Vec<u32> = vec![0; states.first().map_or(0, |s| s.2.len())];
    for (t, _, state) in &states {
        for (j, joint) in state.iter().enumerate() {
            if joint.status.flags != was[j] {
                was[j] = joint.status.flags;
                flag_changes.push(json!([s(*t), j, joint.status.flags]));
            }
        }
    }
    let wire = json!({
        "unit_us": 0.1,
        "frames": {
            "id": on_wire.iter().map(|f| f.id).collect::<Vec<_>>(),
            "ready": deltas(&mut on_wire.iter().map(|f| tenth(f.ready_ns))),
            "wait": on_wire.iter().map(|f| tenth(f.start_ns) - tenth(f.ready_ns)).collect::<Vec<_>>(),
            "dur": on_wire.iter().map(|f| tenth(f.end_ns) - tenth(f.start_ns)).collect::<Vec<_>>(),
            "data": data_index,
            "payloads": payloads,
        },
        "states": deltas(&mut states.iter().map(|s| tenth(s.0))),
        "commands": deltas(&mut commands.iter().map(|c| tenth(c.0))),
        "flags": flag_changes,
    });
    let wire_out = Path::new(out).with_file_name(format!("{name}-wire.js"));
    fs::write(
        &wire_out,
        format!(
            "// Written by keel-book from a recorded run; see the README.\n\
             (window.KEEL_WIRE = window.KEEL_WIRE || {{}})[{}] = {};\n",
            serde_json::to_string(name)?,
            serde_json::to_string(&wire)?
        ),
    )?;
    eprintln!(
        "wrote {out}: {} states, {} sampled cycles, {} frames on the bus",
        states.len(),
        cycles.len(),
        frames.len()
    );
    Ok(())
}

type Timed<T> = (u64, u64, Vec<T>);
type Recorded = (Vec<Timed<State>>, Vec<Timed<Command>>);

/// Every state and command, with when it was published and its span.
fn recording(path: &Path) -> Result<Recorded> {
    let mut reader = Reader::open(path)?;
    let channel = |input: &str| reader.header.channels.iter().position(|c| c.input == input).map(|i| i as u16);
    let (state, command) = (channel("state").ok_or("no state input")?, channel("command").ok_or("no command input")?);
    let (mut states, mut commands) = (Vec::new(), Vec::new());
    while let Some(r) = reader.next_record()? {
        if r.channel == state {
            states.push((r.t_ns, r.span, State::read(&r.payload).collect()));
        } else if r.channel == command {
            commands.push((r.t_ns, r.span, Command::read(&r.payload).collect()));
        }
    }
    Ok((states, commands))
}

/// From `keel trace --export`'s Chrome trace: each sampled cycle's state span,
/// command span, and hops (the state's first, then the command's).
fn traces(path: &Path) -> Result<Vec<(u64, u64, Vec<Hop>)>> {
    let v: Value = serde_json::from_str(&fs::read_to_string(path)?)?;
    let events = v["traceEvents"].as_array().ok_or("no traceEvents")?;
    let mut threads = HashMap::new();
    for e in events.iter().filter(|e| e["name"] == "thread_name") {
        threads.insert(e["tid"].as_u64().unwrap_or(0), e["args"]["name"].as_str().unwrap_or("").to_owned());
    }
    // trace -> span -> node -> hop
    let mut by_trace: BTreeMap<u64, BTreeMap<u64, BTreeMap<String, Hop>>> = BTreeMap::new();
    for e in events {
        let (Some(trace), Some(span)) = (e["args"]["trace"].as_u64(), e["args"]["span"].as_u64()) else { continue };
        let (ts, dur) = (e["ts"].as_f64().unwrap_or(0.0), e["dur"].as_f64().unwrap_or(0.0));
        let thread = threads.get(&e["tid"].as_u64().unwrap_or(0)).cloned().unwrap_or_default();
        let node = thread.trim_end_matches(" inbox").to_owned();
        let hop = by_trace.entry(trace).or_default().entry(span).or_default().entry(node.clone()).or_default();
        hop.node = node;
        match e["cat"].as_str() {
            Some("transit") => {
                hop.input = e["name"].as_str().unwrap_or("").to_owned();
                (hop.published, hop.taken) = (ts, ts + dur);
            }
            Some("processing") => hop.released = ts + dur,
            _ => {}
        }
    }
    let mut cycles = Vec::new();
    for spans in by_trace.values() {
        let [(state, to_state), (command, to_command)] = spans.iter().take(2).collect::<Vec<_>>()[..] else { continue };
        if !to_state.contains_key("controller") || !to_command.contains_key("arm") {
            continue;
        }
        let order = |m: &BTreeMap<String, Hop>| {
            let mut hops: Vec<Hop> = m.values().cloned().collect();
            hops.sort_by(|a, b| a.taken.total_cmp(&b.taken));
            hops
        };
        let mut hops = order(to_state);
        hops.extend(order(to_command));
        cycles.push((*state, *command, hops));
    }
    Ok(cycles)
}

/// From keel-can's lines: `motor 0x01: MIT mode, ±12.5 rad, ±45 rad/s, ±54 N m, answering on 0x11`.
fn motors(log: &str) -> Vec<Value> {
    log.lines()
        .filter_map(|l| l.split_once("] motor 0x")?.1.split_once(": MIT mode, ±"))
        .filter_map(|(id, rest)| {
            let id = u32::from_str_radix(id, 16).ok()?;
            let nums: Vec<f64> = rest.split(['±', ',', ' ']).filter_map(|w| w.parse().ok()).collect();
            let answer = u32::from_str_radix(rest.rsplit_once("0x")?.1.trim(), 16).ok()?;
            Some(json!({ "id": id, "answer": answer, "p": nums[0], "v": nums[1], "t": nums[2] }))
        })
        .collect()
}

/// The joints' names: the dataflow's `--names` for its keel-can node.
fn names(dataflow: &str) -> Vec<String> {
    let mut lines = dataflow.lines().skip_while(|l| !l.trim().ends_with("--names"));
    let names = lines.nth(1).unwrap_or("").trim().trim_start_matches("- ");
    names.split(',').map(str::to_owned).collect()
}

/// The arm's lines, which say what's wrong: `[arm] 5.8s joint 4 …`. Their
/// times count from the first command.
fn logs(log: &str, first_command_s: f64) -> Vec<Value> {
    log.lines()
        .filter_map(|l| l.strip_prefix("[arm] "))
        .filter_map(|l| {
            let (t, text) = l.split_once("s ")?;
            let t: f64 = t.parse().ok()?;
            Some(json!({ "t": round(first_command_s + t, 3), "node": "arm", "text": text }))
        })
        .collect()
}

fn round(x: f64, places: i32) -> f64 {
    let k = 10f64.powi(places);
    (x * k).round() / k
}
