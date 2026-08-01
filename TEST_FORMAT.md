# Test Format — the standard shape for every hardware test

_Adopted 2026-08-01._

Every hardware test in this repo is written the same way. The point is not
tidiness: it is that a test you can't abort, whose pass condition you didn't
write down in advance, and whose "why" you don't understand, is not a test —
it is just switching the robot on and hoping.

Powered motion on a self-balancing robot is not free. A test that fails
informatively is worth ten that fail confusingly.

---

## THE TEMPLATE

Copy this. Fill every section. If a section is genuinely empty, say so —
don't delete it, because a missing "Abort" reads identically to a forgotten one.

````markdown
## TEST N — <short name>

**Proves:** one sentence. The single claim this test settles. If you can't
write it in one sentence, it is two tests.

**Rig:** stand or ground. Tethered? Legs collapsed or extended? Where are
your hands? What is the drop height if it goes over?

**Abort:** the exact keystroke or switch, and what the robot does when you
use it. Written BEFORE the commands, because that is when you read it.

**Prerequisites:** which earlier tests must be green. Name them.

### Commands

```bash
# terminal A — what this one is for
<command>
```
One terminal per block, labelled. Copy-pasteable, no placeholders left in.
Every non-obvious flag gets a trailing comment saying why it is there.

### Expected

| Action | Expected |
|---|---|
| what you do | what the robot does |

Written BEFORE running. This is the whole discipline — a prediction made
afterwards is a rationalisation.

### Failure modes

| Symptom | Cause | Fix |
|---|---|---|

Include the failures that look like success, and the ones that look like a
different problem than they are. Those are the expensive ones.

### Why this works

Prose. The mechanism, the sign chain, the units. Enough that you could
re-derive the expected table without being told it.

### Pass criteria

Explicit and checkable. "It seemed fine" is not a pass. State the duration
too — "holds 30 s unaided" is a criterion, "holds up" is not.

### On pass

What to record (measured values into which file), and which test is next.
````

---

## THE RULES BEHIND THE TEMPLATE

1. **Abort before commands, always.** You read the top of a document while
   calm and the middle of it while the robot is falling.

2. **Predict before you run.** The Expected table is written in advance. Its
   real job is not to tell you what happens — it is to make you notice when
   something else does.

3. **One variable per run.** Change one gain, by no more than ~50%, then
   re-run. Two changes and a good result teaches you nothing.

4. **Verify at RUNTIME, never from the file.** `ros2 param get /node param`.
   The yaml, the launch override and the CLI override disagree often enough
   that reading the file proves nothing, and a wrong param usually fails
   silently rather than loudly. This has cost real hours on this project.

5. **State units in the command, not just in your head.** Degrees-vs-radians
   and motor-vs-output turns are the two recurring traps here, and both fail
   as confident, plausible, wrong numbers rather than as errors.

6. **Low limits first.** Under-torque means sag; over-torque means broken
   printed parts. Be wrong in the safe direction, then raise deliberately.

7. **Record the result in the repo the same day.** A measured value that
   lives only in a terminal scrollback is not measured.

---

## WHY THE "Why this works" SECTION IS NOT OPTIONAL

The goal of this project is independent capability, not a working robot —
the robot is the proof, not the point. A runbook you can follow without
understanding produces a robot you cannot debug when it is 11pm and the
runbook doesn't cover what just happened.

If you cannot write that section, you do not yet know what the test proves,
and running it will produce a result you cannot interpret.
