# The four simulations, explained simply

One page, human words. Read it once out loud before presenting.

The controller we're testing is always the same — the rule from our base paper
that keeps the drone level and pointed where we want. We never changed its
settings. The only thing that changes is the *world* the drone flies in.
That's the whole experiment: same pilot, four different planets.

---

## 1. MuJoCo — "the precision lab"

**What it is:** A physics simulator famous for being extremely accurate about
how things touch, spin, and fall. Robots labs use it to test ideas before
touching real hardware.

**What we did:** We built our drone from the paper's exact numbers (weight,
shape, motor strength) and let the paper's controller fly it — the step, the
sine wave, and the full 360° backflip — with the paper's own sensor noise
turned on.

**What happened:** It flew everything. The backflip is the star: the drone
starts at 60 m, flips all the way around in about 3 seconds, and never gets
confused — because the controller thinks in quaternions, which don't have the
"90° blind spot" that normal angles have.

**Say it like this:** *"This is the high-accuracy physics lab. The paper's
controller flies all three test maneuvers here, including a full backflip
with zero orientation glitches."*

---

## 2. gym-pybullet-drones — "the second opinion"

**What it is:** A completely different physics engine, the one used by people
training drones with AI. It calculates motion its own way.

**What we did:** The exact same drone, the exact same controller settings,
the exact same three tests. The only thing that changed was the physics
engine underneath.

**What happened:** The results matched MuJoCo almost perfectly — the step
test differed by 0.03 degrees. That's the key insight: when two different
physics worlds give you the same answer, the credit goes to the controller,
not to luck or to one simulator's quirks.

**Say it like this:** *"Different engine, same result — 0.3% apart. That
proves the controller itself is what's working, not one simulator being
nice to us."*

---

## 3. Gazebo — "the robotics standard"

**What it is:** The simulator almost every robotics lab in the world uses.
It's the closest software gets to a real test field.

**What we did:** We set up the full chain a real lab would run: Gazebo's
physics, the connector plugin, and the actual drone-flight software in the
loop. Then the same controller flew the same three tests.

**What happened:** The smooth wave-tracking test matched the other simulators
within 0.3 degrees — real physics, real flight software, same answer. The
aggressive tests (full-tilt step, backflip) hit real physical limits: the
drone starts the maneuver honestly, then the paper's aggressive settings
overpower the physics and it starts rolling. We tested seven different
setups to confirm it's a genuine physical boundary — not a bug we could
tune away.

**Say it like this:** *"On the industry-standard simulator, smooth tracking
matches everything else. The extreme maneuvers expose where the paper's
settings meet real physics — and we proved that boundary with seven
different experiments."*

---

## 4. ArduPilot — "the real thing"

**What it is:** Not really a simulator — it's the actual flight software
that flies real drones. We compiled it from source code ourselves. This is
as close to a real drone as you can get without buying one.

**What we did:** Our controller talked to it over the radio protocol real
drones use — went through the real checklist (switch to guided mode, arm,
take off) — and flew all three tests, three times each.

**What happened:** Two things worth saying. First, it was the BEST at smooth
wave tracking of all four — 14.7° error, better than the physics-only sims.
Second, we caught a famous trap: the flight software politely accepts your
steering commands... and silently ignores them, unless you flip one specific
setting. We caught it because the drone stayed perfectly level while being
told to tilt fully. One switch later, everything worked. We documented that
trap because everyone who tries this hits it.

**Say it like this:** *"The paper's controller flew on the same software
that flies real drones — and it was the best wave-tracker of all four.
Along the way we found and documented a hidden setting that silently
disables steering — a trap the next person will thank us for."*

---

## The one-sentence summary of all four

> Same controller, four worlds: the precision lab and the second opinion
> agree to a fraction of a degree, the industry standard agrees on smooth
> flying and shows exactly where the extreme stuff meets physics, and the
> real flight software turned out to be the best pilot of them all.
