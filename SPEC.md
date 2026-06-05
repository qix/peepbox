# Rotating Rings of Light — Spec

## Overview

There are `NUM_RINGS` (5) concentric rings of LEDs. Each ring has `ringSizes[ring]`
LEDs evenly spaced around a circle. The rings rotate together on a single wheel
driven by a stepper motor. At a rotation of **0°** all the rings are lined up.

Each ring also has `holeCounts[ring]` holes evenly spaced around it. At 0° the
first hole on a ring sits at `firstHoleDeg[ring]` degrees. A light detector is
mounted at a fixed physical position over the **outermost ring (idx = 4)**. When
a lit LED on ring 4 passes under a hole that is currently aligned with the
detector, light reaches the detector and it reads **1**; otherwise it reads **0**.

The goal of startup calibration is to find exactly where 0° is — i.e. to map the
current motor position to the wheel's true rotation.

### Initial position detection

At startup we don't know the wheel's absolute rotation. We use the ring-4
detector together with individually-addressable LEDs to find a known reference.
The key insight: the detector only reads 1 when **a lit LED** lines up with
**a hole** that lines up with **the detector**. By controlling which LED is lit
we can identify exactly which LED is under the detector at the trigger moment,
and from that single known LED position we can back out the wheel's rotation.

Procedure:

1. **Find a hole.** Light up all LEDs, then rotate the wheel until the detector
   reads 1. At this point *some* LED on ring 4 is shining through a hole onto the
   detector — we just don't yet know which one.

2. **Measure the OFF switching time.** Turn off all LEDs and confirm the detector
   reads 0. Record how long it takes for the reading to switch from 1 → 0. This
   is the detector's response/settle time when light is removed.

3. **Measure the ON switching time.** Turn all LEDs back on and confirm the
   detector reads 1 again. Record how long it takes to switch from 0 → 1.

4. **Identify the LED.** Step through the candidate LEDs one at a time (only one
   lit at a time), waiting **double the previously measured switching time**
   between checks so the detector has settled. The LED that makes the detector
   read 1 is the one currently aligned with the detector through a hole. Call it
   the *chosen LED*.

5. **Measure the on-span.** With only the chosen LED lit, the wheel is already
   sitting inside the pulse where that LED lights the detector. Rotate one way
   until the detector reads 0 to find one edge of the pulse, then rotate back the
   other way through the on-span until it reads 0 again to find the opposite edge.
   The midpoint of those two edges is the rotation at which the chosen LED is
   exactly centred under the detector. Reading both edges of *this* pulse (rather
   than walking off it to the next one) keeps the centre tied to the LED we just
   identified.

6. **Compute 0°.** We now know the precise rotation at which a *specific, known*
   LED is centred on the detector. Because the LED's angular position within its
   ring is known (`ringPixel * 360 / ringSizes[4]`) and the detector's physical
   angle is fixed, we can calculate the wheel's true rotation — and therefore
   where 0° is — from the chosen LED's position.

Notes / open questions:

- This relies on the holes and the detector being narrow enough that only one
  LED lights the detector at a time. If multiple LEDs can be under the detector
  simultaneously, step 4 may find several candidates; pick the one whose centred
  rotation (step 5) is closest to the trigger.
- The doubled switching-time interval (step 4) is a guard so each per-LED check
  is read only after the detector has fully settled.
