# SteadyHand V3.9 with direct screws in the spoon carriage

V3.9 keeps the corrected left-side servo wire opening and changes the spoon
carriage connection so it no longer requires heat-set inserts. The two former
4.6 mm insert pockets are now 2.6 mm pilot holes for ordinary M3 screws.

The handle correction was developed after inspecting a physical-fit
photo. The black rubber strain-relief was hitting the mounting wall before the
servo case could seat. The user confirmed that this interference is on the
left, so the lower-left mounting point is replaced by the wire-boot opening.
The servo lead is permanently attached, so clearance behind the servo was not
enough by itself: the lead also needs an open route through the mounting plate.

## V3.9 spoon-carriage change

- The red pitch-horn adapter retains 3.4 mm pass-through clearance holes.
- The yellow moving spoon carriage now has two 2.6 mm pilot holes.
- Use two M3 x 10 mm machine screws through the adapter. They form threads
  directly in the printed carriage bosses; no inserts or nuts are required.
- Do not drill the 2.6 mm carriage holes larger and do not overtighten them.

## Retained handle changes

- An 18 x 10 mm opening now occupies the lower-left servo mounting point.
- That opening overlaps the servo body opening and reaches the plate's side
  edge, allowing the black rubber boot and cable to enter without bending.
- The roll servo is fastened using the other three DS215 mounting holes. A 6 mm
  protected zone around each retained screw has been checked against the slot.
- Handle height is 34 mm: 2 mm more above and 2 mm more below the centerline.
- A dedicated 12 x 12 x 12 mm service envelope now clears the roll-servo rubber
  boot and the cable's first bend, not only the metal case.
- A small local side blister preserves wall thickness around that cable pocket.
- Front harness opening increased from 18 x 6 mm to 24 x 10 mm.
- Rear cable opening increased from 16 x 10 mm to 20 x 12 mm.
- Pitch-servo wire passage increased from 10 x 6 mm to 14 x 9 mm.

## Printable parts

1. `V3_9_DS215_fit_test_coupon.stl` - optional quick body/hole-pattern check.
2. `V3_9_handle_tray.stl` - includes the three-hole servo mount and left wire opening,
   Nucleo rails/zip-tie slots, and four handle-IMU standoffs.
3. `V3_9_handle_lid.stl` - flat four-screw lid with no tray interference.
4. `V3_9_integrated_outer_frame.stl` - roll yoke and pitch-servo bracket are one
   printed solid; there is no unexplained bracket-to-frame joint.
5. `V3_9_integrated_moving_carriage.stl` - pitch platform and spoon socket are one
   printed solid, now with two direct-screw M3 pilot holes.
6. `V3_9_pitch_horn_adapter.stl` - removable bridge from the purchased pitch
   horn to the moving carriage. This makes the center horn screw accessible.

## Verified interfaces

- KST DS215MG V8: 23 x 12 x 27.5 mm case, 32 x 12 mm ear envelope,
  four 2.5 mm holes on a 28 x 7 mm pattern, 25T/5 mm spline.
- KST-5034 one-arm horn: 28.4 mm overall, 7.5 mm wide, 1.8 mm arm,
  four 1.6 mm holes at 10, 13.5, 17, and 20.5 mm from the spline axis.
  The design uses the 10 and 17 mm holes at both powered joints.
- The round-looking 5 mm output is actually a 25-tooth spline. V3.9 shows
  visible conceptual teeth and the center retaining screw so the joint is not
  mistaken for a smooth pin. The tooth shape is reference-only because KST
  publishes 25T/5 mm but not the manufacturing tooth profile.
- Adafruit MPU6050 #3886: 25.4 x 17.78 mm nominal PCB outline in the Eagle
  source; four 2.5 mm holes on a 20.32 x 12.70 mm pattern.
- Neutral-position solid interference: zero for electronics, both complete
  servo references (including wire boots), the enlarged roll-wire bend
  envelope, frame, carriage, handle, and closed lid.
- Checked travel: pitch +/-50 degrees and roll +/-45 degrees.

## Required purchased hardware

- 2 x KST-5034 reinforced red plastic horn, 25T-5.
- 2 x horn-center screws supplied with the DS215 servos.
- 4 x M2 x 6-8 mm thread-forming screws for the horn-to-print connections.
- 2 x M3 x 10 mm machine screws for the pitch-adapter-to-spoon-carriage joint.
- 1 x M3 heat-set insert and one M3 pivot screw/washer for the passive pivot.

The red `REFERENCE_KST_5034...` files are purchased-part references only. Do
not print a plastic spline substitute. Print the DS215 coupon first, then
dry-fit both actual horns before printing or assembling the full mechanism.

## What attaches to what

The horn does **not** grip a smooth printed round shaft.

1. The horn's toothed center presses onto the servo's metal 25T/5 mm spline.
   The matching teeth transmit torque.
2. The small screw supplied with the servo goes through the horn center and
   threads into the servo shaft. It prevents the horn sliding off.
3. Two M2 screws through the horn's 10 and 17 mm arm holes connect it to the
   printed orange roll frame or the red pitch adapter.
4. The pitch adapter bolts to the yellow moving carriage with two M3 screws.
5. The round yellow feature on the opposite side is only the passive support
   pivot. It does not receive a servo horn and does not drive the carriage.

## Pitch-joint assembly order

1. Bolt the pitch servo to the outer frame.
2. Fasten the KST-5034 horn to `V3_9_pitch_horn_adapter` using the horn's 10 and
   17 mm holes and the adapter's two 2.2 mm holes.
3. Push the horn onto the yellow 25T servo spline and install KST's center screw
   through the adapter's large center access hole.
4. Place the moving carriage in the frame. Pass two M3 x 10 mm screws through
   the adapter's 3.4 mm holes and slowly drive them into the carriage's 2.6 mm
   pilot holes. Stop when snug; the printed plastic supplies the threads.
5. Install the M3 passive-pivot screw and washer from the left frame cheek.

For the roll joint, fasten the horn to the rear of the orange frame through its
10 and 17 mm holes, push it onto the roll-servo spline, then install the KST
center screw through the large center access opening.

## Roll-servo installation through the corrected mount

1. Put the black rubber strain-relief and attached lead into the 18 x 10 mm
   opening that replaces the lower-left screw position.
2. Slide the servo case into the rectangular opening while keeping the lead in
   the notch; do not clamp the wire under a mounting ear.
3. Install screws through the three remaining servo-ear mounting holes.
4. Route the lead through the internal bend pocket toward the controller.

If the corrected V3.8 handle and mechanism parts are already printed, keep
them. For this screw change, reprint only `V3_9_integrated_moving_carriage.stl`;
the existing V3.8 pitch-horn adapter remains compatible.
