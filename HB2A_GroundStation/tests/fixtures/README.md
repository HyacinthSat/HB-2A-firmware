# Recorded Control Regression

`hssup-impulsive.soft.f32` is the unmodified little-endian float32 soft
decision record from session `20261007-035918-598400`, event 38
(`rx-000038.soft.f32`). It contains a CodeID 1, depth 1 control frame
with master/virtual counters 5/5 and text `HSSUP,3F,0F,3F`.

The original Viterbi/RS path fails on this sample. A bounded retry with
soft magnitudes capped at 0.5 recovers all 223 expected TM bytes. This
fixture does not establish receiver sensitivity or explain the physical
source of the discriminator outliers.
